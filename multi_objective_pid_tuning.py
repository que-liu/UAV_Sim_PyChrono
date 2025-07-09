import os
import pickle
import random
import numpy as np
from datetime import datetime
from deap import base, creator, tools

from run_parallel_pid_deap import runParallelPIDSimulations

# --- DEAP SETUP FOR MULTI-OBJECTIVE ---
# Create multi-objective fitness (minimize all three objectives)
creator.create("FitnessMulti", base.Fitness, weights=(-1.0, -1.0, -1.0))  # minimize all three
creator.create("Individual", list, fitness=creator.FitnessMulti)

toolbox = base.Toolbox()

# Set PID gain bounds
PID_LOWER_BOUND = 0.0
PID_UPPER_BOUND = 100.0

toolbox.register("attr_float", lambda: random.uniform(PID_LOWER_BOUND, PID_UPPER_BOUND))
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, 9)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# bound checking decorator
def check_bounds(min_val, max_val):
    def decorator(func):
        def wrapper(*args, **kwargs):
            # apply the original function, mate or mutate
            offspring = func(*args, **kwargs)
            # Clip the values for each individual
            for ind in offspring:
                for i in range(len(ind)):
                    if ind[i] > max_val:
                        ind[i] = max_val
                    elif ind[i] < min_val:
                        ind[i] = min_val
            return offspring
        return wrapper
    return decorator

toolbox.register("mate", tools.cxBlend, alpha=0.4)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=3.0, indpb=0.2)
toolbox.register("select", tools.selNSGA2)  # NSGA-II for multi-objective

toolbox.decorate("mate", check_bounds(PID_LOWER_BOUND, PID_UPPER_BOUND))
toolbox.decorate("mutate", check_bounds(PID_LOWER_BOUND, PID_UPPER_BOUND))

def evaluate_population(population):
    """
    Run simulations in parallel and compute three cost functions for each individual.
    Returns: [(J_position, J_velocity, J_thrust), ...]
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join("simulation_logs", f"multi_obj_batch_{timestamp}")
    os.makedirs(log_dir, exist_ok=True)

    # Run all simulations in parallel
    runParallelPIDSimulations(population, log_dir=log_dir, max_parallel=4)

    results = []
    for i, ind in enumerate(population):
        log_path = os.path.join(log_dir, f"log_dict_{i}.pkl")
        try:
            with open(log_path, "rb") as f:
                log = pickle.load(f)

            # Extract position and velocity data
            pos_des = np.stack([
                np.array(log["user_defined_position"]["x"]).flatten(),
                np.array(log["user_defined_position"]["y"]).flatten(),
                np.array(log["user_defined_position"]["z"]).flatten()
            ], axis=1)
            pos_act = np.stack([
                np.array(log["position"]["x"]).flatten(),
                np.array(log["position"]["y"]).flatten(),
                np.array(log["position"]["z"]).flatten()
            ], axis=1)
            
            vel_des = np.stack([
                np.array(log["user_defined_velocity"]["x"]).flatten(),
                np.array(log["user_defined_velocity"]["y"]).flatten(),
                np.array(log["user_defined_velocity"]["z"]).flatten()
            ], axis=1)
            vel_act = np.stack([
                np.array(log["velocity"]["x"]).flatten(),
                np.array(log["velocity"]["y"]).flatten(),
                np.array(log["velocity"]["z"]).flatten()
            ], axis=1)
            
            # Extract thrust data (T1 to T8)
            thrust_data = np.stack([
                np.array(log["thrust_motors_N"]["T1"]).flatten(),
                np.array(log["thrust_motors_N"]["T2"]).flatten(),
                np.array(log["thrust_motors_N"]["T3"]).flatten(),
                np.array(log["thrust_motors_N"]["T4"]).flatten(),
                np.array(log["thrust_motors_N"]["T5"]).flatten(),
                np.array(log["thrust_motors_N"]["T6"]).flatten(),
                np.array(log["thrust_motors_N"]["T7"]).flatten(),
                np.array(log["thrust_motors_N"]["T8"]).flatten()
            ], axis=1)

            # Compute cost functions
            J_position, J_velocity, J_thrust = compute_multi_objective_costs(
                pos_act, pos_des, vel_act, vel_des, thrust_data
            )
            
            results.append((J_position, J_velocity, J_thrust))
            print(f"[✓] Individual {i}: J_pos={J_position:.4f}, J_vel={J_velocity:.4f}, J_thrust={J_thrust:.4f}")
            
        except Exception as e:
            print(f"[!] Failed to load or compute costs for individual {i}: {e}")
            results.append((1e6, 1e6, 1e6))  # Penalize failed sims

    return results

def compute_multi_objective_costs(pos_act, pos_des, vel_act, vel_des, thrust_data):
    """
    Compute the three cost functions: J_position, J_velocity, J_thrust
    Normalize costs to bring them onto similar ranges.
    Add fallback for NaN/Inf.
    """
    # Weight matrices
    Weight_position = np.diag([3.0, 3.0, 1.0])  # diag(3, 3, 1)
    Weight_velocity = np.diag([0.1, 0.1, 0.1])  # diag(0.1, 0.1, 0.1)
    Weight_thrust = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # diag(1,1,1,1,1,1,1,1)
    
    # Compute errors over time
    pos_error = pos_act - pos_des  # Shape: (time_steps, 3)
    vel_error = vel_act - vel_des  # Shape: (time_steps, 3)
    
    # Compute cost functions (average over time)
    J_position = np.mean(np.sqrt(np.sum((pos_error @ Weight_position) * pos_error, axis=1)))
    J_velocity = np.mean(np.sum((vel_error @ Weight_velocity) * vel_error, axis=1))
    thrust_norms = np.sum((thrust_data @ Weight_thrust) * thrust_data, axis=1)
    J_thrust = np.mean(thrust_norms)

    # add a hard performance constraint
    # If position error is unacceptably high, penalize heavily
    MAX_ACCEPTABLE_POS_ERROR = 0.5  # 50 cm
    if J_position > MAX_ACCEPTABLE_POS_ERROR:
        return 999., 999., 999.

    # --- Normalization ---
    # These divisors are chosen based on typical value ranges; adjust as needed for your system
    J_position_norm = J_position / 1.0  # typical position error < 1 m
    J_velocity_norm = J_velocity / 0.01  # typical velocity error < 0.01
    J_thrust_norm = J_thrust / 1000.0  # typical thrust cost ~100

    # --- Fallback for NaN/Inf ---
    if (np.isnan(J_position_norm) or np.isnan(J_velocity_norm) or np.isnan(J_thrust_norm) or
        np.isinf(J_position_norm) or np.isinf(J_velocity_norm) or np.isinf(J_thrust_norm)):
        return 999., 999., 999.

    return J_position_norm, J_velocity_norm, J_thrust_norm

def main():
    # Configuration
    POP_SIZE = 100
    NGEN = 40
    CXPB = 0.8 # Crossover probability
    MUTPB = 0.3  # Mutation probability
    
    print("=== Multi-Objective PID Tuning with NSGA-II ===")
    print(f"Population size: {POP_SIZE}")
    print(f"Generations: {NGEN}")
    print(f"Crossover probability: {CXPB}")
    print(f"Mutation probability: {MUTPB}")
    print("Cost functions: J_position, J_velocity, J_thrust")
    print("=" * 50)
    
    # Create initial population
    pop = toolbox.population(n=POP_SIZE)
    
    # Evaluate initial population
    print("\n[*] Evaluating initial population...")
    fitnesses = evaluate_population(pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
    
    # Track Pareto front
    pareto_front = tools.ParetoFront()
    
    # Multi-objective optimization with NSGA-II
    for gen in range(NGEN):
        print(f"\n--- Generation {gen + 1}/{NGEN} ---")
        
        # Select and clone the next generation individuals
        offspring = map(toolbox.clone, toolbox.select(pop, len(pop)))
        offspring = list(offspring)
        
        # Apply crossover and mutation
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values
        
        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values
        
        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        if invalid_ind:
            print(f"[*] Evaluating {len(invalid_ind)} new individuals...")
            fitnesses = evaluate_population(invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit
        
        # Replace population
        pop[:] = offspring
        
        # Update Pareto front
        pareto_front.update(pop)
        
        # Print current best
        if pareto_front:
            best_ind = pareto_front[0]
            print(f"[✓] Best individual: {best_ind.fitness.values} | Gains: {best_ind}")
    
    # Save Pareto front solutions
    print(f"\n[✓] Found {len(pareto_front)} Pareto optimal solutions")
    
    # Save all Pareto optimal solutions
    pareto_solutions = []
    for i, ind in enumerate(pareto_front):
        solution = {
            'gains': list(ind),
            'fitness': ind.fitness.values,
            'index': i
        }
        pareto_solutions.append(solution)
        
        # Save individual solution
        with open(f'multi_obj_solution_{i}.pkl', 'wb') as f:
            pickle.dump(solution, f)
        
        # Save gains as text file
        with open(f'multi_obj_gains_{i}.txt', 'w') as f:
            f.write(f'Multi-Objective Solution {i}:\n')
            f.write(f'Gains: {list(ind)}\n')
            f.write(f'J_position: {ind.fitness.values[0]:.6f}\n')
            f.write(f'J_velocity: {ind.fitness.values[1]:.6f}\n')
            f.write(f'J_thrust: {ind.fitness.values[2]:.6f}\n')
    
    # Save all Pareto solutions
    with open('multi_obj_pareto_front.pkl', 'wb') as f:
        pickle.dump(pareto_solutions, f)
    
    print(f"[✓] Pareto front saved to multi_obj_pareto_front.pkl")
    print(f"[✓] Individual solutions saved as multi_obj_solution_*.pkl")
    print(f"[✓] Gain files saved as multi_obj_gains_*.txt")
    
    # Print summary of Pareto front
    print(f"\n=== Pareto Front Summary ===")
    for i, solution in enumerate(pareto_solutions):
        print(f"Solution {i}: J_pos={solution['fitness'][0]:.4f}, J_vel={solution['fitness'][1]:.4f}, J_thrust={solution['fitness'][2]:.4f}")

if __name__ == "__main__":
    main() 