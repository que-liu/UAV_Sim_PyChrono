import os
import pickle
import random
import numpy as np
from datetime import datetime
from deap import base, creator, tools

from run_parallel_pid_deap import runParallelPIDSimulations  

# --- DEAP SETUP ---
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", lambda: random.uniform(0.0, 50.0))
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, 9)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

toolbox.register("mate", tools.cxBlend, alpha=0.4)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=3.0, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)

def evaluate_population(population):
    """
    Run simulations in parallel and compute RMSE for each individual.
    The log files are expected to be saved as: simulation_logs/log_dict_{index}.pkl
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join("simulation_logs", f"ga_batch_{timestamp}")
    os.makedirs(log_dir, exist_ok=True)

    # Run all simulations in parallel
    runParallelPIDSimulations(population, log_dir=log_dir, max_parallel=4)

    results = []
    for i, ind in enumerate(population):
        log_path = os.path.join(log_dir, f"log_dict_{i}.pkl")
        try:
            with open(log_path, "rb") as f:
                log = pickle.load(f)

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

            err = np.linalg.norm(pos_des - pos_act, axis=1)
            rmse = np.sqrt(np.mean(err ** 2))
            results.append((rmse,))
        except Exception as e:
            print(f"[!] Failed to load or compute RMSE for individual {i}: {e}")
            results.append((1e6,))  # Penalize failed sims

    return results

def main():
    pop = toolbox.population(n=10)
    NGEN = 20

    # Evaluate initial population
    print("\n[*] Evaluating initial population...")
    fitnesses = evaluate_population(pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    for gen in range(1, NGEN + 1):
        print(f"\n--- Generation {gen} ---")
        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < 0.7:
                toolbox.mate(child1, child2)
                del child1.fitness.values, child2.fitness.values

        for mutant in offspring:
            if random.random() < 0.2:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        print(f"[*] Evaluating {len(invalid_ind)} new individuals...")
        fitnesses = evaluate_population(invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        pop[:] = offspring

        best = tools.selBest(pop, 1)[0]
        print(f"[✓] Best RMSE: {best.fitness.values[0]:.4f} | Gains: {best}")

    # Save best result
    best = tools.selBest(pop, 1)[0]
    with open("best_pid_gains.pkl", "wb") as f:
        pickle.dump(best, f)
    print("\n[✓] Best individual saved to best_pid_gains.pkl")

if __name__ == "__main__":
    main()
