import os
import sys
import numpy as np
import random
import pickle
from deap import base, creator, tools, algorithms
from datetime import datetime

from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.simulation.simulation import Simulation
import acsl_pychrono.config.config as Cfg
from acsl_pychrono.control.logging import Logging

# --- Force reload of x8copter each time ---
def import_x8copter_module():
    x8_path = os.path.join(os.path.dirname(__file__), "assets", "vehicles", "x8copter", "x8copter.py")
    module_name = "x8copter"
    import importlib.util
    spec = importlib.util.spec_from_file_location(module_name, x8_path)
    x8_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(x8_module)
    sys.modules[module_name] = x8_module
    return x8_module

# --- GA Config ---
POP_SIZE = 6
N_GEN = 5
MUTPB = 0.3
CXPB = 0.5

# --- Fitness Function ---
def run_individual_simulation(pid_gains):
    try:
        import_x8copter_module()  # ensure x8copter is always available

        kp_x, kd_x, ki_x, kp_y, kd_y, ki_y, kp_z, kd_z, ki_z = pid_gains
        wrapper_batch_dir = generateWrapperBatchDir()

        # Set PID into simulation config
        mis_cfg = Cfg.MissionConfig()
        veh_cfg = Cfg.VehicleConfig()
        veh_cfg.pid_gains_outer_loop = [kp_x, kd_x, ki_x, kp_y, kd_y, ki_y, kp_z, kd_z, ki_z]

        env_cfg = Cfg.EnvironmentConfig()
        wrp_prms = Cfg.WrapperParams()
        mis_cfg.wrapper_batch_dir = wrapper_batch_dir

        sim_cfg = Cfg.SimulationConfig(
            mission_config=mis_cfg,
            vehicle_config=veh_cfg,
            environment_config=env_cfg,
            wrapper_params=wrp_prms
        )

        sim = Simulation(sim_cfg)
        simulateMission(sim, Logging.getGitRepoInfo())

        # Load logs from disk
        with open(os.path.join("simulation_logs", "log_dict.pkl"), "rb") as f:
            log_dict = pickle.load(f)

        pos = np.array(log_dict["position"])
        pos_des = np.array(log_dict["user_defined_position"])

        # Compute RMSE over time
        error = np.linalg.norm(pos - pos_des, axis=1)
        rmse = np.sqrt(np.mean(error ** 2))

        print(f"[✓] PID: {np.round(pid_gains, 3)} | RMSE: {rmse:.4f}")
        return (rmse,)

    except Exception as e:
        print(f"[!] Simulation error: {e}")
        return (1e6,)  # Penalize failed sim

def evaluate_population(population):
    return [run_individual_simulation(ind) for ind in population]

# --- DEAP Setup ---
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()

# Bounds for PID tuning (tune as needed)
KP_RANGE = (0.1, 5.0)
KD_RANGE = (0.1, 5.0)
KI_RANGE = (0.0, 1.0)

def uniform_pid():
    return [random.uniform(*KP_RANGE), random.uniform(*KD_RANGE), random.uniform(*KI_RANGE)]

def create_pid_individual():
    return creator.Individual(uniform_pid() + uniform_pid() + uniform_pid())  # x, y, z

toolbox.register("individual", create_pid_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("evaluate", run_individual_simulation)
toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1.0, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)

# --- Helpers ---
def generateWrapperBatchDir() -> str:
    batch_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("logs", "wrapper", batch_timestamp)

# --- Main GA Loop ---
def run_pid_tuning_ga():
    pop = toolbox.population(n=POP_SIZE)

    # Evaluate initial population
    fits = evaluate_population(pop)
    for ind, fit in zip(pop, fits):
        ind.fitness.values = fit

    for gen in range(N_GEN):
        print(f"\n--- Generation {gen} ---")

        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        # Crossover and mutation
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Re-evaluate invalids
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fits = evaluate_population(invalid_ind)
        for ind, fit in zip(invalid_ind, fits):
            ind.fitness.values = fit

        pop[:] = offspring

        # Print best so far
        best = tools.selBest(pop, 1)[0]
        print(f"[✓] Best PID: {np.round(best, 3)} | RMSE: {best.fitness.values[0]:.4f}")

if __name__ == "__main__":
    run_pid_tuning_ga()
