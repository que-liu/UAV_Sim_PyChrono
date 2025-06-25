import pickle
from run_parallel_pid_deap import generatePIDConfig
from acsl_pychrono.simulation.simulation import Simulation
from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.control.logging import Logging
import os

# Load best gains
with open("best_pid_gains.pkl", "rb") as f:
    best_gains = pickle.load(f)

# Set up log directory
log_dir = "simulation_logs/best_run"
os.makedirs(log_dir, exist_ok=True)

# Generate config
sim_cfg = generatePIDConfig(list(best_gains), log_dir)
sim = Simulation(sim_cfg)
simulateMission(sim, Logging.getGitRepoInfo())

print("Best run simulation complete. Now run plot_fig.py to visualize.")