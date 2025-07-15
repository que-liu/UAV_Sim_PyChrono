import os
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
from typing import List
import pickle
import shutil
import traceback

from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.simulation.simulation import Simulation
import acsl_pychrono.config.config as Cfg
from acsl_pychrono.control.logging import Logging

def generatePIDConfig(pid_gains: list[float], wrapper_batch_dir: str) -> Cfg.SimulationConfig:
    """Create SimulationConfig with custom PID gains."""
    mis_cfg = Cfg.MissionConfig()
    veh_cfg = Cfg.VehicleConfig()
    env_cfg = Cfg.EnvironmentConfig()
    wrp_params = Cfg.WrapperParams(pid_gains=pid_gains)
    mis_cfg.wrapper_batch_dir = wrapper_batch_dir
    mis_cfg.visualization_flag = False
    mis_cfg.wrapper_flag = True

    return Cfg.SimulationConfig(
        mission_config=mis_cfg,
        vehicle_config=veh_cfg,
        environment_config=env_cfg,
        wrapper_params=wrp_params,
    )

def runSimulationWithGains(args: tuple[int, list[float], str, dict]):
    idx, pid_gains, wrapper_batch_dir, git_info = args
    sim_cfg = generatePIDConfig(pid_gains, wrapper_batch_dir)
    sim = Simulation(sim_cfg)
    try:
        simulateMission(sim, git_info)
    except Exception as e:
        print(f"[ERROR] simulateMission crashed for individual {idx}: {e}")
        traceback.print_exc()
    
    # Path to the log file written by simulateMission
    src_log_path = os.path.join(os.getcwd(), "simulation_logs", "log_dict.pkl")
    dst_log_path = os.path.join(wrapper_batch_dir, f"log_dict_{idx}.pkl")

    # Copy the log file to the batch directory with a unique name
    os.makedirs(wrapper_batch_dir, exist_ok=True)
    if os.path.exists(src_log_path):
        shutil.copy2(src_log_path, dst_log_path)
    else:
        print(f"[WARNING] Log file {src_log_path} not found after simulation for individual {idx}.")

def getBatchDir() -> str:
    now = datetime.now()
    return os.path.join("logs", "deap_batch", now.strftime("%Y"), now.strftime("%m"), now.strftime("deap_%Y%m%d_%H%M%S"))

def runParallelPIDSimulations(pid_population: List[List[float]], log_dir: str, max_parallel: int = 8):
    os.makedirs(log_dir, exist_ok=True)
    git_info = Logging.getGitRepoInfo()
    
    args = [(idx, ind, log_dir, git_info) for idx, ind in enumerate(pid_population)]

    with ProcessPoolExecutor(max_workers=max_parallel) as executor:
        executor.map(runSimulationWithGains, args)
