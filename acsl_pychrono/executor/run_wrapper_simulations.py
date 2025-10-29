import os
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor

from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.simulation.simulation import Simulation
import acsl_pychrono.config.config as Cfg
from acsl_pychrono.control.logging import Logging

def runWrapperSimulation(sim_cfg: Cfg.SimulationConfig, git_info: dict | None = None):
  """Run a single wrapper simulation from a given configuration."""
  sim = Simulation(sim_cfg)
  simulateMission(sim, git_info)

def generateConfigForDensity(ball_density: float, wrapper_batch_dir: str) -> Cfg.SimulationConfig:
  """Generate a 'SimulationConfig' with the specified ball density."""
  mis_cfg = Cfg.MissionConfig()
  veh_cfg = Cfg.VehicleConfig()
  env_cfg = Cfg.EnvironmentConfig()
  wrp_prms = Cfg.WrapperParams()

  wrp_prms.my_ball_density = ball_density
  mis_cfg.wrapper_batch_dir = wrapper_batch_dir

  sim_cfg = Cfg.SimulationConfig(
    mission_config=mis_cfg,
    vehicle_config=veh_cfg,
    environment_config=env_cfg,
    wrapper_params=wrp_prms
  )

  return sim_cfg

def getMaxParallel(user_requested_cores: int | None = None) -> int:
  """Return the safe number of parallel workers based on CPU availability."""
  available_cores = os.cpu_count() or 1 # Fallback to 1 if detection fails
  return min(user_requested_cores, available_cores) if user_requested_cores else available_cores

def generateWrapperBatchDir() -> str:
  """Generate a unique wrapper batch folder based on timestamp."""
  batch_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  year = datetime.now().strftime("%Y")
  month = datetime.now().strftime("%m")
  return os.path.join("logs", "wrapper", year, month, f"ws_{batch_timestamp}")

def runParallelBatch(max_parallel: int | None = None):
  """Generate configs and run simulations in parallel."""
  max_parallel = getMaxParallel(max_parallel)
  wrapper_batch_dir = generateWrapperBatchDir()
  print(f"Running simulations with up to {max_parallel} parallel workers.")
  print(f"Running batch in folder: {wrapper_batch_dir}")

  git_info = Logging.getGitRepoInfo()

  # Define parameter sweep
  densities = range(1000, 10001, 1000)
  sim_configs = [generateConfigForDensity(d, wrapper_batch_dir) for d in densities]

  args = [(cfg, git_info) for cfg in sim_configs]

  with ProcessPoolExecutor(max_workers=max_parallel) as executor:
    executor.map(runWrapperSimulationWithGitInfo, args)

def runWrapperSimulationWithGitInfo(args: tuple[Cfg.SimulationConfig, dict]):
  sim_cfg, git_info = args
  runWrapperSimulation(sim_cfg, git_info)