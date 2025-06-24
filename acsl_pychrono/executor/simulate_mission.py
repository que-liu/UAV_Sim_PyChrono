import traceback
from acsl_pychrono.simulation.simulation import Simulation
import acsl_pychrono.user_defined_trajectory as Traj
import acsl_pychrono.control as Ctrl
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.simulation.ode_input import OdeInput
from acsl_pychrono.simulation.flight_params import FlightParams

import pickle
import os

def simulateMission(sim: Simulation, git_info: dict | None = None):
  # Instantiation of classes
  flight_params = FlightParams()
  ode_input = OdeInput()
  sim.setGravitationalAcceleration(flight_params)

  # Instantiate user-defined trajectory
  user_defined_trajectory: Traj.BaseUserDefinedTrajectory = Traj.instantiateTrajectory(
    sim.mission_config,
    flight_params,
    sim.mfloor,
    sim.mfloor_Yposition
  )

  # Instantiation of controller, gains, and logger
  (gains, pid_controller, logger) = Ctrl.instantiateController(
    sim.mission_config.controller_type,
    ode_input,
    flight_params,
    sim.mission_config.timestep
  )

  sim.assignInstances(
    flight_params,
    ode_input,
    user_defined_trajectory,
    gains,
    pid_controller,
    logger
  )

  try:
    sim.runSimulationLoop()
  except Exception as e:
    print(f"\n[ERROR] Simulation crashed: {e}")
    traceback.print_exc()
  finally:
    print("[INFO] Saving logs before exit...")
    log_dict = logger.toDictionary()
    
    # Define and create log folder
    logs_folder = os.path.join(os.getcwd(), "simulation_logs")
    os.makedirs(logs_folder, exist_ok=True)

    # Define file path
    log_file = os.path.join(logs_folder, "log_dict.pkl")

    # Save log_dict
    with open(log_file, "wb") as f:
      pickle.dump(log_dict, f)

    print(f"[INFO] Log saved to {log_file}")



    # Logging.saveMatlabWorkspaceLog(
    #   log_dict,
    #   gains,
    #   sim.simulation_config,
    #   git_info
    # )