# acsl_pychrono/control/__init__.py

from .PID import PID, PIDGains, PIDLogger
from .MRAC import MRAC, MRACGains, MRACLogger
from .TwoLayerMRAC import TwoLayerMRAC, TwoLayerMRACGains, TwoLayerMRACLogger

controller_classes = {
  'PID': (PIDGains, PID, PIDLogger),
  'MRAC': (MRACGains, MRAC, MRACLogger),
  'TwoLayerMRAC': (TwoLayerMRACGains, TwoLayerMRAC, TwoLayerMRACLogger),
}

def instantiateController(controller_type: str, ode_input, flight_params, timestep):
  if controller_type not in controller_classes:
    raise ValueError(f"Unknown controller type: {controller_type}")
  
  GainsClass, ControllerClass, LoggerClass = controller_classes[controller_type]
  gains = GainsClass(flight_params)
  controller = ControllerClass(gains, ode_input, flight_params, timestep)
  logger = LoggerClass(gains)

  return gains, controller, logger