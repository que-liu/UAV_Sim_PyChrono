# acsl_pychrono/user_defined_trajectory/__init__.py

from acsl_pychrono.config.config import MissionConfig

from .base_user_defined_trajectory import BaseUserDefinedTrajectory
from .circular_trajectory import CircularTrajectory
from .hover_trajectory import HoverTrajectory
from .square_trajectory import SquareTrajectory
from .rounded_rectangle_trajectory import RoundedRectangleTrajectory
from .piecewise_polynomial_trajectory import PiecewisePolynomialTrajectory


trajectory_classes = {
  'circular_trajectory': CircularTrajectory,
  'hover_trajectory': HoverTrajectory,
  'square_trajectory': SquareTrajectory,
  'rounded_rectangle_trajectory': RoundedRectangleTrajectory,
  'piecewise_polynomial_trajectory': PiecewisePolynomialTrajectory,
}

def instantiateTrajectory(
    mission_config: MissionConfig,
    flight_params,
    mfloor,
    mfloor_Yposition
    ) -> BaseUserDefinedTrajectory:
  trajectory_type = mission_config.trajectory_type
  TrajectoryClass = trajectory_classes.get(trajectory_type)
  if TrajectoryClass is None:
    raise ValueError(f"Unknown trajectory type: {trajectory_type}")
  if trajectory_type == 'piecewise_polynomial_trajectory':
    return TrajectoryClass(flight_params, mfloor, mfloor_Yposition, mission_config)
  else:
    return TrajectoryClass(flight_params, mfloor, mfloor_Yposition)