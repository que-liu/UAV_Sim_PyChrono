"""Mission configuration for GA tuning simulations.

Configure simulation settings like trajectory type, duration, and visualization here.
"""

from typing import Any, Dict

# Mission configuration: configure simulation settings
MISSION_CONFIG: Dict[str, Any] = {
    "trajectory_type": "piecewise_polynomial_trajectory",
    # If the trajectory_type is "piecewise_polynomial_trajectory", then choose the trajectory file to run
    # Path relative to 'current_working_directory/params/user_defined_trajectory'
    "trajectory_data_path": "bean_trajectory0p2.json",
    "duration": 30.0,
    "visualization": False,
    "wrapper_flag": True,
}
