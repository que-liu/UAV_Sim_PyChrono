"""Evaluator configuration for GA tuning - configure evaluation settings.

Users configure:
- Evaluator type (inner loop, outer loop, or grouped)
- Log directory for simulation results
- Parallel evaluation settings
"""

from typing import Dict, Any, Literal

# Evaluator type determines which metrics are optimized:
#   "inner"  : Attitude/rotational control (3 objectives: attitude, angular_velocity, rotational_effort)
#   "outer"  : Position/translational control (3 objectives: position, velocity, translational_effort)  
#   "grouped": Combined inner+outer (3 composite objectives: translational_error, rotational_error, control_effort)
EVALUATOR_TYPE: Literal["inner", "outer", "grouped"] = "grouped"

# Directory where simulation logs will be saved
LOG_DIRECTORY: str = "simulation_logs/Inner_and_Outer_Loop_Test"

# Parallel evaluation configuration
PARALLEL_CONFIG: Dict[str, Any] = {
    "enabled": False,
    "n_workers": 2,
    "use_processes": False,
}

# Whether to clean up eval_* folders after saving Pareto solution logs
CLEAN_EVAL_FOLDERS: bool = True
