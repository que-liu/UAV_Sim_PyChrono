"""Evaluator configuration for GA tuning - configure evaluation settings.

Users configure:
- Evaluator type (inner loop, outer loop, or combined)
- Log directory for simulation results
- Parallel evaluation settings
"""

from typing import Dict, Any, Literal

# Evaluator type determines which metrics are optimized:
#   "inner"    : Attitude/rotational control (3 objectives: attitude, angular_velocity, rotational_effort)
#   "outer"    : Position/translational control (3 objectives: position, velocity, translational_effort)  
#   "combined" : Both inner+outer (6 objectives: all inner + all outer metrics)
EVALUATOR_TYPE: Literal["inner", "outer", "combined"] = "inner"

# Directory where simulation logs will be saved
LOG_DIRECTORY: str = "simulation_logs/inner_full_nsga2"

# Parallel evaluation configuration
PARALLEL_CONFIG: Dict[str, Any] = {
    "enabled": True,
    "n_workers": None,
    "use_processes": True,
}

# Whether to clean up eval_* folders after saving Pareto solution logs
CLEAN_EVAL_FOLDERS: bool = True
