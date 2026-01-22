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
EVALUATOR_TYPE: Literal["inner", "outer", "combined"] = "combined"

# Directory where simulation logs will be saved
LOG_DIRECTORY: str = "simulation_logs/combined_diag_nsga3"

# Parallel evaluation configuration
PARALLEL_CONFIG: Dict[str, Any] = {
    "enabled": False,
    "n_workers": None,  # None = auto-detect CPU cores
    "use_processes": False,  # True = processes for true parallelism on Linux
}

# Whether to clean up eval_* folders after saving Pareto solution logs
CLEAN_EVAL_FOLDERS: bool = True
