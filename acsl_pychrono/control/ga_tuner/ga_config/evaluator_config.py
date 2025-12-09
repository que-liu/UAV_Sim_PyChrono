"""Evaluator configuration for GA tuning - configure evaluation settings.

Users configure:
- Evaluator type (inner loop or outer loop)
- Log directory for simulation results
- Parallel evaluation settings
"""

from typing import Dict, Any

# Evaluator type: "inner" for attitude/rotational control, "outer" for position/translational control
EVALUATOR_TYPE: str = "outer"

# Directory where simulation logs will be saved
LOG_DIRECTORY: str = "simulation_logs/ga_mrac_nsga2"

# Parallel evaluation configuration
PARALLEL_CONFIG: Dict[str, Any] = {
    "enabled": True,
    "n_workers": 2,
    "use_processes": False,
}
