"""Parameter configuration for GA tuning.

Edit the settings below to define which parameters to tune.

PID Options:
    - "all": All 18 parameters
    - "translational": KP/KI/KD for x, y, z (9 params)
    - "rotational": KP/KI/KD for roll, pitch, yaw (9 params)  
    - "proportional", "integral", "derivative": By gain type (6 params each)
    - ["KP_tran_x", "KI_tran_y", ...]: Custom list

MRAC Options (per matrix):
    - "full": All Cholesky elements
    - "diagonal": Only diagonal elements
    - {"diagonal": [1, 0, 1, ...]}: Custom diagonal mask (1=tune, 0=skip)
    - {"selection_matrix": [[1], [0, 1], ...]}: Custom lower-triangular selection

MRAC Available Matrices (with sizes):
    - gamma_x_tran (6x6), gamma_r_tran (3x3), gamma_theta_tran (6x6)
    - gamma_x_rot (3x3), gamma_r_rot (3x3), gamma_theta_rot (6x6)
    - q_tran (6x6): Q matrix for Lyapunov equation (affects P_tran computation)
"""

from typing import Literal

from .param_utils import (
    expand_pid_tuning_selection,
    expand_mrac_tuning_selection,
    expand_matrix_tuning_selection,  # backward compatibility
    PID_PARAMETER_GROUPS,
    ALL_PID_PARAMETERS,
    MRAC_MATRIX_SIZES,
)

# =============================================================================
# CONFIGURATION - Edit these values
# =============================================================================

# Controller type: must match ga_config.py
CONTROLLER_TYPE: Literal["PID", "MRAC"] = "MRAC"

# PID: Which parameters to tune
PID_TUNING_SELECTION = "translational"

# MRAC: Which matrix elements to tune
# Examples of different selection methods:
MRAC_TUNING_SELECTION = {
    # Simple: tune only diagonal elements (3 params)
    #"gamma_x_rot": "diagonal",
    
    # Simple: tune all lower-triangular Cholesky elements (6 params for 3x3)
    # "gamma_r_rot": "full",
    
    # Custom diagonal mask: [1,1,0,1,1,0] means tune L11,L22,L44,L55 but skip L33,L66
    # "gamma_theta_tran": {
    #     "diagonal": [1, 1, 0, 1, 1, 0],
    # },
    
    # Custom selection matrix (lower-triangular, row by row):
    # Row 0: [1]           -> tune L11
    # Row 1: [0, 1]        -> skip L21, tune L22
    # Row 2: [0, 0, 0]     -> skip all
    # Row 3: [0, 1, 0, 0]  -> tune L42 only
    # Row 4: [0, 0, 0, 0, 0]
    # Row 5: [0, 0, 0, 0, 0, 0]
    # "gamma_x_tran": {
    #     "selection_matrix": [
    #         [1],
    #         [0, 1],
    #         [0, 0, 0],
    #         [0, 1, 0, 0],
    #         [0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 0, 0],
    #     ]
    # },
    
    # Q_tran: Q matrix for Lyapunov equation (P_tran = solve_lyapunov(A_ref_tran.T, -Q_tran))
    "q_tran": "diagonal",  # tune all 6 diagonal elements
    "gamma_x_tran": "diagonal",
    "gamma_r_tran": "diagonal",
    "gamma_theta_tran": "diagonal",
}

# =============================================================================
# Computed values (do not edit below)
# =============================================================================

def get_tuned_parameters(controller_type: str = None):
    """Get tuned parameters for the specified controller type."""
    controller = (controller_type or CONTROLLER_TYPE).upper()
    if controller == "PID":
        return expand_pid_tuning_selection(PID_TUNING_SELECTION)
    elif controller == "MRAC":
        return expand_mrac_tuning_selection(MRAC_TUNING_SELECTION)
    else:
        raise ValueError(f"Unknown controller type: {controller}")

TUNED_PARAMETERS = get_tuned_parameters(CONTROLLER_TYPE)

# Backward compatibility
MATRIX_TUNING_SELECTION = MRAC_TUNING_SELECTION
