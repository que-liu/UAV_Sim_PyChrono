"""Parameter selection utilities for GA tuning.

This module contains the implementation details for expanding parameter selections.
Please edit param_config.py for configuration instead of this file.
"""

from typing import Any, Dict, List, Tuple, Union


# =============================================================================
# PID Parameter Definitions
# =============================================================================

PID_PARAMETER_GROUPS = {
    "translational": [
        "KP_tran_x", "KP_tran_y", "KP_tran_z",
        "KI_tran_x", "KI_tran_y", "KI_tran_z",
        "KD_tran_x", "KD_tran_y", "KD_tran_z",
    ],
    "rotational": [
        "KP_rot_roll", "KP_rot_pitch", "KP_rot_yaw",
        "KI_rot_roll", "KI_rot_pitch", "KI_rot_yaw",
        "KD_rot_roll", "KD_rot_pitch", "KD_rot_yaw",
    ],
    "proportional": [
        "KP_tran_x", "KP_tran_y", "KP_tran_z",
        "KP_rot_roll", "KP_rot_pitch", "KP_rot_yaw",
    ],
    "integral": [
        "KI_tran_x", "KI_tran_y", "KI_tran_z",
        "KI_rot_roll", "KI_rot_pitch", "KI_rot_yaw",
    ],
    "derivative": [
        "KD_tran_x", "KD_tran_y", "KD_tran_z",
        "KD_rot_roll", "KD_rot_pitch", "KD_rot_yaw",
    ],
}

ALL_PID_PARAMETERS = PID_PARAMETER_GROUPS["translational"] + PID_PARAMETER_GROUPS["rotational"]

# MRAC matrix sizes (from encoding/config.py)
MRAC_MATRIX_SIZES = {
    "gamma_x_tran": 6,
    "gamma_r_tran": 3,
    "gamma_theta_tran": 6,
    "gamma_x_rot": 3,
    "gamma_r_rot": 3,
    "gamma_theta_rot": 6,
    "q_tran": 6,  # Q matrix for Lyapunov equation (affects P_tran)
}


# =============================================================================
# PID Selection Expansion
# =============================================================================

def expand_pid_tuning_selection(
    selection: Union[str, List[str], Dict[str, Any]]
) -> Tuple[str, ...]:
    """
    Convert PID tuning selection to a tuple of parameter names.
    
    Args:
        selection: One of:
            - "all": All 18 PID parameters
            - "translational": Tune only translational gains (9 params)
            - "rotational": Tune only rotational gains (9 params)
            - "proportional": Tune only KP gains (6 params)
            - "integral": Tune only KI gains (6 params)
            - "derivative": Tune only KD gains (6 params)
            - List of parameter names: Custom selection
            - Dict with "groups": List of group names to combine
    
    Returns:
        Tuple of parameter names to tune
    """
    if isinstance(selection, str):
        if selection == "all":
            return tuple(ALL_PID_PARAMETERS)
        elif selection in PID_PARAMETER_GROUPS:
            return tuple(PID_PARAMETER_GROUPS[selection])
        else:
            raise ValueError(
                f"Unknown PID selection: '{selection}'. "
                f"Valid options: 'all', {list(PID_PARAMETER_GROUPS.keys())}"
            )
    
    elif isinstance(selection, list):
        unknown = set(selection) - set(ALL_PID_PARAMETERS)
        if unknown:
            raise ValueError(f"Unknown PID parameters: {sorted(unknown)}")
        return tuple(selection)
    
    elif isinstance(selection, dict):
        if "groups" in selection:
            params = []
            for group_name in selection["groups"]:
                if group_name not in PID_PARAMETER_GROUPS:
                    raise ValueError(f"Unknown PID group: '{group_name}'")
                params.extend(PID_PARAMETER_GROUPS[group_name])
            seen = set()
            unique_params = [p for p in params if not (p in seen or seen.add(p))]
            return tuple(unique_params)
        elif "params" in selection:
            return expand_pid_tuning_selection(selection["params"])
        else:
            raise ValueError("Invalid PID selection dict. Use 'groups' or 'params' key.")
    
    raise ValueError(f"Invalid PID selection type: {type(selection)}")


# =============================================================================
# MRAC Selection Expansion
# =============================================================================

def _get_matrix_parameter_names(
    prefix: str,
    size: int,
    selection: Union[str, Dict[str, Any]]
) -> List[str]:
    """Generate parameter names for a matrix based on selection method."""
    all_params = [f"{prefix}_L{i+1}{j+1}" for i in range(size) for j in range(i+1)]
    
    if selection == "full":
        return all_params
    
    elif selection == "diagonal":
        return [f"{prefix}_L{i+1}{i+1}" for i in range(size)]
    
    elif isinstance(selection, dict):
        if "diagonal" in selection:
            mask = selection["diagonal"]
            if len(mask) != size:
                raise ValueError(f"Diagonal mask for {prefix} must have length {size}, got {len(mask)}")
            return [f"{prefix}_L{i+1}{i+1}" for i, m in enumerate(mask) if m]
        
        elif "selection_matrix" in selection:
            sel_matrix = selection["selection_matrix"]
            if len(sel_matrix) != size:
                raise ValueError(f"Selection matrix for {prefix} must have {size} rows")
            selected = []
            idx = 0
            for i, row in enumerate(sel_matrix):
                if len(row) != i + 1:
                    raise ValueError(f"Selection matrix row {i} must have {i+1} elements")
                for j, val in enumerate(row):
                    if val:
                        selected.append(all_params[idx])
                    idx += 1
            return selected
    
    raise ValueError(f"Invalid selection for {prefix}: {selection}")


def expand_mrac_tuning_selection(
    matrix_selection: Dict[str, Union[str, Dict[str, Any]]]
) -> Tuple[str, ...]:
    """
    Convert MRAC matrix selection to a tuple of parameter names.
    
    Args:
        matrix_selection: Dict mapping matrix names to selection specs:
            - "full": All lower-triangular Cholesky elements
            - "diagonal": Only diagonal elements
            - {"diagonal": [1, 0, 1, ...]}: Custom diagonal mask
            - {"selection_matrix": [[1], [0, 1], ...]}: Custom selection
    
    Returns:
        Tuple of parameter names to tune
    """
    all_params: List[str] = []
    for matrix_name, selection_spec in matrix_selection.items():
        matrix_name_lower = matrix_name.lower()
        if matrix_name_lower not in MRAC_MATRIX_SIZES:
            raise ValueError(f"Unknown matrix: {matrix_name}. Valid: {list(MRAC_MATRIX_SIZES.keys())}")
        
        size = MRAC_MATRIX_SIZES[matrix_name_lower]
        params = _get_matrix_parameter_names(matrix_name_lower, size, selection_spec)
        all_params.extend(params)
    
    return tuple(all_params)


# Backward compatibility alias
expand_matrix_tuning_selection = expand_mrac_tuning_selection
