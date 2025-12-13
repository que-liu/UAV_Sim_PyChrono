"""
Controller factory with GA tuner integration.

This module provides a thin wrapper around the standard controller factory,
adding support for injecting GA-optimised parameters into the existing PID
and MRAC controllers.
"""

from functools import lru_cache
from typing import Mapping, Sequence
import numpy as np
from scipy import linalg

from ..controllers.mrac_tuning import MRACTuning
from ..controllers.pid_tuning import PIDTuning
from acsl_pychrono.control import controller_classes as CONTROLLER_CLASSES


def instantiateControllerWithGA(controller_type: str, ode_input, flight_params, timestep, wrapper_params=None):
    """
    Factory function to instantiate controllers with optional GA tuner integration.

    Args:
        controller_type: Type of controller ('PID', 'MRAC', or 'TwoLayerMRAC')
        ode_input: ODE input object
        flight_params: Flight parameters object
        timestep: Simulation timestep
        wrapper_params: Optional wrapper parameters (may contain GA tuner params)

    Returns:
        Tuple of (gains, controller, logger)
    """
    if controller_type not in CONTROLLER_CLASSES:
        raise ValueError(f"Unknown controller type: {controller_type}")

    GainsClass, ControllerClass, LoggerClass = CONTROLLER_CLASSES[controller_type]

    # Instantiate default gains
    gains = GainsClass(flight_params)

    # Apply GA parameters if provided
    external_params = getattr(wrapper_params, "external_controller_params", None) if wrapper_params else None
    if external_params:
            try:
                if controller_type == "PID":
                    _apply_pid_ga_parameters(gains, external_params)
                elif controller_type =="MRAC":
                    _apply_mrac_ga_parameters(gains, external_params)
            except Exception as exc:
                print(f"Warning: GA tuner parameter mapping failed: {exc}")

    controller = ControllerClass(gains, ode_input, flight_params, timestep)
    logger = LoggerClass(gains)

    return gains, controller, logger


def _apply_pid_ga_parameters(gains, external_params):
    """Apply GA-optimised PID parameters to the gains object."""
    pid_values = external_params.get("pid_params")
    if pid_values is None:
        print("Warning: GA tuner enabled but missing PID parameters in external_controller_params")
        return

    pid_sequence = list(pid_values)

    tuning = _get_pid_tuning()
    try:
        gain_dict = tuning.vector_to_gains(pid_sequence)
    except ValueError as exc:
        print(f"Warning: Invalid PID parameter vector received from GA tuner: {exc}")
        return

    for attr in ['KP_tran', 'KI_tran', 'KD_tran', 'KP_rot', 'KI_rot', 'KD_rot']:
        if attr in gain_dict:
            gains_attr = np.matrix(gain_dict[attr])
            setattr(gains, attr, gains_attr)

    diag_keys = ['KP_tran', 'KI_tran', 'KD_tran', 'KP_rot', 'KI_rot', 'KD_rot']
    diag_gains = {key: _diagonal_values(gain_dict[key]) for key in diag_keys}

    print(
        "[GA TUNER] Applied external PID gains "
        f"(KP_tran={diag_gains['KP_tran']}, KI_tran={diag_gains['KI_tran']}, "
        f"KD_tran={diag_gains['KD_tran']}, KP_rot={diag_gains['KP_rot']}, "
        f"KI_rot={diag_gains['KI_rot']}, KD_rot={diag_gains['KD_rot']})"
    )


def _apply_mrac_ga_parameters(gains, external_params):
    """
    Apply GA-optimised MRAC parameters to the gains object.

    Accepts either:
      * A full parameter vector matching MRACTuning.get_parameter_names(), or
      * A mapping of parameter names to values (optionally with a 'names'/'values').
    """
    params = external_params.get("mrac_params")
    if params is None:
        print("Warning: GA tuner enabled but no mrac_params in external_controller_params")
        return

    tuning = _get_mrac_tuning()
    parameter_names = tuning.get_parameter_names()
    expected_length = len(parameter_names)

    # Build a full parameter vector
    vector = tuning.gains_to_vector(tuning.get_default_gains())

    if isinstance(params, Mapping):
        names, values = _extract_named_params_from_mapping(params)
        if names is None:
            return
        for name, value in zip(names, values):
            try:
                idx = parameter_names.index(name)
            except ValueError:
                print(f"Warning: Unknown MRAC parameter '{name}' received from GA tuner")
                return
            vector[idx] = float(value)
    else:
        if not isinstance(params, (Sequence, np.ndarray)) or isinstance(params, (str, bytes)):
            print("Warning: mrac_params must be a full vector or a mapping of parameter names")
            return
        if len(params) != expected_length:
            print(f"Warning: Expected {expected_length} MRAC parameters, received {len(params)}")
            return
        vector = list(params)

    # Convert to gain structures and apply to the MRAC gains object
    gain_dict = tuning.vector_to_gains(vector)
    for key, value in gain_dict.items():
        if hasattr(gains, key):
            if isinstance(value, np.ndarray):
                setattr(gains, key, np.matrix(value))
            else:
                setattr(gains, key, value)
    
    # Recompute P matrices via Lyapunov equation if Q matrices were changed
    if 'Q_tran' in gain_dict and hasattr(gains, 'A_ref_tran'):
        gains.P_tran = np.matrix(linalg.solve_continuous_lyapunov(
            gains.A_ref_tran.T, -gains.Q_tran
        ))
    
    if 'Q_rot' in gain_dict and hasattr(gains, 'A_ref_rot'):
        gains.P_rot = np.matrix(linalg.solve_continuous_lyapunov(
            gains.A_ref_rot.T, -gains.Q_rot
        ))


def _extract_named_params_from_mapping(data: Mapping):
    """Parse named MRAC parameters from mapping."""
    if "names" in data and "values" in data:
        names = list(data["names"])
        values = list(data["values"])
        if len(names) != len(values):
            print("Warning: mrac_params names/values length mismatch")
            return None, None
        return names, values

    names = list(data.keys())
    values = [data[name] for name in names]
    return names, values

def _diagonal_values(matrix_like):
    """Return the diagonal of a gain matrix as a flat numpy array for logging."""
    return np.diag(np.asarray(matrix_like, dtype=float))

@lru_cache(maxsize=1)
def _get_mrac_tuning() -> MRACTuning:
    """Initialisation of the shared MRAC tuning helper."""
    return MRACTuning()


@lru_cache(maxsize=1)
def _get_pid_tuning() -> PIDTuning:
    """Initialisation of the shared PID tuning helper."""
    return PIDTuning()
