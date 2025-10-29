"""
Controller factory with GA tuner integration.

This module provides a thin wrapper around the standard controller factory,
adding support for injecting GA-optimised parameters into the existing PID
and MRAC controllers.
"""

from typing import Mapping, Sequence

import numpy as np

from ..PID import PID, PIDGains, PIDLogger
from ..MRAC import MRAC, MRACGains, MRACLogger
from ..TwoLayerMRAC import TwoLayerMRAC, TwoLayerMRACGains, TwoLayerMRACLogger
from .controllers.mrac_tuning import MRACTuning
from .controllers.pid_tuning import PIDTuning

# Controller class mappings
CONTROLLER_CLASSES = {
    'PID': (PIDGains, PID, PIDLogger),
    'MRAC': (MRACGains, MRAC, MRACLogger),
    'TwoLayerMRAC': (TwoLayerMRACGains, TwoLayerMRAC, TwoLayerMRACLogger),
}

# Lazily created tuning helpers reused for parameter reconstruction
_MRAC_TUNING = None
_PID_TUNING = None


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
    if wrapper_params is not None and getattr(wrapper_params, "use_ga_tuner", False):
        external_params = getattr(wrapper_params, "external_controller_params", None)
        if external_params:
            try:
                if controller_type == "PID":
                    _apply_pid_ga_parameters(gains, external_params)
                elif controller_type in {"MRAC", "TwoLayerMRAC"}:
                    _apply_mrac_ga_parameters(gains, external_params)
            except Exception as exc:
                print(f"Warning: GA tuner parameter mapping failed: {exc}")

    controller = ControllerClass(gains, ode_input, flight_params, timestep)
    logger = LoggerClass(gains)

    return gains, controller, logger


def _apply_pid_ga_parameters(gains, external_params):
    """Apply GA-optimised PID parameters to the gains object."""
    pid_values = external_params.get("pid_gains")
    if pid_values is None:
        print("Warning: GA tuner enabled but missing PID gains in external_controller_params")
        return

    pid_sequence = list(pid_values)

    # Backward compatibility: accept legacy 9-parameter vectors (translational only)
    if len(pid_sequence) == 9:
        kp_values = np.array([pid_sequence[0], pid_sequence[3], pid_sequence[6]], dtype=float)
        ki_values = np.array([pid_sequence[1], pid_sequence[4], pid_sequence[7]], dtype=float)
        kd_values = np.array([pid_sequence[2], pid_sequence[5], pid_sequence[8]], dtype=float)

        gains.KP_tran = np.matrix(np.diag(kp_values))
        gains.KI_tran = np.matrix(np.diag(ki_values))
        gains.KD_tran = np.matrix(np.diag(kd_values))

        print("[GA TUNER] Applied external PID gains (translational only)")
        print(f"           KP_tran={kp_values}, KI_tran={ki_values}, KD_tran={kd_values}")
        return

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

    kp_tran = np.diag(np.asarray(gain_dict['KP_tran'], dtype=float))
    ki_tran = np.diag(np.asarray(gain_dict['KI_tran'], dtype=float))
    kd_tran = np.diag(np.asarray(gain_dict['KD_tran'], dtype=float))
    kp_rot = np.diag(np.asarray(gain_dict['KP_rot'], dtype=float))
    ki_rot = np.diag(np.asarray(gain_dict['KI_rot'], dtype=float))
    kd_rot = np.diag(np.asarray(gain_dict['KD_rot'], dtype=float))

    print("[GA TUNER] Applied external PID gains (translational & rotational)")
    print(f"           KP_tran={kp_tran}, KI_tran={ki_tran}, KD_tran={kd_tran}")
    print(f"           KP_rot={kp_rot}, KI_rot={ki_rot}, KD_rot={kd_rot}")


def _apply_mrac_ga_parameters(gains, external_params):
    """
    Apply GA-optimised MRAC parameters to the gains object.

    Accepts either:
      * A full parameter vector matching MRACTuning.get_parameter_names(), or
      * A mapping of parameter names to values (optionally with a 'names'/'values' payload).
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


def _extract_named_params_from_mapping(data: Mapping):
    """Parse named MRAC parameters from a mapping payload."""
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


def _get_mrac_tuning() -> MRACTuning:
    """Lazy initialisation of the shared MRAC tuning helper."""
    global _MRAC_TUNING
    if _MRAC_TUNING is None:
        _MRAC_TUNING = MRACTuning()
    return _MRAC_TUNING


def _get_pid_tuning() -> PIDTuning:
    """Lazy initialisation of the shared PID tuning helper."""
    global _PID_TUNING
    if _PID_TUNING is None:
        _PID_TUNING = PIDTuning()
    return _PID_TUNING
