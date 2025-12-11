"""
Parameter converter for comprehensive MRAC parameter handling.
Converts between parameter vectors and MRAC gain structures based on actual mrac_gains.py.
"""

import ast
import pathlib
import numpy as np
from typing import Dict, List, Any, Tuple

from .cholesky_utils import (
    flatten_cholesky,
    get_cholesky_parameter_names,
    reconstruct_from_cholesky,
)
from .config import (
    GAMMA_MATRIX_CONFIGS,
    _FALLBACK_MRAC_DEFAULTS,
    target_names,
    DIAGONAL_MATRIX_PARAMS,
    SCALAR_PARAMETER_GROUPS,
)


def _set_diagonal_matrices_from_params(params: Dict[str, Any], mrac_gains: Dict[str, Any]) -> None:
    """Populate diagonal matrix gains from scalar parameters."""
    for matrix_name, keys in DIAGONAL_MATRIX_PARAMS.items():
        if not keys or keys[0] not in params:
            continue
        values = [params[key] for key in keys]
        mrac_gains[matrix_name] = np.matrix(np.diag(values))


def _extract_diagonal_matrices_to_params(mrac_gains: Dict[str, Any], params: Dict[str, Any]) -> None:
    """Extract diagonal entries from MRAC gains back to scalar parameters."""
    for matrix_name, keys in DIAGONAL_MATRIX_PARAMS.items():
        if matrix_name not in mrac_gains:
            continue
        for idx, key in enumerate(keys):
            params[key] = mrac_gains[matrix_name][idx, idx]


def _copy_scalar_group_from_params(params: Dict[str, Any], mrac_gains: Dict[str, Any], keys: Tuple[str, ...]) -> None:
    if not keys or keys[0] not in params:
        return
    for key in keys:
        mrac_gains[key] = params[key]


def _extract_scalar_group_to_params(mrac_gains: Dict[str, Any], params: Dict[str, Any], keys: Tuple[str, ...]) -> None:
    if not keys or keys[0] not in mrac_gains:
        return
    for key in keys:
        params[key] = mrac_gains[key]

_MRAC_GAINS_PATH = pathlib.Path(__file__).resolve().parents[2] / "MRAC" / "mrac_gains.py"

def load_default_mrac_gains_from_source() -> Dict[str, Any]:
    """Parse mrac_gains.py to extract default gain values. """
    try:
        source = _MRAC_GAINS_PATH.read_text()
    except OSError:
        return {name: (_FALLBACK_MRAC_DEFAULTS[name].copy()
                      if isinstance(_FALLBACK_MRAC_DEFAULTS[name], np.ndarray) else _FALLBACK_MRAC_DEFAULTS[name])
                for name in target_names if name in _FALLBACK_MRAC_DEFAULTS}

    tree = ast.parse(source, filename=str(_MRAC_GAINS_PATH))
    extracted: Dict[str, Any] = {}

    class _GainsExtractor(ast.NodeVisitor):
        def visit_Assign(self, node):
            for target in node.targets:
                if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == 'self':
                    name = target.attr
                    if name in target_names and name not in extracted:
                        expr = ast.Expression(node.value)
                        code = compile(expr, str(_MRAC_GAINS_PATH), 'eval')
                        try:
                            extracted[name] = eval(code, {'np': np})
                        except Exception:
                            extracted[name] = None
            self.generic_visit(node)

    _GainsExtractor().visit(tree)

    if not all(name in extracted and extracted[name] is not None for name in target_names):
        return {name: (_FALLBACK_MRAC_DEFAULTS[name].copy()
                      if isinstance(_FALLBACK_MRAC_DEFAULTS[name], np.ndarray) else _FALLBACK_MRAC_DEFAULTS[name])
                for name in target_names if name in _FALLBACK_MRAC_DEFAULTS}

    normalized = {}
    for name, value in extracted.items():
        if isinstance(value, np.matrix):
            normalized[name] = np.asarray(value)
        elif isinstance(value, np.ndarray):
            normalized[name] = value.copy()
        else:
            normalized[name] = float(value)

    return normalized

class MRACParameterConverter:
    """
    Converts between parameter vectors and MRAC gain structures.
    Handles the full MRAC parameter set based on actual mrac_gains.py implementation.
    """
    
    def __init__(self, parameter_bounds):
        self.bounds = parameter_bounds
        self.parameter_names = parameter_bounds.parameter_names
        get_groups = getattr(parameter_bounds, "get_parameter_groups", None)
        self.parameter_groups = get_groups() if callable(get_groups) else {}
    
    def vector_to_mrac_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Convert parameter vector to MRAC gain structure matching mrac_gains.py.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Dictionary containing MRAC gain structure
        """
        if len(parameter_vector) != len(self.parameter_names):
            raise ValueError(f"Parameter vector length {len(parameter_vector)} != expected {len(self.parameter_names)}")
        
        # Create parameter dictionary
        params = dict(zip(self.parameter_names, parameter_vector))
        
        # Build MRAC gain structure based on actual mrac_gains.py
        mrac_gains = {}

        # Adaptive learning rate matrices (Gamma) represented via Cholesky factors
        for matrix_name, config in GAMMA_MATRIX_CONFIGS.items():
            prefix = config['prefix']
            size = config['size']
            entry_keys = get_cholesky_parameter_names(prefix, size)

            if not all(key in params for key in entry_keys):
                continue

            cholesky_entries = [float(params[key]) for key in entry_keys]
            gamma_matrix = reconstruct_from_cholesky(
                cholesky_entries,
                size,
                log_diagonals=True,
            )
            mrac_gains[matrix_name] = np.matrix(gamma_matrix)
        
        _set_diagonal_matrices_from_params(params, mrac_gains)
        for _, group_keys in SCALAR_PARAMETER_GROUPS:
            _copy_scalar_group_from_params(params, mrac_gains, group_keys)
        
        return mrac_gains
    
    def mrac_gains_to_vector(self, mrac_gains: Dict[str, Any]) -> List[float]:
        """
        Convert MRAC gain structure to parameter vector.
        
        Args:
            mrac_gains: Dictionary containing MRAC gain structure
            
        Returns:
            List of parameter values
        """
        params = {}
        
        # Extract adaptive learning rates via Cholesky factorization
        for matrix_name, config in GAMMA_MATRIX_CONFIGS.items():
            if matrix_name not in mrac_gains:
                continue

            matrix = np.array(mrac_gains[matrix_name], dtype=float)
            size = config['size']
            prefix = config['prefix']

            if matrix.shape != (size, size):
                raise ValueError(f"{matrix_name} must be a {size}x{size} matrix, got {matrix.shape}")

            entries = flatten_cholesky(matrix, log_diagonals=True)
            entry_keys = get_cholesky_parameter_names(prefix, size)
            for key, value in zip(entry_keys, entries):
                params[key] = value
        
        _extract_diagonal_matrices_to_params(mrac_gains, params)
        for _, group_keys in SCALAR_PARAMETER_GROUPS:
            _extract_scalar_group_to_params(mrac_gains, params, group_keys)
        
        # Convert to vector in correct order
        return [params[name] for name in self.parameter_names]
    
    def get_baseline_gains(self) -> Dict[str, Any]:
        """
        Get baseline MRAC gains based on actual mrac_gains.py default values.

        Returns:
            Dictionary containing baseline MRAC gain structure
        """
        baseline_gains = load_default_mrac_gains_from_source()
        result = {}
        for key, value in baseline_gains.items():
            if isinstance(value, np.ndarray):
                result[key] = value.copy()
            else:
                result[key] = float(value)
        return result
    
    def get_parameter_summary(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Get a summary of parameter values organized by groups.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Dictionary with parameter summaries by group
        """
        params = dict(zip(self.parameter_names, parameter_vector))
        
        summary = {}
        
        # Adaptive learning rates
        adaptive_summary = {}
        for matrix_name, config in GAMMA_MATRIX_CONFIGS.items():
            prefix = config['prefix']
            size = config['size']
            entry_keys = get_cholesky_parameter_names(prefix, size)
            if not all(key in params for key in entry_keys):
                continue

            cholesky_entries = [params[key] for key in entry_keys]
            gamma_matrix = reconstruct_from_cholesky(
                cholesky_entries,
                size,
                log_diagonals=True,
            )
            adaptive_summary[matrix_name] = {
                'cholesky_entries': cholesky_entries,
                'gamma_matrix': gamma_matrix.tolist()
            }

        if adaptive_summary:
            summary['adaptive_learning_rates'] = adaptive_summary
        
        # Reference model gains (use DIAGONAL_MATRIX_PARAMS to avoid repeating keys)
        ref_p_keys = DIAGONAL_MATRIX_PARAMS.get('K_P_omega_ref')
        ref_i_keys = DIAGONAL_MATRIX_PARAMS.get('K_I_omega_ref')
        if ref_p_keys and ref_p_keys[0] in params and ref_i_keys and ref_i_keys[0] in params:
            summary['reference_model'] = {
                'K_P_omega_ref': [params[k] for k in ref_p_keys],
                'K_I_omega_ref': [params[k] for k in ref_i_keys],
            }

        # Baseline PID gains (collect available groups from DIAGONAL_MATRIX_PARAMS)
        pid_groups = ['KP_tran', 'KD_tran', 'KI_tran', 'KP_rot']
        baseline = {}
        for g in pid_groups:
            keys = DIAGONAL_MATRIX_PARAMS.get(g)
            if keys and keys[0] in params:
                baseline[g] = [params[k] for k in keys]
        if baseline:
            summary['baseline_pid'] = baseline

        # Modification parameters (use SCALAR_PARAMETER_GROUPS)
        sigma_keys = None
        dead_zone_keys = None
        for grp_name, keys in SCALAR_PARAMETER_GROUPS:
            if grp_name == 'sigma_params':
                sigma_keys = keys
            elif grp_name == 'dead_zone_params':
                dead_zone_keys = keys

        if sigma_keys and sigma_keys[0] in params:
            mod_params = {'sigma': [params[k] for k in sigma_keys]}
            if dead_zone_keys and dead_zone_keys[0] in params:
                mod_params['dead_zone'] = [params[k] for k in dead_zone_keys]
            summary['modification_parameters'] = mod_params
        
        return summary
