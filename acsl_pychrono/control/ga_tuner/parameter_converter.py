"""
Parameter converter for comprehensive MRAC parameter handling.
Converts between parameter vectors and MRAC gain structures based on actual mrac_gains.py.
"""

import ast
import pathlib
import numpy as np
from typing import Dict, List, Any, Tuple

# Configuration for Gamma matrices, this can be extended
GAMMA_MATRIX_CONFIGS = {
    'Gamma_x_tran': {'prefix': 'gamma_x_tran', 'size': 6},
    'Gamma_r_tran': {'prefix': 'gamma_r_tran', 'size': 3},
    'Gamma_Theta_tran': {'prefix': 'gamma_Theta_tran', 'size': 6},
    'Gamma_x_rot': {'prefix': 'gamma_x_rot', 'size': 3},
    'Gamma_r_rot': {'prefix': 'gamma_r_rot', 'size': 3},
    'Gamma_Theta_rot': {'prefix': 'gamma_Theta_rot', 'size': 6},

}

_MRAC_GAINS_PATH = pathlib.Path(__file__).resolve().parents[1] / "MRAC" / "mrac_gains.py"

_FALLBACK_MRAC_DEFAULTS = {
    'Gamma_x_tran': np.diag([1e1, 1e1, 1e2, 1e1, 1e1, 1e2]),
    'Gamma_r_tran': np.diag([3e-2, 3e-2, 12e-2]),
    'Gamma_Theta_tran': np.diag([1e1, 1e1, 2e1, 1e1, 1e1, 2e1]),
    'Gamma_x_rot': np.diag([1e4, 1e4, 1e4]),
    'Gamma_r_rot': np.diag([5e0, 5e0, 5e0]),
    'Gamma_Theta_rot': np.diag([2e3, 2e3, 2e3, 2e3, 2e3, 2e3]),
    'K_P_omega_ref': np.diag([3.8e1 * 0.8, 3.8e1 * 0.8, 3.8e1 * 1.2]),
    'K_I_omega_ref': np.diag([5e-1, 5e-1, 1e-1]),
    'sigma_x_tran': 0.5,
    'sigma_r_tran': 0.5,
    'sigma_Theta_tran': 0.5,
    'sigma_x_rot': 0.5,
    'sigma_r_rot': 0.5,
    'sigma_Theta_rot': 0.5,
    'dead_zone_delta_tran': 0.5,
    'dead_zone_e0_tran': 0.01,
    'dead_zone_delta_rot': 0.5,
    'dead_zone_e0_rot': 0.002,
}


def decompose_gamma_matrix(matrix: np.ndarray) -> Tuple[float, np.ndarray]:
    """Decompose Gamma into gamma*I + v v^T components."""
    symmetrical = 0.5 * (matrix + matrix.T)
    eigvals, _ = np.linalg.eigh(symmetrical)
    gamma_scale = float(np.min(eigvals))
    if gamma_scale < 0.0:
        gamma_scale = max(gamma_scale, -1e-9)

    rank_one = symmetrical - gamma_scale * np.eye(symmetrical.shape[0])
    eigvals_rank, eigvecs_rank = np.linalg.eigh(rank_one)
    idx = int(np.argmax(eigvals_rank))
    dominant_val = eigvals_rank[idx]

    if dominant_val <= 0.0:
        vector = np.zeros(symmetrical.shape[0])
    else:
        dominant_vec = eigvecs_rank[:, idx]
        vector = np.sqrt(dominant_val) * dominant_vec
        for component in vector:
            if abs(component) > 1e-12:
                if component < 0:
                    vector *= -1.0
                break

    return float(gamma_scale), vector.astype(float)


def load_default_mrac_gains_from_source() -> Dict[str, Any]:
    """Parse mrac_gains.py to extract default gain values without heavy imports."""
    target_names = {
        'Gamma_x_tran', 'Gamma_r_tran', 'Gamma_Theta_tran',
        'Gamma_x_rot', 'Gamma_r_rot', 'Gamma_Theta_rot',
        'K_P_omega_ref', 'K_I_omega_ref',
        'sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
        'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot',
        'dead_zone_delta_tran', 'dead_zone_e0_tran',
        'dead_zone_delta_rot', 'dead_zone_e0_rot'
    }

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

        # Adaptive learning rate matrices (Gamma) using gamma*I + v v^T
        for matrix_name, config in GAMMA_MATRIX_CONFIGS.items():
            prefix = config['prefix']
            size = config['size']
            scale_key = f"{prefix}_gamma"
            vector_keys = [f"{prefix}_v{i + 1}" for i in range(size)]

            if scale_key not in params or not all(key in params for key in vector_keys):
                continue

            gamma_scale = float(params[scale_key])
            if gamma_scale <= 0.0:
                gamma_scale = 1e-6

            vector_values = np.array([float(params[key]) for key in vector_keys], dtype=float)
            gamma_matrix = gamma_scale * np.eye(size) + np.outer(vector_values, vector_values)
            mrac_gains[matrix_name] = np.matrix(gamma_matrix)
        
        # Reference model gains
        if 'K_P_omega_ref_1' in params:
            mrac_gains['K_P_omega_ref'] = np.matrix(np.diag([
                params['K_P_omega_ref_1'], params['K_P_omega_ref_2'], params['K_P_omega_ref_3']
            ]))
        
        if 'K_I_omega_ref_1' in params:
            mrac_gains['K_I_omega_ref'] = np.matrix(np.diag([
                params['K_I_omega_ref_1'], params['K_I_omega_ref_2'], params['K_I_omega_ref_3']
            ]))
        
        # Baseline PID gains
        if 'KP_tran_1' in params:
            mrac_gains['KP_tran'] = np.matrix(np.diag([
                params['KP_tran_1'], params['KP_tran_2'], params['KP_tran_3']
            ]))
        
        if 'KD_tran_1' in params:
            mrac_gains['KD_tran'] = np.matrix(np.diag([
                params['KD_tran_1'], params['KD_tran_2'], params['KD_tran_3']
            ]))
        
        if 'KI_tran_1' in params:
            mrac_gains['KI_tran'] = np.matrix(np.diag([
                params['KI_tran_1'], params['KI_tran_2'], params['KI_tran_3']
            ]))
        
        if 'KP_rot_1' in params:
            mrac_gains['KP_rot'] = np.matrix(np.diag([
                params['KP_rot_1'], params['KP_rot_2'], params['KP_rot_3']
            ]))
        
        # E-modification parameters
        if 'sigma_x_tran' in params:
            mrac_gains['sigma_x_tran'] = params['sigma_x_tran']
            mrac_gains['sigma_r_tran'] = params['sigma_r_tran']
            mrac_gains['sigma_Theta_tran'] = params['sigma_Theta_tran']
            mrac_gains['sigma_x_rot'] = params['sigma_x_rot']
            mrac_gains['sigma_r_rot'] = params['sigma_r_rot']
            mrac_gains['sigma_Theta_rot'] = params['sigma_Theta_rot']
        
        # Dead zone parameters
        if 'dead_zone_delta_tran' in params:
            mrac_gains['dead_zone_delta_tran'] = params['dead_zone_delta_tran']
            mrac_gains['dead_zone_e0_tran'] = params['dead_zone_e0_tran']
            mrac_gains['dead_zone_delta_rot'] = params['dead_zone_delta_rot']
            mrac_gains['dead_zone_e0_rot'] = params['dead_zone_e0_rot']
        
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
        
        # Extract adaptive learning rates via gamma*I + v v^T factorization
        for matrix_name, config in GAMMA_MATRIX_CONFIGS.items():
            if matrix_name not in mrac_gains:
                continue

            matrix = np.array(mrac_gains[matrix_name], dtype=float)
            size = config['size']
            prefix = config['prefix']

            if matrix.shape != (size, size):
                raise ValueError(f"{matrix_name} must be a {size}x{size} matrix, got {matrix.shape}")

            gamma_scale, vector_values = decompose_gamma_matrix(matrix)
            params[f"{prefix}_gamma"] = gamma_scale
            for i, value in enumerate(vector_values, start=1):
                params[f"{prefix}_v{i}"] = value
        
        # Extract reference model gains
        if 'K_P_omega_ref' in mrac_gains:
            params['K_P_omega_ref_1'] = mrac_gains['K_P_omega_ref'][0, 0]
            params['K_P_omega_ref_2'] = mrac_gains['K_P_omega_ref'][1, 1]
            params['K_P_omega_ref_3'] = mrac_gains['K_P_omega_ref'][2, 2]
        
        if 'K_I_omega_ref' in mrac_gains:
            params['K_I_omega_ref_1'] = mrac_gains['K_I_omega_ref'][0, 0]
            params['K_I_omega_ref_2'] = mrac_gains['K_I_omega_ref'][1, 1]
            params['K_I_omega_ref_3'] = mrac_gains['K_I_omega_ref'][2, 2]
        
        # Extract PID gains
        if 'KP_tran' in mrac_gains:
            params['KP_tran_1'] = mrac_gains['KP_tran'][0, 0]
            params['KP_tran_2'] = mrac_gains['KP_tran'][1, 1]
            params['KP_tran_3'] = mrac_gains['KP_tran'][2, 2]
        
        if 'KD_tran' in mrac_gains:
            params['KD_tran_1'] = mrac_gains['KD_tran'][0, 0]
            params['KD_tran_2'] = mrac_gains['KD_tran'][1, 1]
            params['KD_tran_3'] = mrac_gains['KD_tran'][2, 2]
        
        if 'KI_tran' in mrac_gains:
            params['KI_tran_1'] = mrac_gains['KI_tran'][0, 0]
            params['KI_tran_2'] = mrac_gains['KI_tran'][1, 1]
            params['KI_tran_3'] = mrac_gains['KI_tran'][2, 2]
        
        if 'KP_rot' in mrac_gains:
            params['KP_rot_1'] = mrac_gains['KP_rot'][0, 0]
            params['KP_rot_2'] = mrac_gains['KP_rot'][1, 1]
            params['KP_rot_3'] = mrac_gains['KP_rot'][2, 2]
        
        # Extract modification parameters
        if 'sigma_x_tran' in mrac_gains:
            params['sigma_x_tran'] = mrac_gains['sigma_x_tran']
            params['sigma_r_tran'] = mrac_gains['sigma_r_tran']
            params['sigma_Theta_tran'] = mrac_gains['sigma_Theta_tran']
            params['sigma_x_rot'] = mrac_gains['sigma_x_rot']
            params['sigma_r_rot'] = mrac_gains['sigma_r_rot']
            params['sigma_Theta_rot'] = mrac_gains['sigma_Theta_rot']
        
        if 'dead_zone_delta_tran' in mrac_gains:
            params['dead_zone_delta_tran'] = mrac_gains['dead_zone_delta_tran']
            params['dead_zone_e0_tran'] = mrac_gains['dead_zone_e0_tran']
            params['dead_zone_delta_rot'] = mrac_gains['dead_zone_delta_rot']
            params['dead_zone_e0_rot'] = mrac_gains['dead_zone_e0_rot']
        
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
            scale_key = f"{prefix}_gamma"
            vector_keys = [f"{prefix}_v{i + 1}" for i in range(size)]
            if scale_key not in params or not all(key in params for key in vector_keys):
                continue

            gamma_scale = params[scale_key]
            vector_values = [params[key] for key in vector_keys]
            gamma_matrix = gamma_scale * np.eye(size) + np.outer(vector_values, vector_values)
            adaptive_summary[matrix_name] = {
                'gamma': gamma_scale,
                'vector': vector_values,
                'gamma_matrix': gamma_matrix.tolist()
            }

        if adaptive_summary:
            summary['adaptive_learning_rates'] = adaptive_summary
        
        # Reference model gains
        if 'K_P_omega_ref_1' in params:
            summary['reference_model'] = {
                'K_P_omega_ref': [params['K_P_omega_ref_1'], params['K_P_omega_ref_2'], params['K_P_omega_ref_3']],
                'K_I_omega_ref': [params['K_I_omega_ref_1'], params['K_I_omega_ref_2'], params['K_I_omega_ref_3']]
            }
        
        # Baseline PID gains
        if 'KP_tran_1' in params:
            summary['baseline_pid'] = {
                'KP_tran': [params['KP_tran_1'], params['KP_tran_2'], params['KP_tran_3']],
                'KD_tran': [params['KD_tran_1'], params['KD_tran_2'], params['KD_tran_3']],
                'KI_tran': [params['KI_tran_1'], params['KI_tran_2'], params['KI_tran_3']],
                'KP_rot': [params['KP_rot_1'], params['KP_rot_2'], params['KP_rot_3']]
            }
        
        # Modification parameters
        if 'sigma_x_tran' in params:
            summary['modification_parameters'] = {
                'sigma': [params['sigma_x_tran'], params['sigma_r_tran'], params['sigma_Theta_tran'],
                         params['sigma_x_rot'], params['sigma_r_rot'], params['sigma_Theta_rot']],
                'dead_zone': [params['dead_zone_delta_tran'], params['dead_zone_e0_tran'],
                             params['dead_zone_delta_rot'], params['dead_zone_e0_rot']]
            }
        
        return summary
