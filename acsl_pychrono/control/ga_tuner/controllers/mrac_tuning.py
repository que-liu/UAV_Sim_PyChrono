"""
MRAC controller parameter tuning implementation.
"""

from typing import Dict, List, Any, Union, Optional
import numpy as np

from ..core.controller_tuning import ControllerTuningInterface
from ..core.parameter_bounds import ParameterBounds
from ..encoding.parameter_converter import (
    MRACParameterConverter,
    GAMMA_MATRIX_CONFIGS,
    load_default_mrac_gains_from_source,
)
from ..encoding.cholesky_utils import build_cholesky_parameter_bounds
from ..metrics import (
    MRACInnerLoopMetricsCalculator,
    MRACOuterLoopMetricsCalculator,
)

class MRACTuning(ControllerTuningInterface):
    """
    Implementation of parameter tuning for MRAC controllers.
    """
    
    def __init__(self, tuning_config: Dict[str, Any] = None, use_weighted_sum: bool = False):
        """
        Initialize MRAC tuning interface.
        
        Args:
            tuning_config: Optional configuration for tuning
            use_weighted_sum: (default: False for multi-objective)
        """
        self.config = tuning_config or self._get_default_config()
        self.use_weighted_sum = use_weighted_sum
        
        # Initialize shared metrics calculators for inner and outer loop performance
        self.metrics_calculator = MRACInnerLoopMetricsCalculator()
        self.outer_metrics_calculator = MRACOuterLoopMetricsCalculator()
        
        # Load baseline gains directly from MRAC implementation
        self._baseline_gains = self._load_default_mrac_gains()

        # Define parameter bounds using Cholesky-parameterized Gamma matrices
        self.bounds: Dict[str, List[float]] = {}
        self.gamma_parameter_groups: Dict[str, List[str]] = {}

        scale_factor = 10.0
        log_scale_span = np.log(scale_factor)
        vector_factor = 5.0

        def _positive_bounds(value: float, factor: float = 10.0,
                             *, min_lower: float = 1e-6,
                             max_upper: Optional[float] = None) -> List[float]:
            lower = max(min_lower, value / factor)
            upper = max(lower * 2.0, value * factor)
            if max_upper is not None:
                upper = min(upper, max_upper)
                if upper <= lower:
                    upper = lower * 1.5
            return [lower, upper]

        for matrix_name, config in GAMMA_MATRIX_CONFIGS.items():
            prefix = config['prefix']
            baseline_matrix = self._baseline_gains.get(matrix_name)
            if baseline_matrix is None:
                continue

            diag_bound_fn = lambda value, span=log_scale_span: [value - span, value + span]
            matrix_bounds, parameter_names = build_cholesky_parameter_bounds(
                baseline_matrix,
                prefix,
                diag_bound_fn,
                off_diag_factor=vector_factor,
                min_off_diag_span=1.0,
                log_diagonals=True,
            )

            for name, bounds in matrix_bounds.items():
                self.bounds[name] = bounds
            self.gamma_parameter_groups[prefix] = parameter_names

        # Reference model gains derived from baseline diagonal entries
        kp_ref = np.diag(self._baseline_gains['K_P_omega_ref'])
        for i, value in enumerate(kp_ref, start=1):
            self.bounds[f"K_P_omega_ref_{i}"] = _positive_bounds(float(value), factor=5.0)

        ki_ref = np.diag(self._baseline_gains['K_I_omega_ref'])
        for i, value in enumerate(ki_ref, start=1):
            self.bounds[f"K_I_omega_ref_{i}"] = _positive_bounds(float(value), factor=10.0)

        # E-modification parameters
        for name in ['sigma_x_tran', 'sigma_r_tran', 'sigma_Theta_tran',
                     'sigma_x_rot', 'sigma_r_rot', 'sigma_Theta_rot']:
            base_value = float(self._baseline_gains[name])
            self.bounds[name] = _positive_bounds(base_value, factor=3.0,
                                                 min_lower=0.05, max_upper=1.5)


        # cost function weights for weighted sum objective 
        self.cost_weights = {
            'attitude_tracking_error': 1.0,
            'angular_velocity_tracking_error': 0.8,
            'moment_effort': 0.3,
            'position_error': 1.0,
            'velocity_error': 0.5,
            'control_effort': 0.2,
        }

        # Cache ordered parameter names for reuse
        self._parameter_names = list(self.bounds.keys())

        # Instantiate reusable ParameterBounds and converter
        lower_bounds = [self.bounds[name][0] for name in self._parameter_names]
        upper_bounds = [self.bounds[name][1] for name in self._parameter_names]
        self._parameter_bounds = ParameterBounds(
            lower_bounds,
            upper_bounds,
            self._parameter_names,
        )
        self.converter = MRACParameterConverter(self._parameter_bounds)
    
    def _get_default_config(self) -> Dict[str, Any]:
        return {
            # 'tracking_tolerance': 0.1,        # Unused - define acceptable tracking error threshold
            # 'adaptation_threshold': 0.01,     # Unused - set minimum threshold for adaptation
            # 'max_parameter_drift': 10.0,      # Unused - limit adaptive parameter changes
            # 'settling_time_weight': 0.2        # Unused - weight settling time in cost function
        }

    def _load_default_mrac_gains(self) -> Dict[str, Any]:
        """Load baseline gains from the core MRAC implementation."""
        defaults = load_default_mrac_gains_from_source()
        normalized = {}
        for key, value in defaults.items():
            if isinstance(value, np.ndarray):
                normalized[key] = value.copy()
            else:
                normalized[key] = float(value)
        return normalized
    
    def get_parameter_bounds(self) -> ParameterBounds:
        return self._parameter_bounds
    
    def get_parameter_names(self) -> List[str]:
        return self._parameter_names
    
    def get_parameter_groups(self) -> Dict[str, List[str]]:
        """Get parameter grouping information."""
        return self._parameter_bounds.get_parameter_groups()
    
    def vector_to_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """Convert parameter vector to MRAC gain structure."""
        gains = self.converter.vector_to_mrac_gains(parameter_vector)
        
        # Convert numpy matrices to arrays for consistency with rest of codebase
        normalized_gains = {}
        for key, value in gains.items():
            if isinstance(value, np.matrix):
                normalized_gains[key] = np.asarray(value)
            else:
                normalized_gains[key] = value
        return normalized_gains
    
    def gains_to_vector(self, gains: Dict[str, Any]) -> List[float]:
        """Convert MRAC gain structure to parameter vector."""
        prepared_gains = {}
        for key, value in gains.items():
            if isinstance(value, np.ndarray):
                prepared_gains[key] = np.asmatrix(value)
            elif isinstance(value, (list, tuple)):
                prepared_gains[key] = np.asmatrix(value)
            else:
                prepared_gains[key] = value
        
        return self.converter.mrac_gains_to_vector(prepared_gains)
    
    def get_default_gains(self) -> Dict[str, Any]:
        """Get default MRAC gains."""
        defaults = {}
        for key, value in self._baseline_gains.items():
            if isinstance(value, np.ndarray):
                defaults[key] = value.copy()
            else:
                defaults[key] = float(value)
        return defaults
    
    def get_cost_functions(self) -> Dict[str, Any]:
        """Get cost functions and weights."""
        return self.cost_weights
    
    def evaluate_performance(self, log_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Currently not used in the GA flow!
        Evaluate MRAC controller performance using inner and outer loop metrics.
        
        Metrics include:
        - attitude_tracking_error: RMS error in roll/pitch/yaw tracking
        - angular_velocity_tracking_error: RMS error in angular rate tracking
        - moment_effort: RMS control effort (moments u2, u3, u4)
        - position_error: RMS translational position tracking error
        - velocity_error: RMS translational velocity tracking error
        - control_effort: RMS total thrust effort
        """
        # Inner loop metrics capture attitude/rotational performance
        inner_metrics = self.metrics_calculator.compute_all_metrics(log_data)

        # Outer loop metrics capture translational performance
        outer_metrics_obj = self.outer_metrics_calculator.calculate_metrics(log_data)
        outer_metrics = outer_metrics_obj.to_dict()

        # Merge dictionaries
        return {**inner_metrics, **outer_metrics}
    
    def compute_total_cost(self, metrics: Dict[str, float]) -> Union[float, None]:
        """
        Compute total cost from performance metrics.
        
        Args:
            metrics: Dictionary of performance metrics
            
        Returns:
            Weighted sum of metrics if use_weighted_sum=True, None otherwise (for multi-objective)
        """
        if not self.use_weighted_sum:
            return None
        
        total_cost = 0.0
        for metric, value in metrics.items():
            if metric in self.cost_weights:
                total_cost += self.cost_weights[metric] * value
        return total_cost
