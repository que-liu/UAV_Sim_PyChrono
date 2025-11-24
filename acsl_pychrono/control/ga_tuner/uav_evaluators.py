"""
UAV-specific fitness evaluators for genetic algorithm optimization.
"""

from abc import ABC, abstractmethod
from typing import List, Union, Dict, Any, Optional, Tuple
import numpy as np
import os
import pickle
from datetime import datetime

from .core.fitness_evaluator import FitnessEvaluator
from .metrics.translational_utils import (
    calculate_position_velocity_rmse,
    extract_position_velocity_vectors,
)


class MetricNormalizer:
    """
    Normalizer for inner loop metrics using max-min normalization for sensitivity analysis after all simulations are complete.
    """
    
    def __init__(self):
        self.metric_mins = None
        self.metric_maxs = None
        self.metric_names = [
            'attitude_tracking_error',
            'angular_velocity_tracking_error',
            'moment_effort'
        ]
        self.fitted = False
    
    def fit(self, metrics_array: np.ndarray):
        """
        Fit the normalizer by computing min and max for each metric.
        
        Args:
            metrics_array: Array of shape (n_samples, n_metrics) containing all metric values
        """
        if metrics_array.ndim != 2:
            raise ValueError(f"Expected 2D array, got shape {metrics_array.shape}")
        
        if metrics_array.shape[1] != 3:
            raise ValueError(f"Expected 3 metrics, got {metrics_array.shape[1]}")
        
        # Filter out inf values for computing bounds
        valid_mask = ~np.isinf(metrics_array)
        
        self.metric_mins = np.zeros(3)
        self.metric_maxs = np.zeros(3)
        
        for i in range(3):
            valid_values = metrics_array[valid_mask[:, i], i]
            if len(valid_values) > 0:
                self.metric_mins[i] = np.min(valid_values)
                self.metric_maxs[i] = np.max(valid_values)
            else:
                # If all values are inf, set default range
                self.metric_mins[i] = 0.0
                self.metric_maxs[i] = 1.0
        
        self.fitted = True
        
        print("\n[METRIC NORMALIZER] Fitted normalization bounds:")
        for i, name in enumerate(self.metric_names):
            print(f"  {name:40s}: [{self.metric_mins[i]:.6f}, {self.metric_maxs[i]:.6f}]")
    
    def transform(self, metrics_array: np.ndarray) -> np.ndarray:
        """
        Normalize metrics using min-max normalization to [0, 1].
        
        Args:
            metrics_array: Array of shape (n_samples, n_metrics) or (n_metrics,)
            
        Returns:
            Normalized array with same shape as input
        """
        if not self.fitted:
            raise RuntimeError("Normalizer must be fitted before transform. Call fit() first.")
        
        original_shape = metrics_array.shape
        metrics_array = np.atleast_2d(metrics_array)
        
        if metrics_array.shape[1] != 3:
            raise ValueError(f"Expected 3 metrics, got {metrics_array.shape[1]}")
        
        normalized = np.zeros_like(metrics_array)
        
        for i in range(3):
            metric_range = self.metric_maxs[i] - self.metric_mins[i]
            
            if metric_range > 1e-10:
                # Standard min-max normalization
                normalized[:, i] = (metrics_array[:, i] - self.metric_mins[i]) / metric_range
            else:
                # Range is too small, set to 0.5 (middle of [0, 1])
                normalized[:, i] = 0.5
            
            # Handle inf values: set to 1.0 (worst case in normalized space)
            inf_mask = np.isinf(metrics_array[:, i])
            normalized[inf_mask, i] = 1.0
            
            # Clip to [0, 1] to handle numerical errors
            normalized[:, i] = np.clip(normalized[:, i], 0.0, 1.0)
        
        # Restore original shape if input was 1D
        if len(original_shape) == 1:
            return normalized[0]
        
        return normalized
    
    def fit_transform(self, metrics_array: np.ndarray) -> np.ndarray:
        """
        Fit the normalizer and transform in one step.
        
        Args:
            metrics_array: Array of shape (n_samples, n_metrics)
            
        Returns:
            Normalized array
        """
        self.fit(metrics_array)
        return self.transform(metrics_array)
    
    def get_bounds(self) -> Dict[str, Tuple[float, float]]:
        """
        Get the fitted min-max bounds for each metric.
        
        Returns:
            Dictionary mapping metric names to (min, max) tuples
        """
        if not self.fitted:
            raise RuntimeError("Normalizer must be fitted first. Call fit() first.")
        
        return {
            name: (self.metric_mins[i], self.metric_maxs[i])
            for i, name in enumerate(self.metric_names)
        }


class InnerLoopSensitivityHelper:
    """
    Helper class for conducting sensitivity analysis on inner loop metrics.
    Handles metric collection, normalization, and preparation for Morris/Sobol analysis.
    """
    
    def __init__(self, evaluator: 'MRACInnerLoopEvaluator'):
        """
        Initialize the sensitivity helper.
        
        Args:
            evaluator: MRACInnerLoopEvaluator instance (must have multi_objective=True)
        """
        if not isinstance(evaluator, MRACInnerLoopEvaluator):
            raise TypeError("evaluator must be an instance of MRACInnerLoopEvaluator")
        
        if not evaluator.multi_objective:
            raise ValueError("evaluator must have multi_objective=True for sensitivity analysis")
        
        self.evaluator = evaluator
        self.normalizer = MetricNormalizer()
        
        # Storage for all evaluations
        self.parameter_vectors = []
        self.raw_metrics = []
        self.normalized_metrics = None
    
    def evaluate_parameter_set(self, parameters: List[float]) -> List[float]:
        """
        Evaluate a single parameter set and store results.
        
        Args:
            parameters: Parameter values to evaluate
            
        Returns:
            Raw metrics (not normalized)
        """
        metrics = self.evaluator.evaluate_individual(parameters, use_cache=False)
        
        self.parameter_vectors.append(parameters)
        self.raw_metrics.append(metrics)
        
        return metrics
    
    def evaluate_parameter_sets(self, parameter_array: np.ndarray) -> np.ndarray:
        """
        Evaluate multiple parameter sets and store results.
        
        Args:
            parameter_array: Array of shape (n_samples, n_parameters)
            
        Returns:
            Array of raw metrics with shape (n_samples, 3)
        """
        results = []
        for params in parameter_array:
            metrics = self.evaluate_parameter_set(params.tolist())
            results.append(metrics)
        
        return np.array(results)
    
    def normalize_metrics(self) -> np.ndarray:
        """
        Normalize all collected metrics using min-max normalization.
        This should be called after all simulations are complete.
        
        Returns:
            Array of normalized metrics with shape (n_samples, 3)
        """
        if len(self.raw_metrics) == 0:
            raise RuntimeError("No metrics collected yet. Call evaluate_parameter_set first.")
        
        raw_array = np.array(self.raw_metrics)
        self.normalized_metrics = self.normalizer.fit_transform(raw_array)
        
        print(f"\n[SENSITIVITY] Normalized {len(self.raw_metrics)} metric sets")
        
        return self.normalized_metrics
    
    def get_normalized_metric(self, metric_index: int) -> np.ndarray:
        """
        Get normalized values for a specific metric across all evaluations.
        
        Args:
            metric_index: Index of metric (0-2)
            
        Returns:
            1D array of normalized metric values
        """
        if self.normalized_metrics is None:
            raise RuntimeError("Metrics not normalized yet. Call normalize_metrics first.")
        
        if not 0 <= metric_index < 3:
            raise ValueError(f"metric_index must be 0-2, got {metric_index}")
        
        return self.normalized_metrics[:, metric_index]
    
    def get_normalized_metric_by_name(self, metric_name: str) -> np.ndarray:
        """
        Get normalized values for a specific metric by name.
        
        Args:
            metric_name: Name of metric ('attitude_tracking_error', 
                        'angular_velocity_tracking_error', 'moment_effort')
                        
        Returns:
            1D array of normalized metric values
        """
        metric_index = self.normalizer.metric_names.index(metric_name)
        return self.get_normalized_metric(metric_index)
    
    def prepare_for_morris(self) -> Dict[str, np.ndarray]:
        """
        Prepare data for Morris sensitivity analysis.
        
        Returns:
            Dictionary with keys for each metric containing normalized values
        """
        if self.normalized_metrics is None:
            self.normalize_metrics()
        
        return {
            'attitude_tracking_error': self.normalized_metrics[:, 0],
            'angular_velocity_tracking_error': self.normalized_metrics[:, 1],
            'moment_effort': self.normalized_metrics[:, 2]
        }
    
    def prepare_for_sobol(self, metric_index: int = None) -> np.ndarray:
        """
        Prepare data for Sobol sensitivity analysis.
        
        Args:
            metric_index: If specified, return only that metric. 
                         If None, return all metrics stacked.
                         
        Returns:
            Array of normalized metric values
        """
        if self.normalized_metrics is None:
            self.normalize_metrics()
        
        if metric_index is not None:
            return self.get_normalized_metric(metric_index)
        
        return self.normalized_metrics
    
    def get_summary(self) -> Dict[str, Any]:
        """
        Get summary statistics for all metrics (raw and normalized).
        
        Returns:
            Dictionary with statistics
        """
        if len(self.raw_metrics) == 0:
            return {"status": "No evaluations performed"}
        
        raw_array = np.array(self.raw_metrics)
        
        summary = {
            "n_evaluations": len(self.raw_metrics),
            "raw_metrics": {},
            "normalized_metrics": {}
        }
        
        for i, name in enumerate(self.normalizer.metric_names):
            raw_values = raw_array[:, i]
            valid_mask = ~np.isinf(raw_values)
            
            if np.any(valid_mask):
                summary["raw_metrics"][name] = {
                    "min": float(np.min(raw_values[valid_mask])),
                    "max": float(np.max(raw_values[valid_mask])),
                    "mean": float(np.mean(raw_values[valid_mask])),
                    "std": float(np.std(raw_values[valid_mask])),
                    "n_valid": int(np.sum(valid_mask)),
                    "n_inf": int(np.sum(~valid_mask))
                }
            else:
                summary["raw_metrics"][name] = {"status": "All values are inf"}
        
        if self.normalized_metrics is not None:
            for i, name in enumerate(self.normalizer.metric_names):
                norm_values = self.normalized_metrics[:, i]
                summary["normalized_metrics"][name] = {
                    "min": float(np.min(norm_values)),
                    "max": float(np.max(norm_values)),
                    "mean": float(np.mean(norm_values)),
                    "std": float(np.std(norm_values))
                }
        
        return summary
    
    def reset(self):
        """Reset all stored data."""
        self.parameter_vectors = []
        self.raw_metrics = []
        self.normalized_metrics = None
        self.normalizer = MetricNormalizer()


class UAVSimulationEvaluator(FitnessEvaluator):
    """
    Base class for UAV simulation-based fitness evaluation.
    This provides an interface between genetic algorithms and the UAV model.
    """
    
    def __init__(self, 
                 uav_adapter,  # UAVModelAdapter
                 log_directory="simulation_logs",
                 parallel_config=None):
        """
        Initialize UAV simulation evaluator.
        
        Args:
            uav_adapter: UAV model adapter instance
            log_directory: Directory to store simulation logs
            parallel_config: Optional parallel configuration dict
        """
        self.uav_adapter = uav_adapter
        self.log_directory = log_directory
        os.makedirs(log_directory, exist_ok=True)
        
        # Parse parallel configuration
        if parallel_config is None:
            parallel_config = {}
        
        parallel_enabled = parallel_config.get('enabled', False)
        n_workers = parallel_config.get('n_workers', 1)
        use_processes = parallel_config.get('use_processes', False)
        
        # Call parent constructor with evaluation function
        super().__init__(
            evaluation_function=self._evaluate_parameters,
            n_objectives=1,  # Default to single objective
            parallel=parallel_enabled,
            n_workers=n_workers,
            use_processes=use_processes
        )
        
        # Print parallelization status
        if parallel_enabled:
            worker_type = 'processes' if use_processes else 'threads'
            print(f"[PARALLEL] Enabled with {n_workers} {worker_type}")
            if not use_processes:
                print("[PARALLEL] Using threads (processes disabled to avoid pickling issues)")
            else:
                print("[PARALLEL] Using processes for true parallelism")
        else:
            print("[PARALLEL] Disabled - running sequentially")
        
        # Set controller type for parallel processing
        self.controller_type = self._get_controller_type()
    
    def _evaluate_parameters(self, parameters: List[float]) -> float:
        """
        Evaluate fitness for given parameters by running UAV simulation.
        
        Args:
            parameters: Parameter values to evaluate
            
        Returns:
            Fitness value (lower is better)
        """
        try:
            # Create unique log directory for this evaluation
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            eval_log_dir = os.path.join(self.log_directory, f"eval_{timestamp}")
            os.makedirs(eval_log_dir, exist_ok=True)
            
            # Create simulation configuration
            config = self.uav_adapter.create_simulation_config(
                parameters, 
                eval_log_dir,
                controller_type=self._get_controller_type()
            )
            
            # Run simulation
            log_data = self.uav_adapter.run_simulation(config)
            
            if log_data is None:
                return float('inf')  # Worst possible fitness
            
            # Add log data directly to config for fitness computation
            config['log_data'] = log_data
            
            # Find the log file path after simulation (for backup)
            log_path = self._find_log_file(eval_log_dir)
            config['log_path'] = log_path
            
            # Compute fitness from simulation results
            fitness = self._compute_fitness(config, parameters)
            
            if fitness is None:
                return float('inf')
            
            return fitness
            
        except Exception as e:
            print(f"Error evaluating parameters {parameters}: {e}")
            return float('inf')  # Worst possible fitness
    
    def _find_log_file(self, log_dir: str) -> Optional[str]:
        """Find the simulation log file in the given directory."""
        if not os.path.exists(log_dir):
            return None
            
        # Look for the specific log file name
        log_path = os.path.join(log_dir, "log_dict.pkl")
        if os.path.exists(log_path):
            return log_path
        
        # The evaluator might create a subdirectory with timestamp, so we need to find it
        try:
            subdirs = [d for d in os.listdir(log_dir) if os.path.isdir(os.path.join(log_dir, d))]
            if subdirs:
                # Use the most recent subdirectory
                subdirs.sort()
                latest_subdir = subdirs[-1]
                log_path = os.path.join(log_dir, latest_subdir, "log_dict.pkl")
                if os.path.exists(log_path):
                    return log_path
            
            # Fallback: look for any .pkl file
            log_files = [f for f in os.listdir(log_dir) if f.endswith('.pkl')]
            if log_files:
                return os.path.join(log_dir, log_files[-1])
                
        except Exception as e:
            print(f"Error finding log file in {log_dir}: {e}")
            
        return None

    
    @abstractmethod
    def _get_controller_type(self) -> str:
        """Get the controller type for this evaluator."""
        pass
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> float:
        """Compute fitness value from simulation configuration and results."""
        # Try to call the derived class method if it exists
        if hasattr(self, '_compute_fitness_from_log_data'):
            log_data = config.get('log_data')
            if log_data is not None:
                return self._compute_fitness_from_log_data(log_data, parameters)
        
        # Fallback: derived class should implement this method
        raise NotImplementedError("Derived class must implement _compute_fitness or _compute_fitness_from_log_data")


class UAVFitnessEvaluator(UAVSimulationEvaluator):
    """
    UAV fitness evaluator for general UAV simulations.
    """
    
    def __init__(self, 
                 uav_adapter,
                 log_directory="simulation_logs",
                 cost_function_weights=None,
                 parallel_config=None,
                 multi_objective=False):
        """
        Initialize UAV fitness evaluator.
        
        Args:
            uav_adapter: UAV model adapter instance
            log_directory: Directory to store simulation logs
            cost_function_weights: Weights for different cost components
            parallel_config: Optional parallel configuration dict
            multi_objective: Whether to use multi-objective optimization
        """
        super().__init__(uav_adapter, log_directory, parallel_config)
        self.cost_function_weights = cost_function_weights or {'total': 1.0}
        self.multi_objective = multi_objective
    
    def _get_controller_type(self) -> str:
        """Get the controller type for this evaluator."""
        return 'PID'  # Default to PID
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> float:
        """Compute fitness value from simulation configuration and results."""
        try:
            # Check if we have log data directly in config (from simulation)
            log_data = config.get('log_data')
            
            if log_data is not None:
                # Use log data directly from simulation
                result = self._compute_fitness_from_log_data(log_data, parameters)
                if result is None:
                    if hasattr(self, 'multi_objective') and self.multi_objective:
                        return [float('inf'), float('inf'), float('inf')]
                    else:
                        return float('inf')
                return result
            else:
                # Simulation failed, no log data
                if hasattr(self, 'multi_objective') and self.multi_objective:
                    return [float('inf'), float('inf'), float('inf')]
                else:
                    return float('inf')
            
            # Fallback: try to load from file
            log_path = config.get('log_path')
            
            if not log_path or not os.path.exists(log_path):
                if hasattr(self, 'multi_objective') and self.multi_objective:
                    return [float('inf'), float('inf'), float('inf')]
                else:
                    return float('inf')  # Worst possible fitness
            
            # Load simulation log
            with open(log_path, 'rb') as f:
                log = pickle.load(f)
            
            result = self._compute_fitness_from_log_data(log, parameters)
            return result
            
        except Exception as e:
            print(f"Error computing fitness: {e}")
            import traceback
            traceback.print_exc()
            if hasattr(self, 'multi_objective') and self.multi_objective:
                return [float('inf'), float('inf'), float('inf')]
            else:
                return float('inf')
        
        if hasattr(self, 'multi_objective') and self.multi_objective:
            return [float('inf'), float('inf'), float('inf')]
        else:
            return float('inf')  # Fallback
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness from log data dictionary. Returns single objective or multi-objective based on mode."""
        try:
            # Extract control effort data
            control_input = log_data.get("control_input", {})
            if isinstance(control_input, dict) and len(control_input) > 0:
                try:
                    # Control input is a dictionary with keys like 'U1', 'U2', 'U3', 'U4'
                    # Each value is a numpy array of control inputs over time
                    control_efforts = []
                    for key, control_array in control_input.items():
                        if hasattr(control_array, '__len__') and len(control_array) > 0:
                            # Calculate RMS for this control input
                            if control_array.ndim > 1:
                                # Flatten if multi-dimensional
                                control_array = control_array.flatten()
                            control_efforts.append(np.sqrt(np.mean(control_array**2)))
                    
                    if control_efforts:
                        # Total control effort as sum of individual control efforts
                        control_effort = np.sum(control_efforts)
                    else:
                        control_effort = 0.0
                except (ValueError, TypeError, IndexError) as e:
                    print(f"Warning: Error calculating control effort: {e}")
                    control_effort = 0.0
            else:
                control_effort = 0.0
            
            # Compute tracking RMSE metrics using shared helper
            rmse_values = calculate_position_velocity_rmse(log_data)
            if rmse_values is None:
                if hasattr(self, 'multi_objective') and self.multi_objective:
                    return [float('inf'), float('inf'), float('inf')]
                return float('inf')

            pos_tracking_error, vel_tracking_error = rmse_values
            
            # Check for NaN or Inf values
            if (np.isnan(pos_tracking_error) or np.isinf(pos_tracking_error) or
                np.isnan(vel_tracking_error) or np.isinf(vel_tracking_error) or
                np.isnan(control_effort) or np.isinf(control_effort)):
                if hasattr(self, 'multi_objective') and self.multi_objective:
                    return [float('inf'), float('inf'), float('inf')]
                else:
                    return float('inf')
            
            # Return based on optimization mode
            if hasattr(self, 'multi_objective') and self.multi_objective:
                # Multi-objective: return list of objectives
                objectives = [pos_tracking_error, vel_tracking_error, control_effort]
                print(f"[FITNESS] Parameters {parameters[:3]} -> Objectives: pos={pos_tracking_error:.6f}, vel={vel_tracking_error:.6f}, control={control_effort:.6f}")
                return objectives
            else:
                # Single-objective: return weighted sum
                weights = [3.0, 0.1, 0.01]  # [pos_weight, vel_weight, control_weight]
                total_fitness = (weights[0] * pos_tracking_error + 
                               weights[1] * vel_tracking_error + 
                               weights[2] * control_effort)
                print(f"[FITNESS] Parameters {parameters[:3]} -> Fitness: {total_fitness:.6f} (pos={pos_tracking_error:.6f}, vel={vel_tracking_error:.6f}, control={control_effort:.6f})")
                return float(total_fitness)
                
        except (KeyError, IndexError, ValueError) as e:
            print(f"Warning: Error processing simulation log: {e}")
            import traceback
            traceback.print_exc()
            if hasattr(self, 'multi_objective') and self.multi_objective:
                return [float('inf'), float('inf'), float('inf')]
            else:
                return float('inf')
                
        except Exception as e:
            print(f"Error computing fitness: {e}")
            import traceback
            traceback.print_exc()
            return float('inf')  # Worst possible fitness


class MRACSimulationEvaluator(UAVSimulationEvaluator):
    """
    MRAC-specific fitness evaluator for UAV simulations.
    """
    
    def __init__(self, 
                 uav_adapter,
                 log_directory="simulation_logs",
                 cost_function_weights=None,
                 parallel_config=None):
        """
        Initialize MRAC simulation evaluator.
        
        Args:
            uav_adapter: UAV model adapter instance
            log_directory: Directory to store simulation logs
            cost_function_weights: Weights for different cost components
            parallel_config: Optional parallel configuration dict
        """
        super().__init__(uav_adapter, log_directory, parallel_config)
        self.cost_function_weights = cost_function_weights or {
            'tracking_error': 1.0,
            'adaptation_rate': 0.5,
            'control_effort': 0.3
        }
    
    def _get_controller_type(self) -> str:
        """Get the controller type for this evaluator."""
        return 'MRAC'
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> float:
        """Compute fitness value from MRAC simulation results."""
        try:
            # Check if we have log data directly in config (from simulation)
            log_data = config.get('log_data')
            if log_data is not None:
                # Use log data directly from simulation
                return self._compute_fitness_from_log_data(log_data, parameters)
            
            # Fallback: try to load from file
            log_path = config.get('log_path')
            if not log_path or not os.path.exists(log_path):
                print(f"Warning: Log file not found at {log_path}, using fallback fitness")
                # Use parameter-based fallback with some variation to avoid zero sensitivity
                return 1000.0 + sum(abs(p) for p in parameters) * 0.1
            
            # Load the log data
            import pickle
            with open(log_path, 'rb') as f:
                log_data = pickle.load(f)
            
            return self._compute_fitness_from_log_data(log_data, parameters)
            
        except Exception as e:
            print(f"Error computing MRAC fitness: {e}")
            return float('inf')
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> float:
        """Compute fitness from log data dictionary."""
        try:
            
            # Create a more sensitive fitness function based on available data
            fitness = self._compute_sensitive_fitness(log_data, parameters)
            
            print(f"[MRAC FITNESS] Parameters {parameters[:3]} -> Fitness: {fitness:.6f}")
            return fitness
            
        except Exception as e:
            print(f"Warning: Failed to compute MRAC fitness from simulation logs: {e}")
            # Fallback to parameter-based fitness with high penalty and some variation
            return 1000.0 + sum(abs(p) for p in parameters) * 0.1
    
    def _compute_sensitive_fitness(self, log_data: Dict[str, Any], parameters: List[float]) -> float:
        """Compute a more sensitive fitness function based on available simulation data."""
        try:
            vectors = extract_position_velocity_vectors(log_data)
            if vectors is None:
                return float('inf')

            actual_pos, desired_pos, actual_vel, desired_vel = vectors

            # Remove NaN values
            valid_mask = ~(np.isnan(actual_pos).any(axis=1) | np.isnan(desired_pos).any(axis=1))
            actual_pos_clean = actual_pos[valid_mask]
            desired_pos_clean = desired_pos[valid_mask]
            
            if len(actual_pos_clean) == 0:
                return float('inf')
            
            # Calculate tracking errors
            pos_error = actual_pos_clean - desired_pos_clean
            pos_error_magnitude = np.linalg.norm(pos_error, axis=1)
            
            # Remove NaN values for velocity
            valid_vel_mask = ~(np.isnan(actual_vel).any(axis=1) | np.isnan(desired_vel).any(axis=1))
            actual_vel_clean = actual_vel[valid_vel_mask]
            desired_vel_clean = desired_vel[valid_vel_mask]
            
            if len(actual_vel_clean) > 0:
                vel_error = actual_vel_clean - desired_vel_clean
                vel_error_magnitude = np.linalg.norm(vel_error, axis=1)
            else:
                vel_error_magnitude = np.array([0.0])
            
            # Calculate control effort
            thrust_arrays = []
            for motor_key, thrust_data in log_data['thrust_motors_N'].items():
                thrust_array = np.array(thrust_data).flatten()
                thrust_arrays.append(thrust_array)
            
            if thrust_arrays:
                all_thrusts = np.column_stack(thrust_arrays)
                total_thrust_per_timestep = np.sum(all_thrusts, axis=1)
                valid_thrust = total_thrust_per_timestep[~np.isnan(total_thrust_per_timestep)]
                control_effort = np.mean(valid_thrust) if len(valid_thrust) > 0 else 0.0
            else:
                control_effort = 0.0
            
            # Calculate fitness components based on ACTUAL PERFORMANCE
            pos_fitness = np.mean(pos_error_magnitude)
            vel_fitness = np.mean(vel_error_magnitude)
            
            # Check if parameters are actually being applied by looking for gamma values in log
            gamma_values = log_data.get('gamma_x_rot', [])
            if gamma_values:
                print(f"[DEBUG] Found gamma_x_rot in log: {gamma_values}")
            else:
                print(f"[DEBUG] No gamma_x_rot found in log data - parameters may not be applied!")
            
            # Normalize control effort to make it comparable to tracking errors
            # Typical control effort is ~20-40 N, normalize by expected value (30 N)
            normalized_control_effort = control_effort / 30.0
            total_fitness = (
                20.0 * pos_fitness +                   
                10.0 * vel_fitness +              
                0.1 * normalized_control_effort         
            )
            
            # Add very small random component to break ties (but minimal)
            param_hash = sum(parameters) % 1000
            total_fitness += param_hash * 0.0001  # Very small tie-breaker
            
            # Debug: Print detailed fitness breakdown
            print(f"[DEBUG] Performance-based fitness for params {parameters[:3]}:")
            print(f"  pos_error: {pos_fitness:.6f} → weighted: {10.0 * pos_fitness:.6f}")
            print(f"  vel_error: {vel_fitness:.6f} → weighted: {1.0 * vel_fitness:.6f}")
            print(f"  control_effort: {control_effort:.6f} → normalized: {normalized_control_effort:.6f} → weighted: {0.1 * normalized_control_effort:.6f}")
            print(f"  tie_breaker: {param_hash * 0.0001:.6f}")
            print(f"  TOTAL FITNESS: {total_fitness:.6f}")
            
            if np.isnan(total_fitness) or np.isinf(total_fitness):
                return float('inf')
            
            return float(total_fitness)
            
        except Exception as e:
            print(f"Error in sensitive fitness computation: {e}")
            return float('inf')


# Import MRAC inner loop metrics from centralized metrics module
from .metrics import MRACInnerLoopMetrics, MRACOuterLoopMetricsCalculator


class MRACInnerLoopEvaluator(UAVSimulationEvaluator):
    """
    MRAC inner loop fitness evaluator with separate metrics.
    Computes attitude tracking, angular velocity tracking, and moment effort.
    """
    
    def __init__(self, 
                 uav_adapter,
                 log_directory="simulation_logs",
                 parallel_config=None,
                 multi_objective=False):
        """
        Initialize MRAC inner loop evaluator.
        
        Args:
            uav_adapter: UAV model adapter instance
            log_directory: Directory to store simulation logs
            parallel_config: Optional parallel configuration dict
            multi_objective: Whether to return list of objectives (True) or combined fitness (False)
        """
        super().__init__(uav_adapter, log_directory, parallel_config)
        self.multi_objective = multi_objective
        
        # Initialize shared inner loop metrics calculator for consistency
        from .metrics import MRACInnerLoopMetricsCalculator
        self.metrics_calculator = MRACInnerLoopMetricsCalculator()
    
    def _get_controller_type(self) -> str:
        """Get the controller type for this evaluator."""
        return 'MRAC'
    
    def _evaluate_parameters(self, parameters: List[float]) -> Union[float, List[float]]:
        """
        Override the base evaluation method to use inner loop metrics.
        
        Args:
            parameters: Parameter values to evaluate
            
        Returns:
            Inner loop metrics (list if multi_objective=True, float otherwise)
        """
        try:
            # Create unique log directory for this evaluation
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            eval_log_dir = os.path.join(self.log_directory, f"inner_loop_eval_{timestamp}")
            os.makedirs(eval_log_dir, exist_ok=True)
            
            # Create simulation configuration
            config = self.uav_adapter.create_simulation_config(
                parameters, 
                eval_log_dir,
                controller_type=self._get_controller_type()
            )
            
            # Run simulation - this returns log_data directly
            log_data = self.uav_adapter.run_simulation(config)
            
            if log_data is None:
                print(f"Warning: Simulation failed for parameters {parameters[:3]}")
                return self._get_fallback_fitness()
            
            # Add log data to config for consistency with base class
            config['log_data'] = log_data
            
            # Find the log file path after simulation (for backup)
            log_path = self._find_log_file(eval_log_dir)
            config['log_path'] = log_path
            
            # Compute inner loop fitness from log data
            return self._compute_fitness_from_log_data(log_data, parameters)
            
        except Exception as e:
            print(f"Error in inner loop evaluation: {e}")
            import traceback
            traceback.print_exc()
            return self._get_fallback_fitness()
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness value from MRAC simulation results."""
        # Check if we have log data directly in config (from simulation)
        log_data = config.get('log_data')
        if log_data is not None:
            # Use log data directly from simulation
            return self._compute_fitness_from_log_data(log_data, parameters)
        
        # Fallback: try to load from file
        log_path = config.get('log_path')
        if not log_path or not os.path.exists(log_path):
            print(f"Warning: Log file not found at {log_path}, using fallback fitness")
            return self._get_fallback_fitness()
        
        # Load the log data
        import pickle
        with open(log_path, 'rb') as f:
            log_data = pickle.load(f)
        
        return self._compute_fitness_from_log_data(log_data, parameters)
    
    def _get_fallback_fitness(self) -> Union[float, List[float]]:
        """Return fallback fitness for failed simulations."""
        if self.multi_objective:
            return [float('inf'), float('inf'), float('inf')]
        else:
            return float('inf')
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness metrics from log data dictionary."""
        # Calculate inner loop metrics
        metrics = self._calculate_inner_loop_metrics(log_data, parameters)
        
        # Print detailed metrics - show actual parameters being optimized if this is a partial evaluator
        if hasattr(self, '_tuned_indices') and hasattr(self, '_template'):
            # This is a partial evaluator - show the actual GA parameters being optimized
            ga_parameters = [parameters[i] for i in self._tuned_indices]
            print(f"\n[Parameters tuned] {ga_parameters}:")
        else:
            pass
        print(f"  Attitude Tracking Error:        {metrics.attitude_tracking_error:.6f} rad")
        print(f"  Angular Velocity Tracking Error: {metrics.angular_velocity_tracking_error:.6f} rad/s")
        print(f"  Moment Effort:                   {metrics.moment_effort:.6f} N·m")
        
        if self.multi_objective:
            # Return list of objectives for multi-objective optimization
            return metrics.to_list()
        else:
            # Return combined fitness with equal weights for single-objective
            # User can adjust weights or use multi-objective mode for sensitivity analysis
            combined_fitness = (
                metrics.attitude_tracking_error +
                metrics.angular_velocity_tracking_error +
                0.1 * metrics.moment_effort
            )
            print(f"  Combined Fitness:                {combined_fitness:.6f}")
            return float(combined_fitness)
    
    def _calculate_inner_loop_metrics(self, log_data: Dict[str, Any], parameters: List[float]) -> MRACInnerLoopMetrics:
        """Calculate all inner loop metrics from simulation log data."""
        # Use shared metrics calculator for consistency with mrac_tuning.py
        metrics = self.metrics_calculator.compute_metrics_object(log_data)
        
        # Validate all metrics
        for key, value in metrics.to_dict().items():
            if value is None or np.isnan(value) or np.isinf(value):
                print(f"Warning: Invalid metric {key} = {value}, setting to high penalty")
                setattr(metrics, key, 999.0)
        
        return metrics


class MRACOuterLoopEvaluator(UAVSimulationEvaluator):
    """
    MRAC outer loop fitness evaluator using translational metrics
    (position error, velocity error, control effort).
    """

    def __init__(self,
                 uav_adapter,
                 log_directory: str = "simulation_logs",
                 parallel_config: Optional[Dict[str, Any]] = None,
                 multi_objective: bool = False,
                 cost_function_weights: Optional[Dict[str, float]] = None):
        super().__init__(uav_adapter, log_directory, parallel_config)
        self.multi_objective = multi_objective
        self.cost_function_weights = cost_function_weights or {
            'position_error': 1.0,
            'velocity_error': 0.5,
            'control_effort': 0.2
        }
        self.metrics_calculator = MRACOuterLoopMetricsCalculator()

    def _get_controller_type(self) -> str:
        """Outer loop tuning targets MRAC controller."""
        return 'MRAC'

    def _compute_fitness_from_log_data(self,
                                       log_data: Dict[str, Any],
                                       parameters: List[float]) -> Union[float, List[float]]:
        metrics = self.metrics_calculator.calculate_metrics(log_data)
        metrics_dict = metrics.to_dict()

        # Log metrics for debugging
        if hasattr(self, '_tuned_indices') and hasattr(self, '_template'):
            ga_parameters = [parameters[i] for i in self._tuned_indices]
            print(f"\n[OUTER LOOP METRICS] Tuned parameters {ga_parameters}:")
        else:
            print(f"\n[OUTER LOOP METRICS] Parameters preview {parameters[:3]}:")
        print(f"  Position Error:  {metrics_dict['position_error']:.6f} m")
        print(f"  Velocity Error:  {metrics_dict['velocity_error']:.6f} m/s")
        print(f"  Control Effort:  {metrics_dict['control_effort']:.6f} N")

        if self.multi_objective:
            return metrics.to_list()

        total_cost = 0.0
        for metric_name, weight in self.cost_function_weights.items():
            total_cost += weight * metrics_dict.get(metric_name, 0.0)

        print(f"  Weighted Fitness: {total_cost:.6f}")
        return float(total_cost)
