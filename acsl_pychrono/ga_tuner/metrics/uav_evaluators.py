"""
UAV-specific fitness evaluators for genetic algorithm optimization.

Architecture Overview:
---------------------
This module provides simulation-based fitness evaluators for UAV controller tuning:

1. UAVSimulationEvaluator: Base class for all simulation-based evaluators
2. PIDSimulationEvaluator: For PID controller tuning (translational control)
3. MRACInnerLoopEvaluator: For MRAC inner loop tuning (attitude/rotational)
4. MRACOuterLoopEvaluator: For MRAC outer loop tuning (position/translational)

"""

from abc import ABC, abstractmethod
from typing import List, Union, Dict, Any, Optional, Tuple
import numpy as np
import os
import pickle
import traceback
from datetime import datetime

from ..core.fitness_evaluator import FitnessEvaluator
from .translational_utils import (
    calculate_position_velocity_rmse,
    extract_position_velocity_vectors,
)
from ..ga_config.metrics_config import MetricsConfig
from ..metrics import MRACInnerLoopMetrics, MRACInnerLoopMetricsCalculator, MRACOuterLoopMetrics, MRACOuterLoopMetricsCalculator, MetricNormalizer


def failure_penalty(multi_objective: bool, n_objectives: int = 3) -> Union[float, List[float]]:
    """
    Return an INF penalty for failed evaluations.
    
    Args:
        multi_objective: Whether using multi-objective optimization
        n_objectives: Number of objectives (used when multi_objective=True)
        
    Returns:
        List of inf values for multi-objective, single inf for single-objective
    """
    return [float("inf")] * n_objectives if multi_objective else float("inf")


def _log_objectives(params: List[float], metrics: List[float], label: str) -> None:
    """Print objective vectors consistently."""
    trimmed_params = params[:3]
    joined = ", ".join(f"{m:.6f}" for m in metrics)
    print(f"[{label}] Parameters {trimmed_params} -> Objectives: {joined}")


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
    
    def _create_eval_log_dir(self, prefix: str) -> str:
        """Create a timestamped log directory for a single evaluation."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        eval_log_dir = os.path.join(self.log_directory, f"{prefix}_{timestamp}")
        os.makedirs(eval_log_dir, exist_ok=True)
        return eval_log_dir

    def _simulate_with_logs(self, parameters: List[float], prefix: str) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
        """
        Run a simulation and attach log metadata to the config.
        Returns (config, log_data).
        """
        eval_log_dir = self._create_eval_log_dir(prefix)

        config = self.uav_adapter.create_simulation_config(
            parameters,
            eval_log_dir,
            controller_type=self._get_controller_type(),
        )

        log_data = self.uav_adapter.run_simulation(config)
        config["log_data"] = log_data

        log_path = self._find_log_file(eval_log_dir)
        config["log_path"] = log_path

        return config, log_data

    def _get_log_data_from_config(self, config: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Retrieve inline log data or load it from disk if available.
        """
        if "log_data" in config and config["log_data"] is not None:
            return config["log_data"]

        log_path = config.get("log_path")
        if log_path and os.path.exists(log_path):
            try:
                with open(log_path, "rb") as f:
                    return pickle.load(f)
            except Exception as e:
                print(f"Warning: Failed to load log data from {log_path}: {e}")
        return None

    def _evaluate_parameters(self, parameters: List[float]) -> float:
        """
        Evaluate fitness for given parameters by running UAV simulation.
        
        Args:
            parameters: Parameter values to evaluate
            
        Returns:
            Fitness value (lower is better)
        """
        config, log_data = self._simulate_with_logs(parameters, prefix="eval")

        if log_data is None:
            return float('inf')  # Worst possible fitness

        # Compute fitness from simulation results
        fitness = self._compute_fitness(config, parameters)
        
        if fitness is None:
            return float('inf')
            
        return fitness

    
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
            log_data = self._get_log_data_from_config(config)
            if log_data is not None:
                return self._compute_fitness_from_log_data(log_data, parameters)
        
        # Fallback: derived class should implement this method
        raise NotImplementedError("Derived class must implement _compute_fitness or _compute_fitness_from_log_data")


class PIDSimulationEvaluator(UAVSimulationEvaluator):
    """
    UAV fitness evaluator for general UAV simulations.
    """
    
    def __init__(self, 
                 uav_adapter,
                 log_directory="simulation_logs",
                 cost_function_weights: Optional[Dict[str, float]] = None,
                 metrics_config: Optional[MetricsConfig] = None,
                 parallel_config=None,
                 multi_objective=False):
        """
        Initialize UAV fitness evaluator.
        
        Args:
            uav_adapter: UAV model adapter instance
            log_directory: Directory to store simulation logs
            cost_function_weights: Weights for different cost components (position, velocity, control)
            metrics_config: Optional MetricsConfig providing default translational weights
            parallel_config: Optional parallel configuration dict
            multi_objective: Whether to use multi-objective optimization
        """
        super().__init__(uav_adapter, log_directory, parallel_config)
        if metrics_config is None or cost_function_weights is None:
            raise ValueError("metrics_config and cost_function_weights must be provided")
        self.cost_function_weights = cost_function_weights
        self.multi_objective = multi_objective
    
    def _get_controller_type(self) -> str:
        """Get the controller type for this evaluator."""
        return 'PID'  # Default to PID
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> float:
        """Compute fitness value from simulation configuration and results."""
        log_data = self._get_log_data_from_config(config)
        if log_data is None:
            return failure_penalty(self.multi_objective)

        result = self._compute_fitness_from_log_data(log_data, parameters)
        if result is None:
            return failure_penalty(self.multi_objective)
        return result
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness from log data dictionary. Returns single objective or multi-objective based on mode."""
        control_input = log_data.get("control_input", {})
        if isinstance(control_input, dict) and len(control_input) > 0:
            translational_control_efforts = []
            for control_array in control_input.values():
                if hasattr(control_array, '__len__') and len(control_array) > 0:
                    control_array = control_array.flatten() if getattr(control_array, "ndim", 1) > 1 else control_array
                    translational_control_efforts.append(np.sqrt(np.mean(control_array**2)))
            translational_control_effort = np.sum(translational_control_efforts) if translational_control_efforts else 0.0
        else:
            translational_control_effort = 0.0

        rmse_values = calculate_position_velocity_rmse(log_data)
        if rmse_values is None:
            return failure_penalty(self.multi_objective)

        pos_tracking_error, vel_tracking_error = rmse_values

        if any(np.isnan(val) or np.isinf(val) for val in (pos_tracking_error, vel_tracking_error, translational_control_effort)):
            return failure_penalty(self.multi_objective)

        if self.multi_objective:
            objectives = [pos_tracking_error, vel_tracking_error, translational_control_effort]
            _log_objectives(parameters, objectives, label="FITNESS")
            return objectives

        w = self.cost_function_weights
        total_fitness = (
            w.get('position_error', 0.0) * pos_tracking_error +
            w.get('velocity_error', 0.0) * vel_tracking_error +
            w.get('translational_control_effort', 0.0) * translational_control_effort
        )
        print(f"[FITNESS] Parameters {parameters[:3]} -> Fitness: {total_fitness:.6f} (pos={pos_tracking_error:.6f}, vel={vel_tracking_error:.6f}, control={translational_control_effort:.6f})")
        return float(total_fitness)

class BaseMRACEvaluator(UAVSimulationEvaluator):
    """Base class for MRAC evaluators with common metric handling."""
    
    def __init__(self,
                 uav_adapter,
                 log_directory: str,
                 parallel_config: Optional[Dict[str, Any]],
                 multi_objective: bool,
                 metric_weights: Dict[str, float],
                 normalize_metrics: bool,
                 metric_names: List[str],
                 metrics_config: MetricsConfig):
        """Initialize base MRAC evaluator with common configuration."""
        super().__init__(uav_adapter, log_directory, parallel_config)
        self.multi_objective = multi_objective
        self.normalize_metrics = normalize_metrics
        self.metric_weights = metric_weights
        
        # Initialize normalizer
        self.normalizer = MetricNormalizer(metric_names=metric_names)
        self.normalizer_fitted = False
        self.collected_metrics = []
        self._objective_count = len(metric_names)

    def _failure_objective(self):
        """Consistent INF penalty for MRAC evaluators."""
        return failure_penalty(self.multi_objective, self._objective_count)

    def _process_metric_array(
        self,
        metrics_array: np.ndarray,
        metric_names: List[str],
        units: Optional[List[str]] = None,
        parameters: Optional[List[float]] = None,
    ) -> Union[float, List[float]]:
        """Handle common metric logging, normalization, and output."""
        self.collected_metrics.append(metrics_array.copy())

        if (
            parameters is not None
            and hasattr(self, '_tuned_indices')
            and hasattr(self, '_template')
            and len(parameters) == len(self._template)
        ):
            ga_parameters = [parameters[i] for i in self._tuned_indices]
            print(f"\n[Metrics - Tuned Parameters (n={len(ga_parameters)})] {ga_parameters[:5]}...:")
        else:
            print(f"\n[Metrics - Raw]:")

        for idx, name in enumerate(metric_names):
            unit = f" {units[idx]}" if units and idx < len(units) else ""
            print(f"  {name.replace('_', ' ').title():30s} {metrics_array[idx]:.6f}{unit}")

        # Apply normalization if enabled and normalizer is fitted
        if self.normalize_metrics and self.normalizer_fitted:
            metrics_array = self.normalizer.transform(metrics_array.reshape(1, -1))[0]
            print(f"[Metrics - Normalized]:")
            for idx, name in enumerate(metric_names):
                unit = f" {units[idx]}" if units and idx < len(units) else ""
                print(f"  {name.replace('_', ' ').title():30s} {metrics_array[idx]:.6f}{unit}")

        if self.multi_objective:
            print(f"  [MODE: Multi-Objective - returning {len(metrics_array)} objectives for Pareto]")
            return metrics_array.tolist()

        total = sum(self.metric_weights[name] * metrics_array[idx] for idx, name in enumerate(metric_names))
        print(f"  [MODE: Single-Objective - weighted sum]")
        print(f"  Weighted Fitness:                {total:.6f}")
        return float(total)

    def fit_normalizer(self):
        """
        Fit the normalizer using collected metrics from all evaluations.
        Fallback method - prefer fit_normalizer_from_baseline().
        """
        if len(self.collected_metrics) > 0:
            metrics_matrix = np.array(self.collected_metrics)
            self.normalizer.fit(metrics_matrix)
            self.normalizer_fitted = True
            print(f"\n[NORMALIZER] Fitted with {len(self.collected_metrics)} samples")

    def fit_normalizer_from_reference(self, default_parameters: List[float]) -> bool:
        """
        Fit the normalizer using default gains as reference.
        
        Runs one simulation with default parameters to get reference metrics,
        then normalizes all future metrics relative to this reference.
        
        Args:
            default_parameters: Full parameter vector from default gains
            
        Returns:
            True if reference evaluation succeeded, False otherwise
        """
        print("\n[NORMALIZER] Evaluating default parameters for normalization reference...")
        
        reference_result = self.evaluate_individual(default_parameters, use_cache=False)
        
        # Check if evaluation failed
        if reference_result is None:
            print("[NORMALIZER] WARNING: Reference evaluation returned None")
            return False
        
        reference_metrics = np.array(reference_result if isinstance(reference_result, list) else [reference_result])
        
        # Check for inf/nan in reference
        if np.any(np.isinf(reference_metrics)) or np.any(np.isnan(reference_metrics)):
            print("[NORMALIZER] WARNING: Reference evaluation produced inf/nan values")
            return False
        
        self.normalizer.fit_from_reference(reference_metrics)
        self.normalizer_fitted = True
        return True


class MRACInnerLoopEvaluator(BaseMRACEvaluator):
    """
    MRAC inner loop fitness evaluator with separate metrics.
    Computes attitude tracking, angular velocity tracking, and moment effort.
    """
    
    def __init__(self, 
                 uav_adapter,
                 log_directory="simulation_logs",
                 parallel_config=None,
                 multi_objective: bool = False,
                 metric_weights: Dict[str, float] = None,
                 normalize_metrics: bool = True,
                 metrics_config: MetricsConfig = None):
        """Initialize MRAC inner loop evaluator."""
        if metrics_config is None or metric_weights is None:
            raise ValueError("metrics_config and metric_weights must be provided for MRACInnerLoopEvaluator")

        super().__init__(
            uav_adapter=uav_adapter,
            log_directory=log_directory,
            parallel_config=parallel_config,
            multi_objective=multi_objective,
            metric_weights=metric_weights,
            normalize_metrics=normalize_metrics,
            metric_names=[
                'attitude_tracking_error',
                'angular_velocity_tracking_error',
                'rotational_control_effort'
            ],
            metrics_config=metrics_config,
        )
        
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
            config, log_data = self._simulate_with_logs(parameters, prefix="inner_loop_eval")

            if log_data is None:
                print(f"Warning: Simulation failed for parameters {parameters[:3]}")
                return self._get_fallback_fitness()

            # Compute inner loop fitness from log data
            return self._compute_fitness_from_log_data(log_data, parameters)
            
        except Exception as e:
            print(f"Error in inner loop evaluation: {e}")
            traceback.print_exc()
            return self._get_fallback_fitness()
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness value from MRAC simulation results."""
        # Check if we have log data directly in config (from simulation)
        log_data = self._get_log_data_from_config(config)
        if log_data is not None:
            # Use log data directly from simulation
            return self._compute_fitness_from_log_data(log_data, parameters)
        
        # Fallback: try to load from file
        print(f"Warning: Log data not found, using fallback fitness")
        return self._get_fallback_fitness()
    
    def _get_fallback_fitness(self) -> Union[float, List[float]]:
        """Return fallback fitness for failed simulations."""
        return self._failure_objective()
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness metrics from log data dictionary."""
        # Calculate inner loop metrics
        metrics = self._calculate_inner_loop_metrics(log_data, parameters)
        
        # Convert metrics to array for normalization
        metrics_array = np.array([
            metrics.attitude_tracking_error,
            metrics.angular_velocity_tracking_error,
            metrics.rotational_control_effort
        ])
        return self._process_metric_array(
            metrics_array,
            [
                'attitude_tracking_error',
                'angular_velocity_tracking_error',
                'rotational_control_effort'
            ],
            units=["rad", "rad/s", "N·m"],
            parameters=parameters,
        )
    
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


class MRACOuterLoopEvaluator(BaseMRACEvaluator):
    """
    MRAC outer loop fitness evaluator using translational metrics
    (position error, velocity error, control effort).
    """

    def __init__(self,
                 uav_adapter,
                 log_directory: str = "simulation_logs",
                 parallel_config: Optional[Dict[str, Any]] = None,
                 multi_objective: bool = False,
                 metric_weights: Dict[str, float] = None,
                 normalize_metrics: bool = True,
                 metrics_config: MetricsConfig = None):
        """Initialize MRAC outer loop evaluator."""
        if metrics_config is None or metric_weights is None:
            raise ValueError("metrics_config and metric_weights must be provided for MRACOuterLoopEvaluator")

        super().__init__(
            uav_adapter=uav_adapter,
            log_directory=log_directory,
            parallel_config=parallel_config,
            multi_objective=multi_objective,
            metric_weights=metric_weights,
            normalize_metrics=normalize_metrics,
            metric_names=[
                'position_error',
                'velocity_error',
                'translational_control_effort'
            ],
            metrics_config=metrics_config,
        )
        
        self.metrics_calculator = MRACOuterLoopMetricsCalculator()

    def _get_controller_type(self) -> str:
        """Outer loop tuning targets MRAC controller."""
        return 'MRAC'

    def _compute_fitness_from_log_data(self,
                                       log_data: Dict[str, Any],
                                       parameters: List[float]) -> Union[float, List[float]]:
        metrics = self.metrics_calculator.calculate_metrics(log_data)
        
        # Convert metrics to array for normalization
        metrics_array = np.array([
            metrics.position_error,
            metrics.velocity_error,
            metrics.translational_control_effort
        ])
        return self._process_metric_array(
            metrics_array,
            [
                'position_error',
                'velocity_error',
                'translational_control_effort'
            ],
            units=["m", "m/s", "N"],
            parameters=parameters,
        )
