"""
Integration module for connecting the GA tuner with the UAV simulation model.
"""

from pathlib import Path
from typing import Dict, Any, Literal, Optional, List, Sequence, Tuple, Union
from copy import deepcopy

from acsl_pychrono.simulation.simulation import Simulation
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.config.config import MissionConfig, WrapperParams, SimulationConfig

from .uav_evaluators import PIDSimulationEvaluator, MRACInnerLoopEvaluator, MRACOuterLoopEvaluator
from .controllers.pid_tuning import PIDTuning
from .controllers.mrac_tuning import MRACTuning
from .algorithms.deap_ga import DEAPGATuner
from .algorithms.pymoo_ga import PymooGATuner
from .core.parameter_bounds import ParameterBounds
from .ga_config import GAConfig
from .ga_config.metrics_config import MetricsConfig

class UAVModelAdapter:
    """
    Adapter class that connects the GA tuner with the UAV simulation model.
    """
    
    def __init__(self, 
                 base_config=None,  # Optional[Dict[str, Any]]
                 log_base_dir="simulation_logs"):  # str
        """
        Initialize the UAV model adapter.
        
        Args:
            base_config: Base configuration for simulations
            log_base_dir: Base directory for simulation logs
        """
        
        base = base_config or {}
        self.base_config = {
            'mission': base.get('mission', MissionConfig()),
            'wrapper_params': base.get('wrapper_params', WrapperParams()),
        }
        self.log_base_dir = Path(log_base_dir)
        self.log_base_dir.mkdir(parents=True, exist_ok=True)
    
    
    def create_simulation_config(self,
                               parameters: List[float],
                               log_dir: str,
                               controller_type: str):
        """
        Create simulation configuration with given parameters and controller type.
        """

        Path(log_dir).mkdir(parents=True, exist_ok=True)

        mission_cfg = self._clone_mission_config()
        wrapper_params = self._build_wrapper_params(controller_type, parameters)

        mission_cfg.controller_type = controller_type
        mission_cfg.wrapper_batch_dir = log_dir
        mission_cfg.visualization_flag = False
        mission_cfg.wrapper_flag = True

        sim_config = SimulationConfig(
            mission_config=mission_cfg,
            wrapper_params=wrapper_params
        )
        return {
            'simulation_config': sim_config,
            'log_directory': log_dir,
            'parameters': parameters,
            'controller_type': controller_type
        }
    
    def _clone_mission_config(self):
        """Clone the base MissionConfig to avoid cross-run mutation."""
        return deepcopy(self.base_config['mission'])

    def _build_wrapper_params(self, controller_type: str, parameters: List[float]):
        """Create wrapper params with external parameters for GA tuning."""
        wrapper_params = deepcopy(self.base_config['wrapper_params'])
        controller_params = list(parameters)

        if controller_type == 'MRAC':
            wrapper_params.external_controller_params = {'mrac_params': controller_params}
        elif controller_type.upper() == 'PID':
            wrapper_params.external_controller_params = {'pid_params': controller_params}
        else:
            raise ValueError(f"Unsupported controller type for GA tuning: {controller_type}")

        return wrapper_params
    
    def run_simulation(self, config):  # Dict[str, Any] -> Dict[str, Any] or None
        """
        Run UAV simulation with given configuration.
        
        Args:
            config: Simulation configuration
            
        Returns:
            Log data dictionary if simulation completed successfully, None otherwise
        """
        sim_config = config['simulation_config']
        simulation = Simulation(sim_config)
        git_info = Logging.getGitRepoInfo()
        log_dict = simulateMission(simulation, git_info)
        if log_dict is None:
            print("Warning: simulateMission finished without producing log data.")
        return log_dict
    
    def get_simulation_status(self):  # -> Dict[str, Any]
        """Get status of UAV simulation system."""
        return {
            'base_config': self.base_config,
            'log_base_dir': self.log_base_dir
        }


class _PymooGATunerAdapter(PymooGATuner):
    """Adapter to satisfy BaseGATuner abstract interface (methods not used by Pymoo)."""

    def _initialize_population(self):
        return []

    def _select_parents(self, population, fitnesses):
        return []

    def _crossover(self, parents):
        return []

    def _mutate(self, offspring):
        return []

    def _survive(self, population, offspring, population_fitnesses, offspring_fitnesses):
        return []


class _PartialEvaluatorMixin:
    """Mixin that maps a partial parameter vector onto the full parameter set."""

    def __init__(self, tuned_indices: Sequence[int], template_vector: Sequence[float], **kwargs):
        super().__init__(**kwargs)
        self._tuned_indices = list(tuned_indices)
        self._template = list(template_vector)

    def evaluate_individual(self, parameters, use_cache: bool = True):
        full_vector = self._template.copy()
        for idx, value in zip(self._tuned_indices, parameters):
            full_vector[idx] = float(value)
        return super().evaluate_individual(full_vector, use_cache)


class PartialMRACInnerLoopEvaluator(_PartialEvaluatorMixin, MRACInnerLoopEvaluator):
    """Partial-parameter adapter for the inner-loop MRAC evaluator."""


class PartialMRACOuterLoopEvaluator(_PartialEvaluatorMixin, MRACOuterLoopEvaluator):
    """Partial-parameter adapter for the outer-loop MRAC evaluator."""


class PartialPIDSimulationEvaluator(_PartialEvaluatorMixin, PIDSimulationEvaluator):
    """Partial-parameter adapter for the PID evaluator."""


def _build_partial_bounds_generic(
    tuning,
    tuned_subset: Optional[Sequence[str]] = None,
) -> Tuple[ParameterBounds, Sequence[float], Sequence[int], Sequence[str]]:
    """
    Build parameter bounds for partial parameter tuning (works for both PID and MRAC).
    
    Args:
        tuning: Tuning instance (PIDTuning or MRACTuning)
        tuned_subset: Optional list of parameter names to tune (None = all)
        
    Returns:
        Tuple of (partial_bounds, default_vector, tuned_indices, all_names)
    """
    base_bounds = tuning.get_parameter_bounds()
    all_names = list(base_bounds.parameter_names)
    default_gains = tuning.get_default_gains()
    default_vector = tuning.gains_to_vector(default_gains)

    lower_bounds: List[float] = []
    upper_bounds: List[float] = []
    tuned_indices: List[int] = []
    tuned_names: List[str] = []
    name_to_index = {name: idx for idx, name in enumerate(all_names)}

    if tuned_subset:
        unknown_params = set(tuned_subset) - set(all_names)
        if unknown_params:
            raise ValueError(f"Unknown parameters requested: {sorted(unknown_params)}")
        subset = set(tuned_subset)
    else:
        subset = set(all_names)

    for name in all_names:
        if name in subset:
            idx = name_to_index[name]
            tuned_indices.append(idx)
            tuned_names.append(name)
            lower_bounds.append(float(base_bounds.lower_bounds[idx]))
            upper_bounds.append(float(base_bounds.upper_bounds[idx]))

    partial_bounds = ParameterBounds(lower_bounds, upper_bounds, tuned_names)
    return partial_bounds, default_vector, tuned_indices, all_names


# Backward compatibility alias
_build_partial_bounds = _build_partial_bounds_generic


def _apply_mission_overrides(adapter: UAVModelAdapter, overrides: Optional[Dict[str, Any]]):
    """Apply mission-level overrides to the adapter base config."""
    if not overrides:
        return

    mission_cfg = adapter.base_config.get("mission")
    if mission_cfg is None:
        adapter.base_config["mission"] = dict(overrides)
        return

    if isinstance(mission_cfg, dict):
        mission_cfg.update(dict(overrides))
    else:
        for key, value in dict(overrides).items():
            setattr(mission_cfg, key, value)


def create_uav_ga_tuner(
    ga_config: GAConfig,
    evaluator_type: Literal["inner", "outer"] = "outer",
    metrics_config: Optional[MetricsConfig] = None,
    tuned_parameters: Optional[Sequence[str]] = None,
    mission_overrides: Optional[Dict[str, Any]] = None,
    uav_adapter: Optional[UAVModelAdapter] = None,
    log_directory: str = "simulation_logs/ga_optimization",
    parallel_config: Optional[Dict[str, Any]] = None,
    n_objectives: int = 3,
):
    """
    Factory function to create a GA tuner configured for UAV simulations.
    
    Supports both PID and MRAC controllers, with inner/outer loop evaluation
    for MRAC and partial parameter tuning.
    
    Args:
        ga_config: GAConfig dataclass providing controller/algorithm and GA hyperparameters
        evaluator_type: For MRAC, 'inner' (attitude) or 'outer' (position) loop tuning
        metrics_config: Optional MetricsConfig for metric weights (uses defaults if None)
        tuned_parameters: Optional list of parameter names to tune (None = all parameters)
        mission_overrides: Optional dict of mission config overrides
        uav_adapter: UAV adapter instance (created if None)
        log_directory: Directory for simulation logs
        parallel_config: Optional parallel evaluation config
        n_objectives: Number of objectives for multi-objective optimization
        
    Returns:
        Configured GA tuner
    """
    controller = ga_config.controller_type.upper()
    algorithm_name = ga_config.algorithm.upper()
    
    # Use default metrics config if not provided
    if metrics_config is None:
        metrics_config = MetricsConfig()
    
    # Create UAV adapter if not provided
    if uav_adapter is None:
        uav_adapter = UAVModelAdapter()
    
    # Apply mission overrides
    if mission_overrides:
        _apply_mission_overrides(uav_adapter, mission_overrides)
    
    # Determine objective count expected from evaluators/GA
    objective_count = n_objectives if ga_config.multi_objective else 1

    # Build parameter bounds and evaluator based on controller type
    if controller == 'PID':
        pid_tuning = PIDTuning()
        
        # Build partial bounds if tuning subset of parameters
        parameter_bounds, default_vector, tuned_indices, all_names = _build_partial_bounds_generic(
            pid_tuning, tuned_parameters
        )
        tuned_names = list(parameter_bounds.parameter_names)
        
        cost_function_weights = {
            'position_error': metrics_config.outer_loop_weights.position_error,
            'velocity_error': metrics_config.outer_loop_weights.velocity_error,
            'translational_control_effort': metrics_config.outer_loop_weights.translational_control_effort,
        }
        
        # Use partial evaluator if not tuning all parameters
        if tuned_parameters and len(tuned_parameters) < len(all_names):
            fitness_evaluator = PartialPIDSimulationEvaluator(
                tuned_indices=tuned_indices,
                template_vector=default_vector,
                uav_adapter=uav_adapter,
                log_directory=log_directory,
                cost_function_weights=cost_function_weights,
                metrics_config=metrics_config,
                parallel_config=parallel_config,
                multi_objective=ga_config.multi_objective,
            )
        else:
            fitness_evaluator = PIDSimulationEvaluator(
                uav_adapter=uav_adapter,
                log_directory=log_directory,
                cost_function_weights=cost_function_weights,
                metrics_config=metrics_config,
                parallel_config=parallel_config,
                multi_objective=ga_config.multi_objective,
            )
        
    elif controller == 'MRAC':
        mrac_tuning = MRACTuning()
        
        # Build partial bounds if tuning subset of parameters
        parameter_bounds, default_vector, tuned_indices, all_names = _build_partial_bounds(
            mrac_tuning, tuned_parameters
        )
        tuned_names = list(parameter_bounds.parameter_names)
        
        # Select evaluator class and weights based on loop type
        if evaluator_type == "inner":
            evaluator_cls = PartialMRACInnerLoopEvaluator
            metric_weights = {
                'attitude_tracking_error': metrics_config.inner_loop_weights.attitude_tracking_error,
                'angular_velocity_tracking_error': metrics_config.inner_loop_weights.angular_velocity_tracking_error,
                'rotational_control_effort': metrics_config.inner_loop_weights.rotational_control_effort,
            }
        elif evaluator_type == "outer":
            evaluator_cls = PartialMRACOuterLoopEvaluator
            metric_weights = {
                'position_error': metrics_config.outer_loop_weights.position_error,
                'velocity_error': metrics_config.outer_loop_weights.velocity_error,
                'translational_control_effort': metrics_config.outer_loop_weights.translational_control_effort,
            }
        else:
            raise ValueError(f"Unknown evaluator_type: {evaluator_type}. Use 'inner' or 'outer'")
        
        fitness_evaluator = evaluator_cls(
            tuned_indices=tuned_indices,
            template_vector=default_vector,
            uav_adapter=uav_adapter,
            log_directory=log_directory,
            parallel_config=parallel_config,
            multi_objective=ga_config.multi_objective,
            metric_weights=metric_weights,
            normalize_metrics=metrics_config.normalize_metrics,
            metrics_config=metrics_config,
        )
    else:
        raise ValueError(f"Unknown controller_type: {controller}. Use 'PID' or 'MRAC'")
    
    # Build tuner kwargs from ga_config
    tuner_kwargs = {
        'population_size': ga_config.population_size,
        'n_generations': ga_config.num_generations,
        'random_seed': ga_config.random_seed,
    }
    
    # Create appropriate tuner
    if algorithm_name == 'DEAP':
        tuner_kwargs.update({
            'crossover_prob': ga_config.crossover_rate,
            'mutation_prob': ga_config.mutation_rate,
            'selection_method': ga_config.selection_method,
            'tournament_size': ga_config.tournament_size,
            'multi_objective': ga_config.multi_objective,
            'n_objectives': objective_count,
        })
        tuner = DEAPGATuner(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    elif algorithm_name == 'PYMOO':
        tuner_kwargs.update({
            'algorithm': ga_config.pymoo_variant,
            'n_objectives': objective_count,
        })
        if ga_config.pymoo_algorithm_params:
            tuner_kwargs['algorithm_params'] = dict(ga_config.pymoo_algorithm_params)
        tuner = _PymooGATunerAdapter(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm_name}. Use 'DEAP' or 'PYMOO'")
    
    # Fit normalizer from default gains if normalization is enabled (MRAC only)
    if controller == 'MRAC' and metrics_config.normalize_metrics:
        if hasattr(fitness_evaluator, 'fit_normalizer_from_reference'):
            success = fitness_evaluator.fit_normalizer_from_reference(default_vector)
            if not success:
                print("[WARNING] Failed to fit normalizer from reference - normalization disabled")
    
    # Attach metadata for result summarization
    tuner._tuned_names = tuned_names
    tuner._all_names = all_names
    tuner._evaluator_type = evaluator_type
    
    return tuner


def summarize_tuner_result(tuner, result) -> None:
    """
    Print a summary of GA tuner results.
    
    Args:
        tuner: The tuner returned by create_uav_ga_tuner
        result: The OptimizationResult from tuner.optimize()
    """
    tuned_names = getattr(tuner, '_tuned_names', [])
    all_names = getattr(tuner, '_all_names', tuned_names)
    evaluator_type = getattr(tuner, '_evaluator_type', 'outer')
    
    print("GA tuner run complete")
    print(f"Tuned parameters ({len(tuned_names)} / {len(all_names)} total): {list(tuned_names)}")
    
    if evaluator_type == "inner":
        metrics_label = "Inner loop metrics [attitude_error, angular_rate_error, control_effort]"
    else:
        metrics_label = "Outer loop metrics [position_error, velocity_error, control_effort]"

    if result.pareto_front is not None and len(result.pareto_front) > 0:
        print(f"\nPareto front ({len(result.pareto_front)} solutions, showing up to 5):")
        for idx, (params, metrics) in enumerate(zip(result.pareto_front[:5], result.pareto_fitnesses[:5])):
            tuned_values = [f"{v:.4f}" for v in params]
            metrics_str = [f"{m:.4f}" for m in metrics]
            print(f"  Solution {idx + 1}:")
            print(f"    Tuned values: [{', '.join(tuned_values)}]")
            print(f"    {metrics_label}: [{', '.join(metrics_str)}]")
    elif result.best_parameters is not None:
        print(f"\nBest solution:")
        print(f"  Parameters: {result.best_parameters}")
        print(f"  Fitness: {result.get_best_fitness()}")
