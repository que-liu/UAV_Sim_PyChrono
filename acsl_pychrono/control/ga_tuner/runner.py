"""Core utilities for running GA tuning workflows driven by ``TuningConfig``."""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

from acsl_pychrono.control.ga_tuner.controllers.mrac_tuning import MRACTuning
from acsl_pychrono.control.ga_tuner.core.parameter_bounds import ParameterBounds
from acsl_pychrono.control.ga_tuner.ga_config.config import TuningConfig
from acsl_pychrono.control.ga_tuner.uav_evaluators import (
    MRACInnerLoopEvaluator,
    MRACOuterLoopEvaluator,
)
from acsl_pychrono.control.ga_tuner.uav_integration import UAVModelAdapter
from acsl_pychrono.control.ga_tuner.algorithms.pymoo_ga import PymooGATuner
from acsl_pychrono.control.ga_tuner.algorithms.deap_ga import DEAPGATuner


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


def _format_vector(vector: Iterable[float]) -> str:
    return "[" + ", ".join(f"{float(val):.4f}" for val in vector) + "]"


def _build_partial_bounds(
    tuned_subset: Optional[Sequence[str]] = None,
) -> Tuple[ParameterBounds, Sequence[float], Sequence[int], Sequence[str]]:
    tuning = MRACTuning()
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
            raise ValueError(f"Unknown MRAC parameters requested: {sorted(unknown_params)}")
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


class _PartialEvaluatorMixin:
    """Mixin that maps a partial parameter vector onto the full MRAC parameter set."""

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


def run_mrac_ga(config: TuningConfig):
    """Execute the MRAC NSGA-II tuner using the provided configuration."""

    bounds, default_vector, tuned_indices, all_names = _build_partial_bounds(config.tuned_parameters)
    adapter = UAVModelAdapter()
    _apply_mission_overrides(adapter, config.mission_overrides)

    evaluator_type = config.evaluator.evaluator_type
    if evaluator_type == "inner":
        evaluator_cls = PartialMRACInnerLoopEvaluator
        n_objectives = config.n_objectives_inner
    elif evaluator_type == "outer":
        evaluator_cls = PartialMRACOuterLoopEvaluator
        n_objectives = config.n_objectives_outer
    else:
        raise ValueError(f"Unsupported evaluator type '{evaluator_type}'")

    evaluator = evaluator_cls(
        tuned_indices=tuned_indices,
        template_vector=default_vector,
        uav_adapter=adapter,
        log_directory=config.evaluator.log_directory,
        parallel_config=dict(config.evaluator.parallel_config),
        multi_objective=config.evaluator.multi_objective,
    )

    ga_config = config.ga
    
    # Select tuner based on algorithm backend
    if ga_config.algorithm == "DEAP":
        tuner = DEAPGATuner(
            parameter_bounds=bounds,
            fitness_evaluator=evaluator,
            population_size=ga_config.population_size,
            n_generations=ga_config.num_generations,
            crossover_prob=ga_config.crossover_rate,
            mutation_prob=ga_config.mutation_rate,
            selection_method=ga_config.selection_method,
            tournament_size=ga_config.tournament_size,
            random_seed=ga_config.random_seed,
        )
    elif ga_config.algorithm == "PYMOO":
        tuner = _PymooGATunerAdapter(
            parameter_bounds=bounds,
            fitness_evaluator=evaluator,
            population_size=ga_config.population_size,
            n_generations=ga_config.num_generations,
            random_seed=ga_config.random_seed,
            algorithm=ga_config.pymoo_variant,
            n_objectives=n_objectives,
        )
    else:
        raise ValueError(f"Unknown GA backend: {ga_config.algorithm}. Use 'DEAP' or 'PYMOO'")

    result = tuner.optimize(verbose=False)

    if not result.pareto_front:
        raise RuntimeError("GA run did not produce a Pareto front")
    return result, bounds.parameter_names, all_names, evaluator_type


def _apply_mission_overrides(adapter: UAVModelAdapter, overrides):
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


def summarize_result(result, tuned_names: Sequence[str], all_names: Sequence[str], evaluator_type: str = "inner") -> None:
    print("GA tuner MRAC run")
    print(f"Tuned parameters ({len(tuned_names)} / {len(all_names)} total): {list(tuned_names)}")
    
    if evaluator_type == "inner":
        metrics_label = "Inner loop metrics [attitude_error, angular_rate_error, control_effort]"
    else:
        metrics_label = "Outer loop metrics [position_error, velocity_error, control_effort]"

    print("\nPareto front samples (up to first 5 solutions):")
    for idx, (params, metrics) in enumerate(zip(result.pareto_front, result.pareto_fitnesses)):
        tuned_values = list(params)
        print(f"  Solution {idx + 1}:")
        print(f"    Tuned values: {_format_vector(tuned_values)}")
        print(f"    {metrics_label}: {_format_vector(metrics)}")
