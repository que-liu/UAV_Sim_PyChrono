"""MRAC NSGA-II check using the PyChrono-based UAV simulation."""

from __future__ import annotations

import sys
import os
from pathlib import Path
from typing import Iterable, List, Sequence

# Ensure project root is the working directory
PROJECT_ROOT = Path(__file__).resolve().parents[4]  # adjust if needed
os.chdir(PROJECT_ROOT)
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
# print("Working directory set to:", PROJECT_ROOT)

from acsl_pychrono.control.ga_tuner.controllers.mrac_tuning import MRACTuning
from acsl_pychrono.control.ga_tuner.core.parameter_bounds import ParameterBounds
from acsl_pychrono.control.ga_tuner.uav_evaluators import MRACInnerLoopEvaluator
# from acsl_pychrono.control.ga_tuner.uav_evaluators import MRACOuterLoopEvaluator
from acsl_pychrono.control.ga_tuner.uav_integration import UAVModelAdapter
from acsl_pychrono.control.ga_tuner.algorithms.pymoo_ga import PymooGATuner as _BasePymooGATuner

class _PymooGATunerAdapter(_BasePymooGATuner):
    """Minimal adapter satisfying BaseGATuner abstract interface."""
    def _initialize_population(self):  # pragma: no cover - not used by pymoo backend
        return []

    def _select_parents(self, population, fitnesses):  # pragma: no cover - not used
        return []

    def _crossover(self, parents):  # pragma: no cover - not used
        return []

    def _mutate(self, offspring):  # pragma: no cover - not used
        return []

    def _survive(self, population, offspring, population_fitnesses, offspring_fitnesses):  # pragma: no cover - not used
        return []


def _format_vector(vector: Iterable[float]) -> str:
    return "[" + ", ".join(f"{float(val):.4f}" for val in vector) + "]"


def _build_partial_bounds():
    tuning = MRACTuning()
    base_bounds = tuning.get_parameter_bounds()
    all_names = list(base_bounds.parameter_names)
    default_gains = tuning.get_default_gains()
    default_vector = tuning.gains_to_vector(default_gains)

    # Example subset targeting translation adaptive gains (advisor parameterization)
    tuned_subset = {
        # "gamma_x_tran_gamma",
        # "gamma_x_tran_v1",
        # "gamma_x_tran_v2",
        # "gamma_x_tran_v3",
        # "gamma_x_tran_v4",
        # "gamma_x_tran_v5",
        # "gamma_x_tran_v6",
        "gamma_x_rot_gamma",
        "gamma_x_rot_v1",
        "gamma_x_rot_v2",
        "gamma_x_rot_v3",
    }

    lower_bounds: List[float] = []
    upper_bounds: List[float] = []
    tuned_indices: List[int] = []
    tuned_names: List[str] = []

    for name in all_names:
        if name in tuned_subset:
            idx = all_names.index(name)
            tuned_indices.append(idx)
            tuned_names.append(name)
            lower_bounds.append(float(base_bounds.lower_bounds[idx]))
            upper_bounds.append(float(base_bounds.upper_bounds[idx]))

    partial_bounds = ParameterBounds(lower_bounds, upper_bounds, tuned_names)
    return partial_bounds, default_vector, tuned_indices, all_names


class PartialMRACInnerLoopEvaluator(MRACInnerLoopEvaluator):
# class PartialMRACOuterLoopEvaluator(MRACOuterLoopEvaluator):
    def __init__(self, tuned_indices: Sequence[int], template_vector: Sequence[float], **kwargs):
        super().__init__(**kwargs)
        self._tuned_indices = list(tuned_indices)
        self._template = list(template_vector)

    def evaluate_individual(self, parameters, use_cache: bool = True):
        full_vector = self._template.copy()
        for idx, value in zip(self._tuned_indices, parameters):
            full_vector[idx] = float(value)
        return super().evaluate_individual(full_vector, use_cache)


def run_mrac_nsga2():
    bounds, default_vector, tuned_indices, all_names = _build_partial_bounds()
    adapter = UAVModelAdapter()
    adapter.base_config['mission'].update({
        'trajectory_type': 'piecewise_polynomial_trajectory',  
        'duration': 30.0,
        'visualization': False,
        'wrapper_flag': True
    })
    evaluator = PartialMRACInnerLoopEvaluator(
    # evaluator = PartialMRACOuterLoopEvaluator(
        tuned_indices=tuned_indices,
        template_vector=default_vector,
        uav_adapter=adapter,
        log_directory="simulation_logs/ga_mrac_nsga2",
        parallel_config={"enabled": True, "n_workers": 2, "use_processes": False},
        multi_objective=True,
    )

    tuner = _PymooGATunerAdapter(
        parameter_bounds=bounds,
        fitness_evaluator=evaluator,
        population_size=5,
        n_generations=2,
        random_seed=42,
        algorithm="NSGA2",
        n_objectives=3, # for inner-loop evaluator
        # n_objectives=3, # for outer-loop evaluator
    )

    result = tuner.optimize(verbose=False)

    if not result.pareto_front:
        raise RuntimeError("NSGA-II run did not produce a Pareto front")
    return result, bounds.parameter_names, all_names


def _summarize(result, tuned_names: Sequence[str], all_names: Sequence[str]) -> None:
    print("GA tuner MRAC NSGA-II check")
    # print(f"Computation time: {float(result.computation_time or 0.0):.2f}s")
    print(f"Tuned parameters ({len(tuned_names)} / {len(all_names)} total): {list(tuned_names)}")

    print("\nPareto front samples (up to first 5 solutions):")
    for idx, (params, metrics) in enumerate(zip(result.pareto_front, result.pareto_fitnesses)):
        '''if idx >= 5:
            print("  … (additional solutions omitted)")
            break'''
        tuned_values = list(params)
        print(f"  Solution {idx + 1}:")
        print(f"    Tuned values: {_format_vector(tuned_values)}")
        print(
            "    Metrics [position_error, velocity_error, control_effort]: "
            f"{_format_vector(metrics)}"
        )


def main() -> None:
    # debug check UAV model availability
    # adapter = UAVModelAdapter()
    # print(adapter.uav_available)

    result, tuned_names, all_names = run_mrac_nsga2()
    _summarize(result, tuned_names, all_names)


if __name__ == "__main__":
    main()
