"""Structured configuration helpers for GA tuning runs."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Any, Dict, Literal, Mapping, MutableMapping, Optional, Sequence

from .ga_config import GAConfig


@dataclass(frozen=True)
class EvaluatorConfig:
    evaluator_type: Literal["inner", "outer"] = "inner"
    log_directory: str = "simulation_logs/ga_mrac_nsga2"
    parallel_config: Dict[str, Any] = field(
        default_factory=lambda: {"enabled": True, "n_workers": 2, "use_processes": False}
    )
    multi_objective: bool = True


@dataclass(frozen=True)
class TuningConfig:
    tuned_parameters: Sequence[str] = field(default_factory=tuple)
    mission_overrides: Dict[str, Any] = field(default_factory=dict)
    evaluator: EvaluatorConfig = field(default_factory=EvaluatorConfig)
    ga: GAConfig = field(default_factory=GAConfig)
    n_objectives_inner: int = 3
    n_objectives_outer: int = 3

    def get_objective_count(self, evaluator_type: Optional[str] = None) -> int:
        evaluator = evaluator_type or self.evaluator.evaluator_type
        evaluator = evaluator.lower()
        if evaluator not in {"inner", "outer"}:
            raise ValueError(f"Unknown evaluator type '{evaluator}'")
        return self.n_objectives_inner if evaluator == "inner" else self.n_objectives_outer


_BASE_TUNING_CONFIG = TuningConfig()


def tuning_config_from_mapping(
    data: Mapping[str, Any], base: Optional[TuningConfig] = None
) -> TuningConfig:
    """Build a :class:`TuningConfig` by applying a mapping of overrides."""
    base_config = base or _BASE_TUNING_CONFIG

    tuned_parameters = tuple(data.get("tuned_parameters", base_config.tuned_parameters))
    mission_overrides = dict(base_config.mission_overrides)
    mission_overrides.update(data.get("mission_overrides", {}))

    evaluator = _merge_dataclass(EvaluatorConfig, base_config.evaluator, data.get("evaluator"))
    ga = _merge_dataclass(GAConfig, base_config.ga, data.get("ga"))

    n_objectives_inner = data.get("n_objectives_inner", base_config.n_objectives_inner)
    n_objectives_outer = data.get("n_objectives_outer", base_config.n_objectives_outer)

    return TuningConfig(
        tuned_parameters=tuned_parameters,
        mission_overrides=mission_overrides,
        evaluator=evaluator,
        ga=ga,
        n_objectives_inner=n_objectives_inner,
        n_objectives_outer=n_objectives_outer,
    )

def _merge_dataclass(cls, base_value, overrides: Optional[MutableMapping[str, Any]]):
    if overrides is None:
        return base_value
    init_kwargs = {}
    for field_info in fields(cls):
        if field_info.name in overrides:
            value = overrides[field_info.name]
            if isinstance(value, Mapping) and isinstance(getattr(base_value, field_info.name), dict):
                init_kwargs[field_info.name] = dict(value)
            else:
                init_kwargs[field_info.name] = value
        else:
            attr_value = getattr(base_value, field_info.name)
            if isinstance(attr_value, dict):
                init_kwargs[field_info.name] = dict(attr_value)
            else:
                init_kwargs[field_info.name] = attr_value
    return cls(**init_kwargs)


DEFAULT_TUNING_CONFIG = tuning_config_from_mapping(
    {
        "tuned_parameters": (
            "gamma_x_rot_L11",
            "gamma_x_rot_L21",
            "gamma_x_rot_L22",
            "gamma_x_rot_L31",
        ),
        "mission_overrides": {
            "trajectory_type": "piecewise_polynomial_trajectory",
            "duration": 30.0,
            "visualization": False,
            "wrapper_flag": True,
        },
        "evaluator": {
            "evaluator_type": "inner",  # Change to "outer" for outer loop metrics
            "log_directory": "simulation_logs/ga_mrac_nsga2",
        },
        "ga": {
            "controller_type": "MRAC",
            "algorithm": "PYMOO",
            "pymoo_variant": "NSGA2",
            "population_size": 2,
            "num_generations": 2,
            "random_seed": 42,
        },
    },
    base=_BASE_TUNING_CONFIG,
)
