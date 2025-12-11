"""
Metrics configuration for GA tuning.

This module defines configuration for both inner loop and outer loop evaluators,
supporting both single-objective and multi-objective optimization modes.

Multi-objective modes:
- "full": 6 objectives (position, velocity, attitude, angular_velocity, trans_effort, rot_effort)
- "grouped": 3 objectives (translational_error, rotational_error, control_effort)
- False: Single objective (weighted sum)
"""

from dataclasses import dataclass, field
from typing import Union, Literal


@dataclass(frozen=True)
class InnerLoopMetricWeights:
    """
    Weights for MRAC inner loop metrics (attitude/rotational control).
    """
    attitude_tracking_error: float = 1.0          # RMS error in roll/pitch/yaw tracking (rad)
    angular_velocity_tracking_error: float = 1.0  # RMS error in angular rate tracking (rad/s)
    rotational_control_effort: float = 0.1        # RMS control moments u2, u3, u4 (N·m)


@dataclass(frozen=True)
class OuterLoopMetricWeights:
    """
    Weights for outer loop metrics (position/translational control).
    """
    position_error: float = 3.0                      # RMS position tracking error (m)
    velocity_error: float = 0.1                      # RMS velocity tracking error (m/s)
    translational_control_effort: float = 0.01       # RMS total thrust from motors (N)


@dataclass(frozen=True)
class GroupedMetricWeights:
    """
    Weights for grouped metrics (3 composite objectives).
    Used when multi_objective="grouped".
    
    Each group combines related metrics:
    - translational_error = position_error + velocity_error
    - rotational_error = attitude_tracking_error + angular_velocity_tracking_error  
    - control_effort = translational_control_effort + rotational_control_effort
    """
    # Weights within each group (for combining sub-metrics)
    position_weight: float = 1.0           # Weight for position error in translational group
    velocity_weight: float = 1.0           # Weight for velocity error in translational group
    attitude_weight: float = 1.0           # Weight for attitude error in rotational group
    angular_velocity_weight: float = 1.0   # Weight for angular velocity error in rotational group
    trans_effort_weight: float = 1.0       # Weight for translational effort in control group
    rot_effort_weight: float = 1.0         # Weight for rotational effort in control group


@dataclass(frozen=True)
class MetricsConfig:
    """
    Configuration for metrics used in GA tuning.

    Attributes:
        multi_objective: Optimization mode
            - True or "full": 3 objectives per loop (inner: 3, outer: 3)
            - "grouped": 3 composite objectives combining inner+outer metrics
            - False: Single-objective weighted sum (returns scalar fitness)
        normalize_metrics: Whether to normalize metrics
        inner_loop_weights: Weights for inner loop (used when evaluator_type="inner" and multi_objective=False)
        outer_loop_weights: Weights for outer loop (used when evaluator_type="outer" and multi_objective=False)
        grouped_weights: Weights for grouped mode (used when multi_objective="grouped")
    """
    multi_objective: Union[bool, Literal["full", "grouped"]] = "grouped"  # True/"full": Pareto front, "grouped": 3 objectives, False: weighted sum
    normalize_metrics: bool = True  # Normalize metrics
    inner_loop_weights: InnerLoopMetricWeights = field(default_factory=InnerLoopMetricWeights)
    outer_loop_weights: OuterLoopMetricWeights = field(default_factory=OuterLoopMetricWeights)
    grouped_weights: GroupedMetricWeights = field(default_factory=GroupedMetricWeights)
    
    @property
    def is_multi_objective(self) -> bool:
        """Check if running in any multi-objective mode."""
        return self.multi_objective is True or self.multi_objective in ("full", "grouped")
    
    @property
    def is_grouped_mode(self) -> bool:
        """Check if running in grouped (3-objective) mode."""
        return self.multi_objective == "grouped"
    
    @property
    def objective_count(self) -> int:
        """Get number of objectives based on mode."""
        if self.multi_objective == "grouped":
            return 3  # translational_error, rotational_error, control_effort
        elif self.multi_objective is True or self.multi_objective == "full":
            return 3  # Per loop: 3 objectives
        else:
            return 1  # Single objective
