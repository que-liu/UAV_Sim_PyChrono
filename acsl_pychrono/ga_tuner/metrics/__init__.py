"""
MRAC Metrics Package

Centralized metrics calculation for MRAC controller performance evaluation.
Provides both inner loop (attitude/rotational) and outer loop (position/translational) metrics.

Inner Loop Metrics (attitude/rotational control):
- attitude_tracking_error: RMS error in roll/pitch/yaw tracking
- angular_velocity_tracking_error: RMS error in angular rate tracking
- moment_effort: RMS control effort (moments u2, u3, u4)

Outer Loop Metrics (position/translational control):
- position_error: RMS position tracking error
- velocity_error: RMS velocity tracking error
- control_effort: RMS control effort (total thrust from all motors)
"""

from .mrac_inner_loop_metrics import (
    MRACInnerLoopMetrics,
    MRACInnerLoopMetricsCalculator
)

from .mrac_outer_loop_metrics import (
    MRACOuterLoopMetrics,
    MRACOuterLoopMetricsCalculator,
    load_simulation_log
)

from .normalizer import MetricNormalizer

from .uav_evaluators import (
    UAVSimulationEvaluator,
    PIDSimulationEvaluator,
    MRACInnerLoopEvaluator,
    MRACOuterLoopEvaluator,
    MRACGroupedEvaluator,
)

__all__ = [
    # Inner loop
    'MRACInnerLoopMetrics',
    'MRACInnerLoopMetricsCalculator',
    
    # Outer loop
    'MRACOuterLoopMetrics',
    'MRACOuterLoopMetricsCalculator',
    
    # Normalizer
    'MetricNormalizer',
    
    # Evaluators
    'UAVSimulationEvaluator',
    'PIDSimulationEvaluator',
    'MRACInnerLoopEvaluator',
    'MRACOuterLoopEvaluator',
    'MRACGroupedEvaluator',
    
    # Utility
    'load_simulation_log'
]
