"""
Genetic Algorithm Tuning Package for UAV Control Systems

This package provides a modular framework for tuning control parameters
using various genetic algorithms and optimization techniques.
"""

from .algorithms.base_ga import BaseGATuner
from .algorithms.deap_ga import DEAPGATuner
try:
    from .algorithms.pymoo_ga import PymooGATuner
except ImportError:
    PymooGATuner = None
from .core.fitness_evaluator import FitnessEvaluator
from .core.parameter_bounds import ParameterBounds
from .core.optimization_result import OptimizationResult
from .uav_integration import UAVModelAdapter, create_uav_ga_tuner
from .uav_evaluators import UAVSimulationEvaluator, UAVFitnessEvaluator, MRACSimulationEvaluator

# Import specific parameter bounds classes from core
try:
    from .core.parameter_bounds import PIDParameterBounds
except ImportError:
    PIDParameterBounds = None
    MRACParameterBounds = None

__all__ = [
    'BaseGATuner',
    'DEAPGATuner', 
    'PymooGATuner',
    'FitnessEvaluator',
    'ParameterBounds',
    'PIDParameterBounds',
 
    'OptimizationResult',
    'UAVModelAdapter',
    'create_uav_ga_tuner',
    'UAVSimulationEvaluator',
    'UAVFitnessEvaluator',
    'MRACSimulationEvaluator'
]
