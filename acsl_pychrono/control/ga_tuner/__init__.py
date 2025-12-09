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
from .uav_integration import UAVModelAdapter, create_uav_ga_tuner, summarize_tuner_result
from .ga_config import GAConfig
from .uav_evaluators import UAVSimulationEvaluator, PIDSimulationEvaluator, MRACInnerLoopEvaluator, MRACOuterLoopEvaluator


__all__ = [
    'BaseGATuner',
    'DEAPGATuner', 
    'PymooGATuner',
    'FitnessEvaluator',
    'ParameterBounds',
    'OptimizationResult',
    'UAVModelAdapter',
    'create_uav_ga_tuner',
    'summarize_tuner_result',
    'UAVSimulationEvaluator',
    'PIDSimulationEvaluator',
    'MRACInnerLoopEvaluator',
    'MRACOuterLoopEvaluator',
    'GAConfig'
]
