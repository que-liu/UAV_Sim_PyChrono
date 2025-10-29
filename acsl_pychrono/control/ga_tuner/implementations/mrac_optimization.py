"""
MRAC optimization implementation.
"""

# currently not used
from typing import Dict, Any
from ..core.optimization_runner import OptimizationRunner
from ..uav_evaluators import MRACSimulationEvaluator
from ..uav_integration import UAVModelAdapter

class MRACOptimizationRunner(OptimizationRunner):
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize MRAC optimization runner.
        
        Args:
            config: Optimization configuration
        """
        super().__init__(config, "MRAC")
        self.uav_adapter = UAVModelAdapter()
    
    def create_evaluator(self, config: Dict[str, Any]) -> MRACSimulationEvaluator:
        """Create MRAC-specific evaluator."""
        return MRACSimulationEvaluator(
            self.uav_adapter,
            cost_function_weights=config.get('metric_weights'),
            log_directory=config.get('log_directory', 'simulation_logs'),
            parallel_config=config.get('parallel')
        )
