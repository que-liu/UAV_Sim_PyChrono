"""
Core module for running optimization processes.
"""

import os
import pickle
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
from abc import ABC, abstractmethod

from .tuning_factory import ControllerTuningFactory
from ..algorithms.deap_ga import DEAPGATuner
from .fitness_evaluator import FitnessEvaluator

class OptimizationRunner(ABC):
    """Base class for running optimization processes."""
    
    def __init__(self,
                 config: Dict[str, Any],
                 controller_type: str,
                 output_base_dir: str = "optimization_results"):
        """
        Initialize optimization runner.
        
        Args:
            config: Optimization configuration
            controller_type: Type of controller to optimize
            output_base_dir: Base directory for output
        """
        self.config = config
        self.controller_type = controller_type
        self.output_base_dir = output_base_dir
        self.tuner = ControllerTuningFactory.create_tuner(controller_type)
    
    @abstractmethod
    def create_evaluator(self, config: Dict[str, Any]) -> FitnessEvaluator:
        """Create fitness evaluator instance."""
        pass
    
    def setup_output_directory(self) -> str:
        """Create and return output directory path."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join(
            self.output_base_dir,
            f"{self.controller_type.lower()}_tuning_{timestamp}"
        )
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "intermediate"), exist_ok=True)
        return output_dir
    
    def save_results(self, 
                    output_dir: str,
                    best_gains: Dict[str, Any],
                    result: Any) -> None:
        """
        Save optimization results.
        
        Args:
            output_dir: Directory to save results
            best_gains: Best gains found
            result: Optimization result object
        """
        # Save gains
        gains_file = os.path.join(output_dir, "best_gains.pkl")
        with open(gains_file, "wb") as f:
            pickle.dump(best_gains, f)
        
        # Save full results
        results_file = os.path.join(output_dir, "optimization_result.pkl")
        with open(results_file, "wb") as f:
            pickle.dump(result, f)
        
        # Save human-readable summary
        summary_file = os.path.join(output_dir, "optimization_summary.txt")
        with open(summary_file, "w") as f:
            f.write(f"Optimization Results for {self.controller_type}\n")
            f.write("=" * 50 + "\n\n")
            f.write("Best Gains:\n")
            for name, value in best_gains.items():
                f.write(f"{name}: {value}\n")
            f.write("\nOptimization Stats:\n")
            if hasattr(result, "get_stats"):
                stats = result.get_stats()
                for key, value in stats.items():
                    f.write(f"{key}: {value}\n")
    
    def print_results(self, 
                     best_gains: Dict[str, Any],
                     parameter_names: List[str]) -> None:
        """
        Print optimization results.
        
        Args:
            best_gains: Best gains found
            parameter_names: Names of parameters
        """
        print("\nOptimization Results:")
        print("=" * 50)
        print("\nBest Parameter Values:")
        for name in parameter_names:
            print(f"{name}: {best_gains[name]}")
    
    def run(self) -> Tuple[Dict[str, Any], Any]:
        """
        Run optimization process.
        
        Returns:
            Tuple of (best gains, optimization result)
        """
        # Setup
        output_dir = self.setup_output_directory()
        evaluator = self.create_evaluator(self.config)
        
        # Create and run optimizer
        ga_config = self.config.get('ga_config', {})
        
        # Debug: Check parameter bounds
        parameter_bounds = self.tuner.get_parameter_bounds()
        print(f"[DEBUG] Parameter bounds from tuner: {parameter_bounds.n_parameters} parameters")
        print(f"[DEBUG] Parameter names: {parameter_bounds.parameter_names[:5]}...")
        
        optimizer = DEAPGATuner(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=evaluator,
            population_size=ga_config.get('population_size', 50),
            n_generations=ga_config.get('n_generations', 50),
            crossover_prob=ga_config.get('crossover_rate', 0.8),
            mutation_prob=ga_config.get('mutation_rate', 0.3)
        )
        
        # Run optimization
        print(f"\nStarting {self.controller_type} parameter tuning...")
        result = optimizer.optimize(
            verbose=True,
            save_intermediate=True,
            intermediate_save_path=os.path.join(output_dir, "intermediate")
        )
        
        # Process results
        best_params = result.get_best_individual()
        if best_params is None:
            raise RuntimeError("Optimization finished without producing a best individual.")

        # Ensure we always work with a plain list for downstream consumers
        if hasattr(best_params, "tolist"):
            best_params = best_params.tolist()

        best_gains = dict(zip(self.tuner.get_parameter_names(), best_params))
        
        # Save and display results
        self.save_results(output_dir, best_gains, result)
        self.print_results(best_gains, self.tuner.get_parameter_names())
        
        return best_gains, result
