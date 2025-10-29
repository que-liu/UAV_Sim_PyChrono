"""
Abstract base class for controller parameter tuning.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Union, Optional
import numpy as np

from .parameter_bounds import ParameterBounds

class ControllerTuningInterface(ABC):

    @abstractmethod
    def get_parameter_bounds(self) -> ParameterBounds:
        """
        Get bounds for all tunable parameters.
        
        Returns:
            Dictionary with parameter names as keys and [min, max] bounds as values
        """
        pass
    
    @abstractmethod
    def get_parameter_names(self) -> List[str]:
        """
        Get names of all tunable parameters.
        
        Returns:
            List of parameter names
        """
        pass
    
    @abstractmethod
    def vector_to_gains(self, parameter_vector: List[float]) -> Dict[str, Any]:
        """
        Convert parameter vector to controller gain structure.
        
        Args:
            parameter_vector: List of parameter values in defined order
            
        Returns:
            Dictionary containing controller gain structure
        """
        pass
    
    @abstractmethod
    def gains_to_vector(self, gains: Dict[str, Any]) -> List[float]:
        """
        Convert controller gain structure to parameter vector.
        
        Args:
            gains: Dictionary containing controller gain structure
            
        Returns:
            List of parameter values in defined order
        """
        pass
    
    @abstractmethod
    def get_default_gains(self) -> Dict[str, Any]:
        """
        Get default controller gains.
        
        Returns:
            Dictionary containing default gain structure
        """
        pass
    
    @abstractmethod
    def get_cost_functions(self) -> Dict[str, Any]:
        """
        Get cost functions for evaluating controller performance.
        
        Returns:
            Dictionary of cost function names and their weights
        """
        pass
    
    @abstractmethod
    def evaluate_performance(self, log_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Evaluate controller performance from simulation log data.
        
        Args:
            log_data: Dictionary containing simulation log data
            
        Returns:
            Dictionary of performance metrics
        """
        pass
    
    def validate_parameters(self, parameter_vector: List[float]) -> bool:
        """
        Validate parameter vector against bounds and constraints.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Whether parameters are valid
        """
        bounds = self.get_parameter_bounds()
        names = self.get_parameter_names()
        
        if len(parameter_vector) != len(names):
            return False
        
        for value, name in zip(parameter_vector, names):
            min_val, max_val = bounds[name]
            if value < min_val or value > max_val:
                return False
        
        return True
    
    def clip_parameters(self, parameter_vector: List[float]) -> List[float]:
        """
        Clip parameter values to be within bounds.
        
        Args:
            parameter_vector: List of parameter values
            
        Returns:
            Clipped parameter vector
        """
        bounds = self.get_parameter_bounds()
        names = self.get_parameter_names()
        
        clipped = []
        for value, name in zip(parameter_vector, names):
            min_val, max_val = bounds[name]
            clipped.append(np.clip(value, min_val, max_val))
        
        return clipped
    
    def get_parameter_groups(self) -> Dict[str, List[str]]:
        """
        Get parameter grouping information.
        
        Returns:
            Dictionary mapping group names to lists of parameter names
        """
        return {'all': self.get_parameter_names()}
