"""
Factory for creating controller tuning interfaces.
"""

from typing import Dict, Any, Optional, Type, List
from .controller_tuning import ControllerTuningInterface
from ..controllers.pid_tuning import PIDTuning
from ..controllers.mrac_tuning import MRACTuning

class ControllerTuningFactory:    
    # Registry of available tuning implementations
    TUNING_REGISTRY = {
        'PID': PIDTuning,
        'MRAC': MRACTuning
    }
    
    @classmethod
    def register_tuner(cls, 
                      controller_type: str,
                      tuner_class: Type[ControllerTuningInterface]):
        """
        Register a new tuning implementation.
        
        Args:
            controller_type: Type identifier for the controller
            tuner_class: Class implementing ControllerTuningInterface
        """
        if not issubclass(tuner_class, ControllerTuningInterface):
            raise ValueError(f"Tuner class must implement ControllerTuningInterface")
        
        cls.TUNING_REGISTRY[controller_type.upper()] = tuner_class
    
    @classmethod
    def create_tuner(cls,
                    controller_type: str,
                    tuning_config: Optional[Dict[str, Any]] = None) -> ControllerTuningInterface:
        """
        Create a tuning interface for the specified controller type.
        
        Args:
            controller_type: Type of controller to tune
            tuning_config: Optional configuration for tuning
            
        Returns:
            Configured tuning interface
            
        Raises:
            ValueError: If controller type is not supported
        """
        tuner_class = cls.TUNING_REGISTRY.get(controller_type.upper())
        if tuner_class is None:
            raise ValueError(
                f"Unsupported controller type: {controller_type}. "
                f"Available types: {list(cls.TUNING_REGISTRY.keys())}"
            )
        
        return tuner_class(tuning_config)
    
    @classmethod
    def get_available_controllers(cls) -> List[str]:
        """
        Get list of supported controller types.
        
        Returns:
            List of controller type identifiers
        """
        return list(cls.TUNING_REGISTRY.keys())
    
    @classmethod
    def get_tuner_info(cls, controller_type: str) -> Dict[str, Any]:
        """
        Get information about a tuning implementation.
        
        Args:
            controller_type: Controller type identifier
            
        Returns:
            Dictionary containing tuner information
        """
        tuner_class = cls.TUNING_REGISTRY.get(controller_type.upper())
        if tuner_class is None:
            raise ValueError(f"Unknown controller type: {controller_type}")
        
        # Create temporary instance to get info
        tuner = tuner_class()
        
        return {
            'parameter_names': tuner.get_parameter_names(),
            'parameter_groups': tuner.get_parameter_groups(),
            'parameter_bounds': tuner.get_parameter_bounds(),
            'cost_functions': tuner.get_cost_functions()
        }
