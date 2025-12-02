"""
Integration module for connecting the GA tuner with the UAV simulation model.
"""

from pathlib import Path
from typing import Dict, Any, Optional, List, Union
from copy import deepcopy

from acsl_pychrono.simulation.simulation import Simulation
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.config.config import MissionConfig, WrapperParams, SimulationConfig

from .uav_evaluators import PIDSimulationEvaluator, MRACSimulationEvaluator
from .controllers.pid_tuning import PIDTuning
from .controllers.mrac_tuning import MRACTuning
from .algorithms.deap_ga import DEAPGATuner
from .algorithms.pymoo_ga import PymooGATuner
from .ga_config import GAConfig

class UAVModelAdapter:
    """
    Adapter class that connects the GA tuner with the UAV simulation model.
    """
    
    def __init__(self, 
                 base_config=None,  # Optional[Dict[str, Any]]
                 log_base_dir="simulation_logs"):  # str
        """
        Initialize the UAV model adapter.
        
        Args:
            base_config: Base configuration for simulations
            log_base_dir: Base directory for simulation logs
        """
        
        base = base_config or {}
        self.base_config = {
            'mission': base.get('mission', MissionConfig()),
            'wrapper_params': base.get('wrapper_params', WrapperParams()),
        }
        self.log_base_dir = Path(log_base_dir)
        self.log_base_dir.mkdir(parents=True, exist_ok=True)
    
    
    def create_simulation_config(self,
                               parameters: List[float],
                               log_dir: str,
                               controller_type: str):
        """
        Create simulation configuration with given parameters and controller type.
        """

        Path(log_dir).mkdir(parents=True, exist_ok=True)

        mission_cfg = self._clone_mission_config()
        wrapper_params = self._build_wrapper_params(controller_type, parameters)

        mission_cfg.controller_type = controller_type
        mission_cfg.wrapper_batch_dir = log_dir
        mission_cfg.visualization_flag = False
        mission_cfg.wrapper_flag = True

        sim_config = SimulationConfig(
            mission_config=mission_cfg,
            wrapper_params=wrapper_params
        )
        return {
            'simulation_config': sim_config,
            'log_directory': log_dir,
            'parameters': parameters,
            'controller_type': controller_type
        }
    
    def _clone_mission_config(self):
        """Clone the base MissionConfig to avoid cross-run mutation."""
        return deepcopy(self.base_config['mission'])

    def _build_wrapper_params(self, controller_type: str, parameters: List[float]):
        """Create wrapper params with GA tuner flags and external parameters."""
        wrapper_params = deepcopy(self.base_config['wrapper_params'])
        wrapper_params.use_ga_tuner = True
        controller_params = list(parameters)

        if controller_type == 'MRAC':
            wrapper_params.external_controller_params = {'mrac_params': controller_params}
        elif controller_type.upper() == 'PID':
            wrapper_params.external_controller_params = {'pid_params': controller_params}
        else:
            raise ValueError(f"Unsupported controller type for GA tuning: {controller_type}")

        return wrapper_params
    
    def run_simulation(self, config):  # Dict[str, Any] -> Dict[str, Any] or None
        """
        Run UAV simulation with given configuration.
        
        Args:
            config: Simulation configuration
            
        Returns:
            Log data dictionary if simulation completed successfully, None otherwise
        """
        sim_config = config['simulation_config']
        simulation = Simulation(sim_config)
        git_info = Logging.getGitRepoInfo()
        log_dict = simulateMission(simulation, git_info)
        if log_dict is None:
            print("Warning: simulateMission finished without producing log data.")
        return log_dict
    
    def get_simulation_status(self):  # -> Dict[str, Any]
        """Get status of UAV simulation system."""
        return {
            'base_config': self.base_config,
            'log_base_dir': self.log_base_dir
        }


def create_uav_ga_tuner(controller_type: Optional[str] = None,
                       algorithm: Optional[str] = None,
                       uav_adapter=None,
                       parameter_bounds=None,
                       ga_config: Optional[GAConfig] = None,
                       **kwargs):
    """
    Factory function to create a GA tuner configured for UAV simulations.
    
    Args:
        controller_type: Optional controller override ('PID' or 'MRAC')
        algorithm: Optional GA backend override ('DEAP' or 'PYMOO')
        uav_adapter: UAV adapter instance (created if None)
        parameter_bounds: Parameter bounds (created based on controller_type if None)
        ga_config: GAConfig dataclass providing controller/algorithm and GA hyperparameters
        **kwargs: Additional arguments for the tuner (override ga_config values)
        
    Returns:
        Configured GA tuner
    """
    controller_type = ga_config.controller_type
    algorithm = ga_config.algorithm

    defaults = {
            'population_size': ga_config.population_size,
            'n_generations': ga_config.num_generations,
            'crossover_prob': ga_config.crossover_rate,
            'mutation_prob': ga_config.mutation_rate,
            'selection_method': ga_config.selection_method,
            'tournament_size': ga_config.tournament_size,
            'random_seed': ga_config.random_seed,
            'multi_objective': ga_config.multi_objective,
        }
    for key, value in defaults.items():
        if value is not None and key not in kwargs:
            kwargs[key] = value

    if ga_config.algorithm.upper() == 'PYMOO':
        kwargs.setdefault('algorithm', ga_config.pymoo_variant)
        if ga_config.pymoo_algorithm_params:
            kwargs.setdefault('algorithm_params', dict(ga_config.pymoo_algorithm_params))

    controller = controller_type.upper()
    algorithm_name = algorithm.upper()

    # Create UAV adapter if not provided
    if uav_adapter is None:
        uav_adapter = UAVModelAdapter()

    # Create parameter bounds if not provided
    pid_tuning = None
    if parameter_bounds is None:
        if controller == 'PID':
            pid_tuning = PIDTuning()
            parameter_bounds = pid_tuning.get_parameter_bounds()
        elif controller == 'MRAC':
            mrac_tuning = MRACTuning()
            parameter_bounds = mrac_tuning.get_parameter_bounds()
        else:
            raise ValueError(f"Unknown controller_type: {controller}. Use 'PID' or 'MRAC'")
    
    # Create appropriate fitness evaluator based on controller type
    if controller == 'PID':
        fitness_evaluator = PIDSimulationEvaluator(
            uav_adapter=uav_adapter,
            log_directory="simulation_logs/ga_optimization",
            cost_function_weights={'total': 1.0},
            parallel_config=kwargs.get('parallel_config'),
            multi_objective=kwargs.get('multi_objective', False)
        )
    elif controller == 'MRAC':
        fitness_evaluator = MRACSimulationEvaluator(
            uav_adapter=uav_adapter,
            log_directory="simulation_logs/ga_optimization",
            parallel_config=kwargs.get('parallel_config')
        )
    else:
        raise ValueError(f"Unknown controller_type: {controller}. Use 'PID' or 'MRAC'")
    
    # For non-PYMOO algorithms, remove PYMOO-specific kwargs
    if algorithm_name != 'PYMOO':
        kwargs.pop('algorithm', None)
        kwargs.pop('algorithm_params', None)

    # Filter out parameters that tuners don't accept
    tuner_kwargs = {k: v for k, v in kwargs.items() if k not in ('controller_type', 'ga_config')}
    
    # Create appropriate tuner
    if algorithm_name == 'DEAP':
        return DEAPGATuner(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    elif algorithm_name == 'PYMOO':
        return PymooGATuner(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}. Use 'DEAP' or 'Pymoo'")
