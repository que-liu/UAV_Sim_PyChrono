"""
Integration module for connecting the GA tuner with the UAV simulation model.
"""

import os
import sys
import numpy as np
from typing import Dict, Any, Optional, List, Union
import pickle
import shutil
from datetime import datetime

# UAV simulation modules will be imported dynamically when needed to avoid import errors at module load time

from .uav_evaluators import UAVSimulationEvaluator

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
        self.base_config = base_config or self._get_default_config()
        self.log_base_dir = log_base_dir
        os.makedirs(log_base_dir, exist_ok=True)
        
        # Check if UAV modules are available
        self.uav_available = self._check_uav_availability()
    
    def _check_uav_availability(self) -> bool:
        """Check if UAV simulation modules are available."""
        try:
            # Try to import the modules dynamically
            from acsl_pychrono.executor.simulate_mission import simulateMission
            from acsl_pychrono.simulation.simulation import Simulation
            from acsl_pychrono.config.config import SimulationConfig, MissionConfig, VehicleConfig, EnvironmentConfig, WrapperParams
            from acsl_pychrono.control.logging import Logging
            return True
        except ImportError:
            return False
    
    def _get_default_config(self) -> Dict[str, Any]:
        """Get default UAV simulation configuration."""
        return {
            'mission': {
                'trajectory_type': 'hover_trajectory', 
                'duration': 10.0,
                'visualization': False,
                'wrapper_flag': True,
                'add_payload_flag': True,  # Enable payload for MRAC simulation
                'payload_type': 'two_steel_balls' 
            },
            'vehicle': {
                'vehicle_type': 'x8copter',
                'mass': 2.5,
                'inertia': [0.1, 0.1, 0.2]
            },
            'environment': {
                'environment_type': 'environmentA',
                'wind_speed': 0.0,
                'turbulence': False
            },
            'control': {
                'controller_type': 'PID',
                'control_frequency': 100.0
            }
        }
    
    def create_simulation_config(self, 
                               parameters,  # List[float]
                               log_dir,      # str
                               controller_type='PID'):  # str
        """
        Create simulation configuration with given parameters and controller type.
        """
        if not self.uav_available:
            return self._create_test_config(parameters, log_dir)
        try:
            from acsl_pychrono.config.config import MissionConfig, VehicleConfig, EnvironmentConfig, WrapperParams, SimulationConfig
            mission_cfg = MissionConfig()
            vehicle_cfg = VehicleConfig()
            environment_cfg = EnvironmentConfig()
            
            # Apply payload settings from base_config
            if 'mission' in self.base_config:
                mission_base = self.base_config['mission']
                if 'add_payload_flag' in mission_base:
                    mission_cfg.add_payload_flag = mission_base['add_payload_flag']
                if 'payload_type' in mission_base:
                    mission_cfg.payload_type = mission_base['payload_type']
                if 'duration' in mission_base:
                    mission_cfg.duration = mission_base['duration']
                if 'trajectory_type' in mission_base:
                    mission_cfg.trajectory_type = mission_base['trajectory_type']
            
            # Attach GA parameters into wrapper params so control layer can consume them
            wrapper_params = WrapperParams()
            if controller_type in {'MRAC', 'TwoLayerMRAC'}:
                try:
                    # Use new GA tuner structure
                    wrapper_params.use_ga_tuner = True
                    wrapper_params.external_controller_params = {
                        'mrac_params': list(parameters)
                    }
                except Exception:
                    pass
            elif controller_type == 'PID':
                try:
                    # Use new GA tuner structure
                    wrapper_params.use_ga_tuner = True
                    wrapper_params.external_controller_params = {
                        'pid_gains': list(parameters)
                    }
                except Exception as e:
                    pass
            else:
                raise ValueError(f"Unsupported controller type for GA tuning: {controller_type}")

            mission_cfg.controller_type = controller_type
            mission_cfg.wrapper_batch_dir = log_dir
            mission_cfg.visualization_flag = False
            mission_cfg.wrapper_flag = True
            sim_config = SimulationConfig(
                mission_config=mission_cfg,
                vehicle_config=vehicle_cfg,
                environment_config=environment_cfg,
                wrapper_params=wrapper_params
            )
            return {
                'simulation_config': sim_config,
                'log_directory': log_dir,
                'parameters': parameters,
                'controller_type': controller_type
            }
        except Exception as e:
            print(f"Error creating simulation config: {e}")
            return self._create_test_config(parameters, log_dir)
    
    def _create_test_config(self, parameters, log_dir):  # parameters: List[float], log_dir: str -> Dict[str, Any]
        """Create test configuration when UAV modules are not available."""
        return {
            'test_mode': True,
            'parameters': parameters,
            'log_directory': log_dir,
            'simulation_type': 'test'
        }
    
    def run_simulation(self, config):  # Dict[str, Any] -> Dict[str, Any] or None
        """
        Run UAV simulation with given configuration.
        
        Args:
            config: Simulation configuration
            
        Returns:
            Log data dictionary if simulation completed successfully, None otherwise
        """
        if not self.uav_available or config.get('test_mode', False):
            return self._run_test_simulation(config)
        
        try:
            # Import modules dynamically
            from acsl_pychrono.simulation.simulation import Simulation
            from acsl_pychrono.control.logging import Logging
            from acsl_pychrono.executor.simulate_mission import simulateMission
            from acsl_pychrono.control.ga_tuner.controller_factory import instantiateControllerWithGA
            
            # Extract simulation config
            sim_config = config['simulation_config']
            log_dir = config['log_directory']
            
            # Create simulation object
            simulation = Simulation(sim_config)
            
            # Get git info for logging
            git_info = Logging.getGitRepoInfo()
            
            try:
                log_dict = self._simulateMissionWithGA(simulation, git_info)
                if log_dict is None:
                    return None
                return log_dict
            except Exception as e:
                import traceback
                print(f"GA simulateMission raised: {e}")
                traceback.print_exc()
                return None
                
        except Exception as e:
            import traceback
            print(f"Error running simulation: {e}")
            traceback.print_exc()
            return False
    
    def _run_test_simulation(self, config):  # Dict[str, Any] -> bool
        """Run test simulation when UAV modules are not available."""
        log_dir = config['log_directory']
        parameters = config['parameters']
        
        # Create test log data
        test_log = self._generate_test_log(parameters)
        
        # Save test log
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, "log_dict.pkl")
        
        with open(log_path, 'wb') as f:
            pickle.dump(test_log, f)
        
        return test_log
    
    def _simulateMissionWithGA(self, sim, git_info):
        """
        GA-enabled version of simulateMission that uses GA controller factory.
        This is a copy of the original simulateMission logic but with GA controller factory.
        """
        try:
            # Import required modules
            from acsl_pychrono.simulation.flight_params import FlightParams
            from acsl_pychrono.simulation.ode_input import OdeInput
            import acsl_pychrono.user_defined_trajectory as Traj
            
            # Instantiation of classes
            flight_params = FlightParams()
            ode_input = OdeInput()
            
            user_defined_trajectory = Traj.instantiateTrajectory(
                sim.mission_config,
                flight_params,
                sim.mfloor,
                sim.mfloor_Yposition
            )

            # Instantiation of controller, gains, and logger using GA factory
            from acsl_pychrono.control.ga_tuner.controller_factory import instantiateControllerWithGA
            (gains, controller, logger) = instantiateControllerWithGA(
                sim.mission_config.controller_type,
                ode_input,
                flight_params,
                sim.mission_config.timestep,
                sim.simulation_config.wrapper_params
            )

            sim.assignInstances(
                flight_params,
                ode_input,
                user_defined_trajectory,
                gains,
                controller,
                logger
            )

            # Run simulation
            sim.runSimulationLoop()

            # Save logs
            log_dict = logger.toDictionary()
            
            if log_dict is None:
                return None
            return log_dict
            
        except Exception as e:
            import traceback
            print(f"Error in _simulateMissionWithGA: {e}")
            traceback.print_exc()
            return None
    
    def _generate_test_log(self, parameters):  # List[float] -> Dict[str, Any]
        """Generate test log data for testing purposes."""
        # Create synthetic simulation data
        time_steps = 100
        dt = 0.01
        
        # Generate time series
        time_series = np.linspace(0, time_steps * dt, time_steps)
        
        # Generate desired trajectory (simple hover)
        pos_des = np.zeros((time_steps, 3))
        pos_des[:, 2] = 2.0  # Hover at 2m height
        
        vel_des = np.zeros((time_steps, 3))
        
        # Generate actual trajectory with some error based on parameters
        # Simulate parameter influence on performance
        param_quality = np.mean(parameters) / 100.0  # Normalize parameters
        
        # Position error decreases with better parameters
        pos_error_scale = max(0.1, 1.0 - param_quality)
        pos_act = pos_des + np.random.normal(0, pos_error_scale, (time_steps, 3))
        
        # Velocity error
        vel_act = vel_des + np.random.normal(0, 0.1 * pos_error_scale, (time_steps, 3))
        
        # Thrust data (normalized)
        thrust_base = 24.5  # mg
        thrust_variation = 0.1 * pos_error_scale
        thrust_data = {}
        for i in range(8):
            thrust_data[f"T{i+1}"] = thrust_base + np.random.normal(0, thrust_variation, time_steps)
        
        # Create log structure
        log = {
            "time": time_series.tolist(),
            "position": {
                "x": pos_act[:, 0].tolist(),
                "y": pos_act[:, 1].tolist(),
                "z": pos_act[:, 2].tolist()
            },
            "velocity": {
                "x": vel_act[:, 0].tolist(),
                "y": vel_act[:, 1].tolist(),
                "z": vel_act[:, 2].tolist()
            },
            "user_defined_position": {
                "x": pos_des[:, 0].tolist(),
                "y": pos_des[:, 1].tolist(),
                "z": pos_des[:, 2].tolist()
            },
            "user_defined_velocity": {
                "x": vel_des[:, 0].tolist(),
                "y": vel_des[:, 1].tolist(),
                "z": vel_des[:, 2].tolist()
            },
            "thrust_motors_N": thrust_data,
            "parameters_used": parameters,
            "test_mode": True
        }
        
        return log
    
    def get_simulation_status(self):  # -> Dict[str, Any]
        """Get status of UAV simulation system."""
        return {
            'uav_modules_available': self.uav_available,
            'base_config': self.base_config,
            'log_base_dir': self.log_base_dir
        }


class UAVFitnessEvaluator(UAVSimulationEvaluator):
    """
    Enhanced fitness evaluator specifically for UAV simulations.
    """
    
    def __init__(self, 
                 uav_adapter,  # UAVModelAdapter
                 cost_function_weights=None,  # Optional[Dict[str, float]]
                 **kwargs):
        """
        Initialize UAV fitness evaluator.
        
        Args:
            uav_adapter: UAV model adapter
            cost_function_weights: Weights for cost functions
            **kwargs: Additional arguments for parent class
        """
        # Extract multi_objective parameter before passing to parent
        self.multi_objective = kwargs.pop('multi_objective', False)
        
        # Create a basic simulation config for the parent class
        simulation_config = uav_adapter.base_config
        
        super().__init__(simulation_config, **kwargs)
        
        self.uav_adapter = uav_adapter
    
    def _get_controller_type(self) -> str:
        """Get the controller type for this evaluator."""
        return 'PID'  # Default to PID
    
    def _compute_fitness(self, config: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness value from simulation configuration and results."""
        try:
            # Check if we have log data directly in config (from simulation)
            log_data = config.get('log_data')
            
            if log_data is not None:
                # Use log data directly from simulation
                result = self._compute_fitness_from_log_data(log_data, parameters)
                if result is None:
                    if self.multi_objective:
                        return [float('inf'), float('inf'), float('inf')]
                    else:
                        return float('inf')
                return result
            else:
                # Simulation failed, no log data
                if self.multi_objective:
                    return [float('inf'), float('inf'), float('inf')]
                else:
                    return float('inf')
            
            # Fallback: try to load from file
            log_path = config.get('log_path')
            
            if not log_path or not os.path.exists(log_path):
                if self.multi_objective:
                    return [float('inf'), float('inf'), float('inf')]
                else:
                    return float('inf')  # Worst possible fitness
            
            # Load simulation log
            with open(log_path, 'rb') as f:
                log = pickle.load(f)
            
            result = self._compute_fitness_from_log_data(log, parameters)
            return result
            
        except Exception as e:
            print(f"Error computing fitness: {e}")
            import traceback
            traceback.print_exc()
            if self.multi_objective:
                return [float('inf'), float('inf'), float('inf')]
            else:
                return float('inf')
        
        if self.multi_objective:
            return [float('inf'), float('inf'), float('inf')]
        else:
            return float('inf')  # Fallback
    
    def _create_simulation_config(self, 
                                parameters,  # Union[List[float], np.ndarray]
                                log_dir):  # str -> Dict[str, Any]
        """Create simulation configuration using UAV adapter."""
        return self.uav_adapter.create_simulation_config(parameters, log_dir)
    
    def _execute_simulation(self, sim_config, log_dir):  # sim_config: Dict[str, Any], log_dir: str
        """Execute simulation using UAV adapter."""
        return self.uav_adapter.run_simulation(sim_config)
    
    def _compute_cost_functions(self, log):  # Dict[str, Any] -> float
        """
        Compute single-objective cost function for PID tuning.
        
        Args:
            log: Simulation log data
            
        Returns:
            Single scalar cost value
        """
        try:
            # Use parent method to get individual cost components
            parent_result = super()._compute_cost_functions(log)
            
            # If parent returns a list (multi-objective), sum them
            if isinstance(parent_result, (list, tuple)):
                return float(sum(parent_result))
            else:
                return float(parent_result)
        except Exception as e:
            return float(999.0)
    
    def _compute_fitness_from_log_data(self, log_data, parameters):
        """Compute fitness from log data dictionary."""
        try:
            import numpy as np
            
            # Extract position and velocity data
            pos_des = np.stack([
                np.array(log_data["user_defined_position"]["x"]).flatten(),
                np.array(log_data["user_defined_position"]["y"]).flatten(),
                np.array(log_data["user_defined_position"]["z"]).flatten()
            ], axis=1)
            pos_act = np.stack([
                np.array(log_data["position"]["x"]).flatten(),
                np.array(log_data["position"]["y"]).flatten(),
                np.array(log_data["position"]["z"]).flatten()
            ], axis=1)
            
            vel_des = np.stack([
                np.array(log_data["user_defined_velocity"]["x"]).flatten(),
                np.array(log_data["user_defined_velocity"]["y"]).flatten(),
                np.array(log_data["user_defined_velocity"]["z"]).flatten()
            ], axis=1)
            vel_act = np.stack([
                np.array(log_data["velocity"]["x"]).flatten(),
                np.array(log_data["velocity"]["y"]).flatten(),
                np.array(log_data["velocity"]["z"]).flatten()
            ], axis=1)
            
            # Compute errors
            pos_error = np.linalg.norm(pos_des - pos_act, axis=1)
            vel_error = np.linalg.norm(vel_des - vel_act, axis=1)
            
            # Compute fitness components
            pos_rmse = np.sqrt(np.mean(pos_error**2))
            vel_rmse = np.sqrt(np.mean(vel_error**2))
            
            # Total fitness (lower is better)
            total_fitness = pos_rmse + vel_rmse
            
            print(f"[FITNESS] Parameters {parameters[:3]} -> Fitness: {total_fitness:.6f}")
            return float(total_fitness)
            
        except (KeyError, IndexError, ValueError) as e:
            print(f"Warning: Error processing simulation log: {e}")
            return float('inf')
    
    def _compute_fitness_from_log_data(self, log_data: Dict[str, Any], parameters: List[float]) -> Union[float, List[float]]:
        """Compute fitness from log data dictionary. Returns single objective or multi-objective based on mode."""
        try:
            # Extract position and velocity data
            pos_des = np.stack([
                np.array(log_data["user_defined_position"]["x"]).flatten(),
                np.array(log_data["user_defined_position"]["y"]).flatten(),
                np.array(log_data["user_defined_position"]["z"]).flatten()
            ], axis=1)
            pos_act = np.stack([
                np.array(log_data["position"]["x"]).flatten(),
                np.array(log_data["position"]["y"]).flatten(),
                np.array(log_data["position"]["z"]).flatten()
            ], axis=1)
            
            vel_des = np.stack([
                np.array(log_data["user_defined_velocity"]["x"]).flatten(),
                np.array(log_data["user_defined_velocity"]["y"]).flatten(),
                np.array(log_data["user_defined_velocity"]["z"]).flatten()
            ], axis=1)
            vel_act = np.stack([
                np.array(log_data["velocity"]["x"]).flatten(),
                np.array(log_data["velocity"]["y"]).flatten(),
                np.array(log_data["velocity"]["z"]).flatten()
            ], axis=1)
            
            # Extract control effort data
            control_input = log_data.get("control_input", {})
            if isinstance(control_input, dict) and len(control_input) > 0:
                try:
                    # Control input is a dictionary with keys like 'U1', 'U2', 'U3', 'U4'
                    # Each value is a numpy array of control inputs over time
                    control_efforts = []
                    for key, control_array in control_input.items():
                        if hasattr(control_array, '__len__') and len(control_array) > 0:
                            # Calculate RMS for this control input
                            if control_array.ndim > 1:
                                # Flatten if multi-dimensional
                                control_array = control_array.flatten()
                            control_efforts.append(np.sqrt(np.mean(control_array**2)))
                    
                    if control_efforts:
                        # Total control effort as sum of individual control efforts
                        control_effort = np.sum(control_efforts)
                    else:
                        control_effort = 0.0
                except (ValueError, TypeError, IndexError) as e:
                    print(f"Warning: Error calculating control effort: {e}")
                    control_effort = 0.0
            else:
                control_effort = 0.0
            
            # Compute individual objectives
            pos_error = pos_act - pos_des
            vel_error = vel_act - vel_des
            
            # Position tracking error (RMSE)
            pos_tracking_error = np.sqrt(np.mean(np.sum(pos_error**2, axis=1)))
            
            # Velocity tracking error (RMSE)
            vel_tracking_error = np.sqrt(np.mean(np.sum(vel_error**2, axis=1)))
            
            # Check for NaN or Inf values
            if (np.isnan(pos_tracking_error) or np.isinf(pos_tracking_error) or
                np.isnan(vel_tracking_error) or np.isinf(vel_tracking_error) or
                np.isnan(control_effort) or np.isinf(control_effort)):
                if self.multi_objective:
                    return [float('inf'), float('inf'), float('inf')]
                else:
                    return float('inf')
            
            # Return based on optimization mode
            if self.multi_objective:
                # Multi-objective: return list of objectives
                objectives = [pos_tracking_error, vel_tracking_error, control_effort]
                print(f"[FITNESS] Parameters {parameters[:3]} -> Objectives: pos={pos_tracking_error:.6f}, vel={vel_tracking_error:.6f}, control={control_effort:.6f}")
                return objectives
            else:
                # Single-objective: return weighted sum
                weights = [3.0, 0.1, 0.01]  # [pos_weight, vel_weight, control_weight]
                total_fitness = (weights[0] * pos_tracking_error + 
                               weights[1] * vel_tracking_error + 
                               weights[2] * control_effort)
                print(f"[FITNESS] Parameters {parameters[:3]} -> Fitness: {total_fitness:.6f} (pos={pos_tracking_error:.6f}, vel={vel_tracking_error:.6f}, control={control_effort:.6f})")
                return float(total_fitness)
                
        except (KeyError, IndexError, ValueError) as e:
            print(f"Warning: Error processing simulation log: {e}")
            import traceback
            traceback.print_exc()
            if self.multi_objective:
                return [float('inf'), float('inf'), float('inf')]
            else:
                return float('inf')


def create_uav_ga_tuner(controller_type='PID',  # str
                       algorithm='DEAP',  # str
                       uav_adapter=None,  # Optional UAVModelAdapter
                       parameter_bounds=None,  # Optional ParameterBounds
                       population_size=100,  # int
                       n_generations=50,  # int
                       **kwargs):
    """
    Factory function to create a GA tuner configured for UAV simulations.
    
    Args:
        controller_type: Type of controller ('PID' or 'MRAC')
        algorithm: Algorithm to use ('DEAP' or 'Pymoo')
        uav_adapter: UAV adapter instance (created if None)
        parameter_bounds: Parameter bounds (created based on controller_type if None)
        population_size: Population size
        n_generations: Number of generations
        **kwargs: Additional arguments for the tuner
        
    Returns:
        Configured GA tuner
    """
    # Create UAV adapter if not provided
    if uav_adapter is None:
        uav_adapter = UAVModelAdapter()

    # Create parameter bounds if not provided
    pid_tuning = None
    if parameter_bounds is None:
        if controller_type.upper() == 'PID':
            from .controllers.pid_tuning import PIDTuning
            pid_tuning = PIDTuning()
            parameter_bounds = pid_tuning.get_parameter_bounds()
        elif controller_type.upper() == 'MRAC':
            from .controllers.mrac_tuning import MRACTuning
            mrac_tuning = MRACTuning()
            parameter_bounds = mrac_tuning.get_parameter_bounds()
        else:
            raise ValueError(f"Unknown controller_type: {controller_type}. Use 'PID' or 'MRAC'")
    
    # Create appropriate fitness evaluator based on controller type and mode
    fast_mode = kwargs.get('fast_mode', False)
    
    if fast_mode:
        # Use mathematical fitness for fast testing
        from .core.fitness_evaluator import FitnessEvaluator
        
        if controller_type.upper() == 'PID':
            from .controllers.pid_tuning import PIDTuning
            pid_helper = pid_tuning or PIDTuning()
            default_vector = np.asarray(
                pid_helper.gains_to_vector(pid_helper.get_default_gains()),
                dtype=float
            )
            expected_length = default_vector.shape[0]

            def fast_pid_fitness(parameters):
                params = np.asarray(parameters, dtype=float)
                if params.shape[0] != expected_length:
                    raise ValueError(
                        f"Expected {expected_length} PID parameters, received {params.shape[0]}"
                    )
                diff = params - default_vector
                return float(np.dot(diff, diff))
                
            fitness_evaluator = FitnessEvaluator(fast_pid_fitness, parallel=False)
            
        elif controller_type.upper() == 'MRAC':
            def fast_mrac_fitness(parameters):
                # Simple mathematical fitness for MRAC parameters
                # Penalize extreme values, reward reasonable adaptive rates
                cost = 0.0
                for i, param in enumerate(parameters):
                    if i < 18:  # Adaptive learning rates
                        target = 10.0 if i % 6 < 3 else 100.0  # Different targets for different groups
                        cost += abs(param - target)**2
                    else:
                        cost += abs(param - 1.0)**2  # Other parameters
                return cost
                
            fitness_evaluator = FitnessEvaluator(fast_mrac_fitness, parallel=False)
        else:
            raise ValueError(f"Unknown controller_type: {controller_type}. Use 'PID' or 'MRAC'")
    else:
        # Use real simulation fitness
        if controller_type.upper() == 'PID':
            fitness_evaluator = UAVFitnessEvaluator(
                uav_adapter=uav_adapter,
                log_directory="simulation_logs/ga_optimization",
                cost_function_weights={'total': 1.0},
                parallel_config=kwargs.get('parallel_config'),
                multi_objective=kwargs.get('multi_objective', False)
            )
        elif controller_type.upper() == 'MRAC':
            from .uav_evaluators import MRACSimulationEvaluator
            fitness_evaluator = MRACSimulationEvaluator(
                uav_adapter=uav_adapter,
                log_directory="simulation_logs/ga_optimization",
                parallel_config=kwargs.get('parallel_config')
            )
        else:
            raise ValueError(f"Unknown controller_type: {controller_type}. Use 'PID' or 'MRAC'")
    
    # Filter out parameters that tuners don't accept
    tuner_kwargs = {k: v for k, v in kwargs.items() if k not in ['controller_type', 'fast_mode']}
    
    # Ensure population_size and n_generations are passed to the tuner
    tuner_kwargs['population_size'] = population_size
    tuner_kwargs['n_generations'] = n_generations
    
    # Create appropriate tuner
    if algorithm.upper() == 'DEAP':
        from .algorithms.deap_ga import DEAPGATuner
        return DEAPGATuner(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    elif algorithm.upper() == 'PYMOO':
        from .algorithms.pymoo_ga import PymooGATuner
        return PymooGATuner(
            parameter_bounds=parameter_bounds,
            fitness_evaluator=fitness_evaluator,
            **tuner_kwargs
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}. Use 'DEAP' or 'Pymoo'")
