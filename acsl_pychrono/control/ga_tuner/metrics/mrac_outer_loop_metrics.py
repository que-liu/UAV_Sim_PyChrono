"""
MRAC Outer Loop Metrics Calculator

Computes performance metrics for MRAC outer loop (position/translational control).

Metrics:
- position_error: RMS position tracking error
- velocity_error: RMS velocity tracking error
- control_effort: RMS control effort (total thrust from all motors)
"""

import numpy as np
import os
import pickle
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class MRACOuterLoopMetrics:
    """Container for MRAC outer loop performance metrics."""
    position_error: float  # RMS position tracking error
    velocity_error: float  # RMS velocity tracking error
    control_effort: float  # RMS control effort (total thrust)
    
    def to_dict(self):
        """Convert metrics to dictionary."""
        return {
            'position_error': self.position_error,
            'velocity_error': self.velocity_error,
            'control_effort': self.control_effort
        }
    
    def to_list(self):
        """Convert metrics to list for multi-objective optimization."""
        return [
            self.position_error,
            self.velocity_error,
            self.control_effort
        ]


class MRACOuterLoopMetricsCalculator:
    """Calculator for MRAC outer loop performance metrics."""
    
    def __init__(self):
        """Initialize the outer loop metrics calculator."""
        pass
    
    def calculate_metrics(self, log_data: Dict) -> MRACOuterLoopMetrics:
        """
        Calculate MRAC outer loop performance metrics from simulation log.
        
        Args:
            log_data: Simulation log data dictionary
            
        Returns:
            MRACOuterLoopMetrics object with computed metrics
        """
        try:
            # Calculate performance metrics
            position_error = self._calculate_position_error(log_data)
            velocity_error = self._calculate_velocity_error(log_data)
            control_effort = self._calculate_control_effort(log_data)
            
            return MRACOuterLoopMetrics(
                position_error=position_error,
                velocity_error=velocity_error,
                control_effort=control_effort
            )
            
        except Exception as e:
            print(f"Error calculating MRAC outer loop metrics: {e}")
            return self._create_default_metrics()
    
    def _calculate_position_error(self, log_data: Dict) -> float:
        """Calculate RMS position tracking error."""
        try:
            # Extract position data
            pos_actual = self._extract_position_data(log_data, 'position')
            pos_desired = self._extract_position_data(log_data, 'user_defined_position')
            
            if pos_actual is None or pos_desired is None:
                return 999.0
            
            # Calculate RMS position error
            pos_error = np.sqrt(np.mean((pos_actual - pos_desired) ** 2))
            return float(pos_error)
            
        except Exception:
            return 999.0
    
    def _calculate_velocity_error(self, log_data: Dict) -> float:
        """Calculate RMS velocity tracking error."""
        try:
            # Extract velocity data
            vel_actual = self._extract_position_data(log_data, 'velocity')
            vel_desired = self._extract_position_data(log_data, 'user_defined_velocity')
            
            if vel_actual is None or vel_desired is None:
                return 999.0
            
            # Calculate RMS velocity error
            vel_error = np.sqrt(np.mean((vel_actual - vel_desired) ** 2))
            return float(vel_error)
            
        except Exception:
            return 999.0
    
    def _calculate_control_effort(self, log_data: Dict) -> float:
        """Calculate RMS control effort (total thrust from all motors)."""
        thrust_data = log_data.get('thrust_motors_N', {})
        if not thrust_data:
            return 999.0

        total_effort = 0.0
        for motor_data in thrust_data.values():
            if isinstance(motor_data, list):
                arr = np.array(motor_data).astype(float).flatten()
            else:
                try:
                    arr = np.array(motor_data, dtype=float).flatten()
                except Exception:
                    continue

            if arr.size == 0:
                continue

            total_effort += np.mean(arr ** 2)

        if total_effort == 0.0:
            return 999.0

        return float(np.sqrt(total_effort))
    
    def _extract_position_data(self, log_data: Dict, key: str) -> Optional[np.ndarray]:
        """Extract position/velocity data from log."""
        if key not in log_data:
            return None
        
        data = log_data[key]
        if isinstance(data, dict):
            # Extract x, y, z components
            x = np.array(data.get('x', []))
            y = np.array(data.get('y', []))
            z = np.array(data.get('z', []))
            
            if len(x) == len(y) == len(z) > 0:
                return np.column_stack([x, y, z])
        
        return None
    
    def _create_default_metrics(self) -> MRACOuterLoopMetrics:
        """Create default metrics for failed calculations."""
        return MRACOuterLoopMetrics(
            position_error=999.0,
            velocity_error=999.0,
            control_effort=999.0
        )


def load_simulation_log(log_path: str) -> Optional[Dict]:
    """Load simulation log from file."""
    try:
        if log_path is None:
            print(f"Error loading log None: log_path is None")
            return None
        
        if log_path.endswith('.pkl'):
            with open(log_path, 'rb') as f:
                return pickle.load(f)
        else:
            # Handle other formats if needed
            return None
    except Exception as e:
        print(f"Error loading log {log_path}: {e}")
        return None
