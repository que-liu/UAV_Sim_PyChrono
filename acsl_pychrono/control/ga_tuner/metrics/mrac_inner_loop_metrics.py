"""
MRAC Inner Loop Metrics Calculator

Computes performance metrics for MRAC inner loop (attitude/rotational control).

Metrics:
- attitude_tracking_error: RMS error in roll/pitch/yaw tracking (quaternion-based)
- angular_velocity_tracking_error: RMS error in angular rate tracking
- moment_effort: RMS control effort (moments u2, u3, u4)
"""

from typing import Dict, Any, Optional
from dataclasses import dataclass
import numpy as np


@dataclass
class MRACInnerLoopMetrics:
    """
    Container for MRAC inner loop performance metrics.
    Each metric is computed separately for sensitivity analysis and multi-objective optimization.
    """
    attitude_tracking_error: Optional[float] = None      # RMS error in roll, pitch, yaw tracking
    angular_velocity_tracking_error: Optional[float] = None  # RMS error in omega tracking
    moment_effort: Optional[float] = None                # RMS of control moments (u2, u3, u4)
    # adaptation_aggressiveness: Optional[float] = None    # Disabled: rate of change of adaptive parameters

    @staticmethod
    def _format_metric(value: Optional[float]) -> str:
        """Format metric for string output."""
        if value is None:
            return "N/A"
        return f"{value:.6f}"

    def __repr__(self):
        return (f"MRACInnerLoopMetrics("
                f"attitude_error={self._format_metric(self.attitude_tracking_error)}, "
                f"omega_error={self._format_metric(self.angular_velocity_tracking_error)}, "
                f"moment_effort={self._format_metric(self.moment_effort)}, "
                # f"adaptation_aggressiveness={self._format_metric(self.adaptation_aggressiveness)})"
                )
    
    def to_dict(self):
        """Convert metrics to dictionary."""
        data = {
            'attitude_tracking_error': self.attitude_tracking_error,
            'angular_velocity_tracking_error': self.angular_velocity_tracking_error,
            'moment_effort': self.moment_effort
        }

        '''if self.adaptation_aggressiveness is not None:
            data['adaptation_aggressiveness'] = self.adaptation_aggressiveness'''

        return data
    
    def to_list(self):
        """Convert metrics to list for multi-objective optimization."""
        metrics = [
            self.attitude_tracking_error,
            self.angular_velocity_tracking_error,
            self.moment_effort
        ]

        '''if self.adaptation_aggressiveness is not None:
            metrics.append(self.adaptation_aggressiveness)'''

        return metrics


class MRACInnerLoopMetricsCalculator:
    """
    Calculator for MRAC inner loop performance metrics.
    
    This class provides a centralized implementation of metric calculations
    to ensure consistency across different parts of the codebase.
    """
    
    def __init__(self):
        """Initialize the metrics calculator."""
        pass
    
    def compute_all_metrics(self, log_data: Dict[str, Any]) -> Dict[str, float]:
        """
        Compute all inner loop metrics from simulation log data.
        
        Args:
            log_data: Simulation log data dictionary
        
        Returns:
            Dictionary with the available metrics
        """
        metrics = {}
        
        try:
            metrics['attitude_tracking_error'] = self.compute_attitude_tracking_error(log_data)
            metrics['angular_velocity_tracking_error'] = self.compute_angular_velocity_tracking_error(log_data)
            metrics['moment_effort'] = self.compute_moment_effort(log_data)
        except Exception as e:
            print(f"Error computing MRAC inner loop metrics: {e}")
            # Return high penalty values if computation fails
            metrics = {
                'attitude_tracking_error': 999.0,
                'angular_velocity_tracking_error': 999.0,
                'moment_effort': 999.0
            }
        
        return metrics
    
    def compute_metrics_object(self, log_data: Dict[str, Any]) -> MRACInnerLoopMetrics:
        """
        Compute all metrics and return as MRACInnerLoopMetrics object.
        
        Args:
            log_data: Simulation log data dictionary
            
        Returns:
            MRACInnerLoopMetrics object with all metrics populated
        """
        metrics_dict = self.compute_all_metrics(log_data)
        
        return MRACInnerLoopMetrics(
            attitude_tracking_error=metrics_dict['attitude_tracking_error'],
            angular_velocity_tracking_error=metrics_dict['angular_velocity_tracking_error'],
            moment_effort=metrics_dict['moment_effort'],
            # adaptation_aggressiveness=metrics_dict.get('adaptation_aggressiveness')
        )
    
    def compute_attitude_tracking_error(self, log_data: Dict[str, Any]) -> float:
        """
        Compute attitude tracking error using quaternion-based method.
        
        This computes the relative quaternion q_err = q_actual^{-1} * q_user
        and extracts the rotation angle: θ_err = 2 * arccos(|w_err|)
        
        This is more mathematically rigorous than Euler angle differences
        and avoids gimbal lock issues.
        
        Attitude error measures how well the UAV tracks the desired orientation.
        This is crucial for inner loop performance as it directly relates to
        the rotational control objectives.
        """
        try:
            # Extract actual attitude (roll, pitch, yaw) from euler_angles
            actual_roll = np.array(log_data.get('euler_angles', {}).get('roll', [])).flatten()
            actual_pitch = np.array(log_data.get('euler_angles', {}).get('pitch', [])).flatten()
            actual_yaw = np.array(log_data.get('euler_angles', {}).get('yaw', [])).flatten()
            
            # Extract reference attitude from desired_euler_angles
            ref_roll = np.array(log_data.get('desired_euler_angles', {}).get('roll', [])).flatten()
            ref_pitch = np.array(log_data.get('desired_euler_angles', {}).get('pitch', [])).flatten()
            ref_yaw = np.array(log_data.get('user_defined_yaw', [])).flatten()
            
            # Ensure all arrays have the same length
            min_length = min(len(actual_roll), len(actual_pitch), len(actual_yaw),
                           len(ref_roll), len(ref_pitch), len(ref_yaw))
            
            if min_length == 0:
                return 999.0
            
            # Truncate to minimum length
            actual_roll = actual_roll[:min_length]
            actual_pitch = actual_pitch[:min_length]
            actual_yaw = actual_yaw[:min_length]
            ref_roll = ref_roll[:min_length]
            ref_pitch = ref_pitch[:min_length]
            ref_yaw = ref_yaw[:min_length]
            
            # Convert Euler angles to quaternions
            q_actual = self._euler_to_quaternion(actual_roll, actual_pitch, actual_yaw)
            q_ref = self._euler_to_quaternion(ref_roll, ref_pitch, ref_yaw)
            
            # Compute relative quaternion: q_err = q_actual^{-1} * q_ref
            q_actual_conj = self._quaternion_conjugate(q_actual)
            q_err = self._quaternion_multiply(q_actual_conj, q_ref)
            
            # Extract scalar part (w component) of error quaternion
            w_err = q_err[:, 0]
            
            # Compute attitude error angle: θ_err = 2 * arccos(|w_err|)
            w_err_clamped = np.clip(np.abs(w_err), 0.0, 1.0)
            theta_err = 2.0 * np.arccos(w_err_clamped)
            
            # Remove NaN values
            valid_mask = ~np.isnan(theta_err)
            theta_err_clean = theta_err[valid_mask]
            
            if len(theta_err_clean) == 0:
                return 999.0
            
            # Compute RMS of the attitude error angle
            rms_attitude_error = np.sqrt(np.mean(theta_err_clean ** 2))
            
            return float(rms_attitude_error)
            
        except Exception as e:
            print(f"Error computing attitude tracking error: {e}")
            return 999.0
    
    def compute_angular_velocity_tracking_error(self, log_data: Dict[str, Any]) -> float:
        """
        Compute RMS angular velocity tracking error.
        
        This measures how well the actual angular velocities track the reference
        model angular velocities (omega_ref). This is a direct measure of inner
        loop tracking performance.
        """
        try:
            # Extract actual angular velocity
            actual_omega = np.column_stack([
                np.array(log_data['angular_velocity']['x']).flatten(),
                np.array(log_data['angular_velocity']['y']).flatten(),
                np.array(log_data['angular_velocity']['z']).flatten()
            ])
            
            # Extract reference angular velocity from inner loop
            if 'inner_loop' not in log_data or 'reference_model' not in log_data['inner_loop']:
                return 999.0
                
            ref_omega = np.column_stack([
                np.array(log_data['inner_loop']['reference_model']['angular_velocity']['x']).flatten(),
                np.array(log_data['inner_loop']['reference_model']['angular_velocity']['y']).flatten(),
                np.array(log_data['inner_loop']['reference_model']['angular_velocity']['z']).flatten()
            ])
            
            # Ensure same length
            min_length = min(len(actual_omega), len(ref_omega))
            if min_length == 0:
                return 999.0
            
            actual_omega = actual_omega[:min_length]
            ref_omega = ref_omega[:min_length]
            
            # Calculate error
            omega_error = actual_omega - ref_omega
            
            # Remove NaN values
            valid_mask = ~np.isnan(omega_error).any(axis=1)
            omega_error_clean = omega_error[valid_mask]
            
            if len(omega_error_clean) == 0:
                return 999.0
            
            # Compute RMS error (Euclidean norm at each timestep, then mean)
            omega_error_magnitude = np.linalg.norm(omega_error_clean, axis=1)
            rms_omega_error = np.sqrt(np.mean(omega_error_magnitude ** 2))
            
            return float(rms_omega_error)
            
        except Exception as e:
            print(f"Error computing angular velocity error: {e}")
            return 999.0
    
    def compute_moment_effort(self, log_data: Dict[str, Any]) -> float:
        """
        Compute RMS control moment effort (u2, u3, u4).
        
        This measures the control effort used by the inner loop controller.
        Lower moment effort indicates more efficient control.
        u2, u3, u4 are the roll, pitch, and yaw moments respectively.
        """
        try:
            # Extract control moments from control_input
            u2 = np.array(log_data.get('control_input', {}).get('U2', [])).flatten()
            u3 = np.array(log_data.get('control_input', {}).get('U3', [])).flatten()
            u4 = np.array(log_data.get('control_input', {}).get('U4', [])).flatten()
            
            # Ensure all arrays have the same length
            min_length = min(len(u2), len(u3), len(u4))
            
            if min_length == 0:
                return 999.0
            
            # Truncate to minimum length
            u2 = u2[:min_length]
            u3 = u3[:min_length]
            u4 = u4[:min_length]
            
            # Stack into matrix
            moments = np.column_stack([u2, u3, u4])
            
            # Remove NaN values
            valid_mask = ~np.isnan(moments).any(axis=1)
            moments_clean = moments[valid_mask]
            
            if len(moments_clean) == 0:
                return 999.0
            
            # Compute RMS moment effort (Euclidean norm at each timestep, then RMS)
            moment_magnitude = np.linalg.norm(moments_clean, axis=1)
            rms_moment = np.sqrt(np.mean(moment_magnitude ** 2))
            
            return float(rms_moment)
            
        except Exception as e:
            print(f"Error computing moment effort: {e}")
            return 999.0
    
    # def compute_adaptation_aggressiveness(self, log_data: Dict[str, Any]) -> float:
    #     """
    #     Compute adaptation aggressiveness metric.
        
    #     This measures how aggressively the adaptive parameters change over time.
    #     High aggressiveness can indicate:
    #     - Fast adaptation (good for quick response to uncertainties)
    #     - Potential instability or chattering (bad for smooth control)
        
    #     compute this as:
    #     1. Extract adaptive parameter time histories (K_hat_x for inner loop, theta_hat for outer loop)
    #     2. Compute time derivative (rate of change)
    #     3. Calculate RMS of the rate of change across ALL adaptive parameters
    #     """
    #     try:
    #         # Extract time data
    #         time_data = np.array(log_data.get('time', [])).flatten()
            
    #         if len(time_data) < 2:
    #             return 999.0
            
    #         param_histories = []
            
    #         # Extract inner loop adaptive parameters (K_hat_x for rotational dynamics)
    #         if 'inner_loop' in log_data and 'K_hat_x' in log_data['inner_loop']:
    #             K_hat_x_data = log_data['inner_loop']['K_hat_x']
                
    #             # Stack all K_hat_x parameters
    #             param_keys = sorted([k for k in K_hat_x_data.keys() if k.startswith('ind')])
    #             for key in param_keys:
    #                 param_data = np.array(K_hat_x_data[key]).flatten()
    #                 param_histories.append(param_data)
            
    #         # Extract outer loop adaptive parameters (theta_hat for translational dynamics)
    #         if 'outer_loop' in log_data and 'theta_hat' in log_data['outer_loop']:
    #             theta_hat_data = log_data['outer_loop']['theta_hat']
                
    #             # Stack all theta_hat parameters
    #             param_keys = sorted([k for k in theta_hat_data.keys() if k.startswith('ind')])
    #             for key in param_keys:
    #                 param_data = np.array(theta_hat_data[key]).flatten()
    #                 param_histories.append(param_data)
            
    #         if len(param_histories) == 0:
    #             return 999.0
            
    #         # Ensure all have the same length
    #         min_length = min(len(ph) for ph in param_histories)
    #         min_length = min(min_length, len(time_data))
            
    #         if min_length < 2:
    #             return 999.0
            
    #         # Truncate to minimum length
    #         time_data = time_data[:min_length]
    #         param_histories = [ph[:min_length] for ph in param_histories]
            
    #         # Stack parameters
    #         params_matrix = np.column_stack(param_histories)
            
    #         # Remove timesteps with NaN values
    #         valid_mask = ~np.isnan(params_matrix).any(axis=1) & ~np.isnan(time_data)
    #         params_clean = params_matrix[valid_mask]
    #         time_clean = time_data[valid_mask]
            
    #         if len(params_clean) < 2:
    #             return 999.0
            
    #         # Compute time derivatives using finite differences
    #         dt = np.diff(time_clean)
    #         dparam_dt = np.diff(params_clean, axis=0) / dt[:, np.newaxis]
            
    #         # Remove any NaN or inf from derivatives
    #         valid_deriv_mask = ~(np.isnan(dparam_dt).any(axis=1) | np.isinf(dparam_dt).any(axis=1))
    #         dparam_dt_clean = dparam_dt[valid_deriv_mask]
            
    #         if len(dparam_dt_clean) == 0:
    #             return 999.0
            
    #         # Compute RMS of rate of change (across all parameters and time)
    #         rms_rate = np.sqrt(np.mean(dparam_dt_clean ** 2))
            
    #         return float(rms_rate)
            
    #     except Exception as e:
    #         print(f"Error computing adaptation aggressiveness: {e}")
    #         return 999.0
    
    # =========================================================================
    # Quaternion Helper Methods
    # =========================================================================
    
    def _euler_to_quaternion(self, roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
        """
        Convert Euler angles (3-2-1 sequence) to quaternion.
        
        Args:
            roll: Roll angle in radians (rotation about x-axis)
            pitch: Pitch angle in radians (rotation about y-axis)
            yaw: Yaw angle in radians (rotation about z-axis)
            
        Returns:
            quaternion: Array of shape (N, 4) with [w, x, y, z] components
        """
        # Ensure inputs are 1D arrays
        roll = np.atleast_1d(roll).flatten()
        pitch = np.atleast_1d(pitch).flatten()
        yaw = np.atleast_1d(yaw).flatten()
        
        # Half angles
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        # Quaternion components [w, x, y, z]
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        # Stack into (N, 4) array
        quaternion = np.column_stack([w, x, y, z])
        
        return quaternion
    
    def _quaternion_conjugate(self, q: np.ndarray) -> np.ndarray:
        """
        Compute the conjugate of a quaternion.
        
        Args:
            q: Quaternion array of shape (N, 4) with [w, x, y, z] components
            
        Returns:
            q_conj: Conjugate quaternion [w, -x, -y, -z]
        """
        q_conj = q.copy()
        q_conj[:, 1:] = -q_conj[:, 1:]  # Negate x, y, z components
        return q_conj
    
    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        Multiply two quaternions element-wise.
        
        Args:
            q1: First quaternion array of shape (N, 4) with [w, x, y, z]
            q2: Second quaternion array of shape (N, 4) with [w, x, y, z]
            
        Returns:
            q_result: Product q1 * q2
        """
        w1, x1, y1, z1 = q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3]
        w2, x2, y2, z2 = q2[:, 0], q2[:, 1], q2[:, 2], q2[:, 3]
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return np.column_stack([w, x, y, z])
