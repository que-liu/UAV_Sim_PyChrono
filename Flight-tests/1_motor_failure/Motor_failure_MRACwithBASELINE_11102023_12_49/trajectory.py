import math
import numpy as np

PI = math.pi

class circular_trajectory:
    def user_defined_trajectory(t, radius_trajectory, angular_velocity_trajectory, altitude_trajectory):
        "Circular trajectory at a constant altitude"
        
        translational_position_in_I_user = np.zeros((3,1))
        translational_velocity_in_I_user = np.zeros((3,1))
        translational_acceleration_in_I_user = np.zeros((3,1))
        
        # translational_position_in_I_user[0] = radius_trajectory*math.cos(-angular_velocity_trajectory * t) - radius_trajectory/2 
        translational_position_in_I_user[0] = radius_trajectory*math.cos(-angular_velocity_trajectory * t) - radius_trajectory
        translational_position_in_I_user[1] = radius_trajectory*math.sin(-angular_velocity_trajectory * t)
        translational_position_in_I_user[2] = altitude_trajectory
        
        translational_velocity_in_I_user[0] = -angular_velocity_trajectory * radius_trajectory * math.sin(angular_velocity_trajectory * t)
        translational_velocity_in_I_user[1] = -angular_velocity_trajectory * radius_trajectory * math.cos(angular_velocity_trajectory * t)
        translational_velocity_in_I_user[2] = 0
        
        translational_acceleration_in_I_user[0] = -angular_velocity_trajectory**2 * radius_trajectory * math.cos(angular_velocity_trajectory * t)
        translational_acceleration_in_I_user[1] = angular_velocity_trajectory**2 * radius_trajectory * math.sin(angular_velocity_trajectory * t)
        translational_acceleration_in_I_user[2] = 0
        
        return [translational_position_in_I_user, translational_velocity_in_I_user, translational_acceleration_in_I_user]
    
    # def user_defined_yaw(t, angular_velocity_trajectory):
    #     "User-defined reference yaw angle"
        
    #     psi_ref = -angular_velocity_trajectory * t
    #     psi_ref_dot = -angular_velocity_trajectory
    #     psi_ref_ddot = 0
        
    #     return [psi_ref, psi_ref_dot, psi_ref_ddot]
    
    
    def user_defined_yaw(t, angular_velocity_trajectory):
        "User-defined reference yaw angle"
        if (t < 0.5):
            psi_ref = -angular_velocity_trajectory * t  - math.pi/8
        elif (t >= 0.5 and t < 1):
            psi_ref = -angular_velocity_trajectory * t  - math.pi/4
        elif (t >= 1 and t < 1.5):
            psi_ref = -angular_velocity_trajectory * t  - 3*math.pi/8
        else:    
            psi_ref = -angular_velocity_trajectory * t  - math.pi/2
            
        psi_ref_dot = -angular_velocity_trajectory
        psi_ref_ddot = 0
        
        return [psi_ref, psi_ref_dot, psi_ref_ddot]
    
    
# ==============================================================================================================================
# 
# ==============================================================================================================================
    
    
class hover_trajectory:
    def user_defined_trajectory(t, altitude_trajectory):
        "Circular trajectory at a constant altitude"
        
        translational_position_in_I_user = np.zeros((3,1))
        translational_velocity_in_I_user = np.zeros((3,1))
        translational_acceleration_in_I_user = np.zeros((3,1))
        
        translational_position_in_I_user[0] = 0 
        translational_position_in_I_user[1] = 0
        translational_position_in_I_user[2] = altitude_trajectory
        
        translational_velocity_in_I_user[0] = 0
        translational_velocity_in_I_user[1] = 0
        translational_velocity_in_I_user[2] = 0
        
        translational_acceleration_in_I_user[0] = 0
        translational_acceleration_in_I_user[1] = 0
        translational_acceleration_in_I_user[2] = 0
        
        return [translational_position_in_I_user, translational_velocity_in_I_user, translational_acceleration_in_I_user]
    
    def user_defined_yaw(t):
        "User-defined reference yaw angle"
        
        psi_ref = 0
        psi_ref_dot = 0
        psi_ref_ddot = 0
        
        return [psi_ref, psi_ref_dot, psi_ref_ddot]
  
    
# ==============================================================================================================================
# 
# ==============================================================================================================================
    
    
class square_trajectory:
    def user_defined_trajectory(t, square_side_size, linear_velocity_trajectory, altitude_trajectory):
        "Square trajectory at a constant altitude"
        
        translational_position_in_I_user = np.zeros((3,1))
        translational_velocity_in_I_user = np.zeros((3,1))
        translational_acceleration_in_I_user = np.zeros((3,1))
        
        time_side = square_side_size/linear_velocity_trajectory # time required to complete one side of the square
        
        if t < time_side:
            # First segment of the square
            translational_position_in_I_user[0] = linear_velocity_trajectory * t
            translational_position_in_I_user[1] = 0
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = linear_velocity_trajectory
            translational_velocity_in_I_user[1] = 0
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = 0
            translational_acceleration_in_I_user[1] = 0
            translational_acceleration_in_I_user[2] = 0
        
        elif (t >= time_side and t < 2*time_side):
            # Second segment of the square
            translational_position_in_I_user[0] = square_side_size
            translational_position_in_I_user[1] = -square_side_size + linear_velocity_trajectory * t
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = 0
            translational_velocity_in_I_user[1] = linear_velocity_trajectory
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = 0
            translational_acceleration_in_I_user[1] = 0
            translational_acceleration_in_I_user[2] = 0
        
        return [translational_position_in_I_user, translational_velocity_in_I_user, translational_acceleration_in_I_user]
    
    def user_defined_yaw(t, square_side_size, linear_velocity_trajectory):
        "User-defined reference yaw angle"
        
        time_side = square_side_size/linear_velocity_trajectory # time required to complete one side of the square
        
        if t < time_side:
            # First segment of the square
            psi_ref = 0
            psi_ref_dot = 0
            psi_ref_ddot = 0
            
        elif (t >= time_side and t < 2*time_side):
            # Second segment of the square
            psi_ref = math.pi/2
            # psi_ref = min(math.pi/2, (t-time_side) * 0.1)
            # psi_ref = 0
            psi_ref_dot = 0
            psi_ref_ddot = 0
        
        return [psi_ref, psi_ref_dot, psi_ref_ddot]
    
    
# ==============================================================================================================================
# 
# ==============================================================================================================================    


class roundedRectangle_trajectory:
    def user_defined_trajectory(t, length_horizontal, length_vertical, rounding_radius, linear_velocity_trajectory, altitude_trajectory):
        "Rounded rectangle trajectory at a constant altitude"
        
        translational_position_in_I_user = np.zeros((3,1))
        translational_velocity_in_I_user = np.zeros((3,1))
        translational_acceleration_in_I_user = np.zeros((3,1))
        
        # Compute the time needed to span each segment
        # Segment 1: From (0, 0)                                                                         to (length_horizontal, 0)
        # Segment 2: From (length_horizontal, 0)                                                         to (length_horizontal + rounding_radius, rounding_radius)
        # Segment 3: From (length_horizontal + rounding_radius, rounding_radius)                         to (length_horizontal + rounding_radius, length_vertical + rounding_radius)
        # Segment 4: From (length_horizontal + rounding_radius, length_vertical + rounding_radius)       to (length_horizontal , length_vertical + 2*rounding_radius)
        # Segment 5: From (length_horizontal , length_vertical + 2*rounding_radius)                      to (0, length_vertical + 2*rounding_radius)
        # Segment 6: From (0, length_vertical + 2*rounding_radius)                                       to (-rounding_radius, length_vertical + rounding_radius)
        # Segment 7: From (-rounding_radius, length_vertical + rounding_radius)                          to (-rounding_radius, rounding_radius)
        # Segment 8: From (-rounding_radius, rounding_radius)                                            to (0, 0)
        
        # Segment 1: From t_0 = 0                                                          to t_1 = length_horizontal/linear_velocity_trajectory
        # Segment 2: From t_1 = length_horizontal/linear_velocity_trajectory               to t_2 = t_1 + (pi/4*rounding_radius)/linear_velocity_trajectory
        # Segment 3: From t_2 = t_1 + (pi/4*rounding_radius)/linear_velocity_trajectory    to t_3 = t_2 + length_vertical/linear_velocity_trajectory
        # Segment 4: From t_3 = t_2 + length_vertical/linear_velocity_trajectory           to t_4 = t_3 + (pi/4*rounding_radius)/linear_velocity_trajectory
        # Segment 5: From t_4 = t_3 + (pi/4*rounding_radius)/linear_velocity_trajectory    to t_5 = t_4 + length_horizontal/linear_velocity_trajectory
        # Segment 6: From t_5 = t_4 + length_horizontal/linear_velocity_trajectory         to t_6 = t_5 + (pi/4*rounding_radius)/linear_velocity_trajectory
        # Segment 7: From t_6 = t_5 + (pi/4*rounding_radius)/linear_velocity_trajectory    to t_7 = t_6 + length_vertical/linear_velocity_trajectory
        # Segment 8: From t_7 = t_6 + length_vertical/linear_velocity_trajectory           to T_max
        
        # Compute the constant angular velocity on smoothed corners
        omega_corner = linear_velocity_trajectory/rounding_radius
                
        t_1 = length_horizontal/linear_velocity_trajectory;
        t_2 = t_1 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        t_3 = t_2 + length_vertical/linear_velocity_trajectory;
        t_4 = t_3 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        t_5 = t_4 + length_horizontal/linear_velocity_trajectory;
        t_6 = t_5 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        t_7 = t_6 + length_vertical/linear_velocity_trajectory;
        t_8 = t_7 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        
        if (t >= 0 and t < t_1):
            # 1 Top horizontal segment
            translational_position_in_I_user[0] = linear_velocity_trajectory * t
            translational_position_in_I_user[1] = 0
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = linear_velocity_trajectory
            translational_velocity_in_I_user[1] = 0
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = 0
            translational_acceleration_in_I_user[1] = 0
            translational_acceleration_in_I_user[2] = 0
        
        elif (t >= t_1 and t < t_2):
            # 2 Top-right rounding radius
            translational_position_in_I_user[0] = length_horizontal + rounding_radius * math.cos(-PI/2 + omega_corner * (t - t_1))
            translational_position_in_I_user[1] = rounding_radius + rounding_radius * math.sin(-PI/2 + omega_corner * (t - t_1))
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = rounding_radius * omega_corner * math.cos(omega_corner * (t - t_1))
            translational_velocity_in_I_user[1] = rounding_radius * omega_corner * math.sin(omega_corner * (t - t_1))
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = -rounding_radius * omega_corner**2 * math.sin(omega_corner * (t - t_1))
            translational_acceleration_in_I_user[1] = rounding_radius * omega_corner**2 * math.cos(omega_corner * (t - t_1))
            translational_acceleration_in_I_user[2] = 0
            
        elif (t >= t_2 and t < t_3):
            # 3 Right vertical segment
            translational_position_in_I_user[0] = length_horizontal + rounding_radius
            translational_position_in_I_user[1] = rounding_radius + linear_velocity_trajectory*(t - t_2)
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = 0
            translational_velocity_in_I_user[1] = linear_velocity_trajectory
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = 0
            translational_acceleration_in_I_user[1] = 0
            translational_acceleration_in_I_user[2] = 0
            
        elif (t >= t_3 and t < t_4):
            # 4 Bottom-right rounding radius
            translational_position_in_I_user[0] = length_horizontal - rounding_radius * math.sin(-PI/2 + omega_corner * (t - t_3))
            translational_position_in_I_user[1] = rounding_radius + length_vertical + rounding_radius * math.cos(-PI/2 + omega_corner * (t - t_3))
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = -rounding_radius * omega_corner * math.sin(omega_corner * (t - t_3))
            translational_velocity_in_I_user[1] = rounding_radius * omega_corner * math.cos(omega_corner * (t - t_3))
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = -rounding_radius * omega_corner**2 * math.cos(omega_corner * (t - t_3))
            translational_acceleration_in_I_user[1] = -rounding_radius * omega_corner**2 * math.sin(omega_corner * (t - t_3))
            translational_acceleration_in_I_user[2] = 0
            
        elif (t >= t_4 and t < t_5):
            # 5 Bottom horizontal segment
            translational_position_in_I_user[0] = length_horizontal - linear_velocity_trajectory * (t - t_4)
            translational_position_in_I_user[1] = length_vertical + 2*rounding_radius
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = -linear_velocity_trajectory
            translational_velocity_in_I_user[1] = 0
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = 0
            translational_acceleration_in_I_user[1] = 0
            translational_acceleration_in_I_user[2] = 0
            
        elif (t >= t_5 and t < t_6):       
            # 6 Bottom-left rounding radius
            translational_position_in_I_user[0] = 0 - rounding_radius * math.cos(-PI/2 + omega_corner * (t - t_5))
            translational_position_in_I_user[1] = length_vertical + rounding_radius - rounding_radius * math.sin(-PI/2 + omega_corner * (t - t_5))
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = -rounding_radius * omega_corner * math.cos(omega_corner * (t - t_5))
            translational_velocity_in_I_user[1] = -rounding_radius * omega_corner * math.sin(omega_corner * (t - t_5))
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = rounding_radius * omega_corner**2 * math.sin(omega_corner * (t - t_5))
            translational_acceleration_in_I_user[1] = -rounding_radius * omega_corner**2 * math.cos(omega_corner * (t - t_5))
            translational_acceleration_in_I_user[2] = 0
            
        elif (t >= t_6 and t < t_7):
            # 7 Left vertical segment
            translational_position_in_I_user[0] = -rounding_radius
            translational_position_in_I_user[1] = rounding_radius + length_vertical - linear_velocity_trajectory * (t - t_6)
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = 0
            translational_velocity_in_I_user[1] = -linear_velocity_trajectory
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = 0
            translational_acceleration_in_I_user[1] = 0
            translational_acceleration_in_I_user[2] = 0
            
        elif (t >= t_7 and t < t_8):
            # 8 Bottom-left rounding radius
            translational_position_in_I_user[0] = 0 + rounding_radius * math.sin(-PI/2 + omega_corner * (t - t_7))
            translational_position_in_I_user[1] = rounding_radius - rounding_radius * math.cos(-PI/2 + omega_corner * (t - t_7))
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = rounding_radius * omega_corner * math.sin(omega_corner * (t - t_7))
            translational_velocity_in_I_user[1] = -rounding_radius * omega_corner * math.cos(omega_corner * (t - t_7))
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = rounding_radius * omega_corner**2 * math.cos(omega_corner * (t - t_7))
            translational_acceleration_in_I_user[1] = rounding_radius * omega_corner**2 * math.sin(omega_corner * (t - t_7))
            translational_acceleration_in_I_user[2] = 0
            
        else:
            translational_position_in_I_user[0] = 0
            translational_position_in_I_user[1] = 0
            translational_position_in_I_user[2] = altitude_trajectory
            
            translational_velocity_in_I_user[0] = 0
            translational_velocity_in_I_user[1] = 0
            translational_velocity_in_I_user[2] = 0
            
            translational_acceleration_in_I_user[0] = 0
            translational_acceleration_in_I_user[1] = 0
            translational_acceleration_in_I_user[2] = 0
        
        return [translational_position_in_I_user, translational_velocity_in_I_user, translational_acceleration_in_I_user]
    
    def user_defined_yaw(t, length_horizontal, length_vertical, rounding_radius, linear_velocity_trajectory, altitude_trajectory):
        "User-defined reference yaw angle" # TO BE MODIFIEEEEEEEDDDDDD!
        
        # Compute the constant angular velocity on smoothed corners
        omega_corner = linear_velocity_trajectory/rounding_radius
                
        t_1 = length_horizontal/linear_velocity_trajectory;
        t_2 = t_1 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        t_3 = t_2 + length_vertical/linear_velocity_trajectory;
        t_4 = t_3 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        t_5 = t_4 + length_horizontal/linear_velocity_trajectory;
        t_6 = t_5 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        t_7 = t_6 + length_vertical/linear_velocity_trajectory;
        t_8 = t_7 + (rounding_radius*PI/2)/linear_velocity_trajectory;
        
        if (t >= 0 and t < t_1):
            # 1 Top horizontal segment
            psi_ref = 0
            psi_ref_dot = 0
            psi_ref_ddot = 0
            
        elif (t >= t_1 and t < t_2):
            # 2 Top-right rounding radius
            psi_ref = omega_corner * (t - t_1)
            psi_ref_dot = omega_corner
            psi_ref_ddot = 0
            
        elif (t >= t_2 and t < t_3):
            # 3 Right vertical segment
            psi_ref = PI/2
            psi_ref_dot = 0
            psi_ref_ddot = 0
        
        elif (t >= t_3 and t < t_4):
            # 4 Bottom-right rounding radius
            psi_ref = PI/2 + omega_corner * (t - t_3)
            psi_ref_dot = omega_corner
            psi_ref_ddot = 0
            
        elif (t >= t_4 and t < t_5):
            # 5 Bottom horizontal segment
            psi_ref = PI
            psi_ref_dot = 0
            psi_ref_ddot = 0
            
        elif (t >= t_5 and t < t_6):
            # 6 Bottom-left rounding radius
            psi_ref = PI + omega_corner * (t - t_5)
            psi_ref_dot = omega_corner
            psi_ref_ddot = 0
            
        elif (t >= t_6 and t < t_7):
            # 7 Left vertical segment
            psi_ref = 3*PI/2
            psi_ref_dot = 0
            psi_ref_ddot = 0
            
        elif (t >= t_7 and t < t_8):
            # 8 Bottom-left rounding radius
            psi_ref = 3*PI/2 + omega_corner * (t - t_7)
            psi_ref_dot = omega_corner
            psi_ref_ddot = 0
            
        else:
            # 7 Left vertical segment
            psi_ref = 0
            psi_ref_dot = 0
            psi_ref_ddot = 0
        
        return [psi_ref, psi_ref_dot, psi_ref_ddot]


















