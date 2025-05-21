import math
import numpy as np

class OdeInput:
  def __init__(self):
    # Vehicle state
    self.translational_position_in_I = np.zeros((3, 1))
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0
    self.translational_velocity_in_I = np.zeros((3, 1))
    self.angular_velocity = np.zeros((3, 1))

    # User-defined trajectory
    self.translational_position_in_I_user = np.zeros((3, 1))
    self.translational_velocity_in_I_user = np.zeros((3, 1))
    self.translational_acceleration_in_I_user = np.zeros((3, 1))
    self.yaw_ref = 0.0
    self.yaw_ref_dot = 0.0
    self.yaw_ref_ddot = 0.0

  def update(self, 
             translational_position_in_I,
             roll,
             pitch,
             yaw,
             translational_velocity_in_I,
             angular_velocity,
             translational_position_in_I_user,
             translational_velocity_in_I_user,
             translational_acceleration_in_I_user,
             yaw_ref,
             yaw_ref_dot,
             yaw_ref_ddot):
    
    # Vehicle state
    self.translational_position_in_I = translational_position_in_I
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw
    self.translational_velocity_in_I = translational_velocity_in_I
    self.angular_velocity = angular_velocity

    # User-defined trajectory
    self.translational_position_in_I_user = translational_position_in_I_user
    self.translational_velocity_in_I_user = translational_velocity_in_I_user
    self.translational_acceleration_in_I_user = translational_acceleration_in_I_user
    self.yaw_ref = yaw_ref
    self.yaw_ref_dot = yaw_ref_dot
    self.yaw_ref_ddot = yaw_ref_ddot
