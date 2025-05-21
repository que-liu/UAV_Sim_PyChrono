import math
import numpy as np  
from acsl_pychrono.control.outerloop_safetymech import OuterLoopSafetyMechanism
from acsl_pychrono.control.MRAC.mrac_gains import MRACGains
from acsl_pychrono.ode_input import OdeInput
from acsl_pychrono.flight_params import FlightParams
from acsl_pychrono.control.control import Control
from acsl_pychrono.control.base_mrac import BaseMRAC

class MRAC(BaseMRAC, Control):
  def __init__(self, gains: MRACGains, ode_input: OdeInput, flight_params: FlightParams):
    self.gains = gains
    self.odein = ode_input
    self.fp = flight_params
    self.safety_mechanism = OuterLoopSafetyMechanism(gains, self.fp.G_acc)
    self.dy = np.zeros((self.gains.number_of_states, 1))

  def computeControlAlgorithm(self, y, ode_input: OdeInput):
    """
    Compute all intermediate variables and control inputs once per RK4 step to compute the dy for RK4.
    """
    # Update the vehicle state and user-defined trajectory
    self.odein = ode_input
    
    # ODE state            
    self.state_phi_ref_diff = y[0:2] # State of the differentiator for phi_ref (roll_ref)
    self.state_theta_ref_diff = y[2:4] # State of the differentiator for theta_ref (pitch_ref)
    self.x_ref_tran = y[4:10] # Reference model state
    self.integral_position_tracking_ref = y[10:13] # Integral of ('translational_position_in_I_ref' - 'translational_position_in_I_user')
    self.K_hat_x_tran = y[13:31] # \hat{K}_x (translational)
    self.K_hat_r_tran = y[31:40] # \hat{K}_r (translational)
    self.Theta_hat_tran = y[40:58] # \hat{\Theta} (translational)
    self.omega_ref = y[58:61] # Reference model rotational dynamics
    self.K_hat_x_rot = y[61:70] # \hat{K}_x (rotational)
    self.K_hat_r_rot = y[70:79] # \hat{K}_r (rotational)
    self.Theta_hat_rot = y[79:97] # \hat{\Theta} (rotational)
    self.integral_e_rot = y[97:100] # Integral of 'e_rot' = (angular_velocity - omega_ref) 
    self.integral_angular_error = y[100:103] # Integral of angular_error = attitude - attitude_ref
    self.integral_e_omega_ref_cmd = y[103:106] #Integral of (omega_ref - omega_cmd)

    # Reshapes all adaptive gains to their correct (row, col) shape as matrices
    self.reshapeAdaptiveGainsToMatrices()

    # compute translational and rotational trajectory tracking error
    self.computeTrajectoryTrackingErrors(self.odein)

    self.r_tran = self.computeReferenceCommandInputOuterLoop()

    self.x_ref_tran_dot = self.computeReferenceModelOuterLoop()

    self.mu_PD_baseline_tran = self.computeMuPDbaselineOuterLoop()

    (self.Phi_adaptive_tran_augmented,
     self.Theta_tran_adaptive_bar_augmented
    ) = self.computeRegressorVectorAndThetaBarOuterLoop()

    self.mu_baseline_tran = self.computeMuBaselineBarOuterLoop()

    self.mu_adaptive_tran = self.computeMuAdaptiveOuterLoop()

    self.mu_tran_raw = self.computeMuRawOuterLoop()

    # Precompute e^T*P*B for outer loop
    eTranspose_P_B_tran = self.compute_eTransposePB_OuterLoop()
    
    # Outer Loop Adaptive Laws
    self.computeAllAdaptiveLawsOuterLoop(eTranspose_P_B_tran)

    # Outer Loop Safety Mechanism
    self.mu_x, self.mu_y, self.mu_z = self.safety_mechanism.apply(self.mu_tran_raw)
    
    # Compute total thrust, desired roll angle, desired pitch angle
    (
    self.u1,
    self.roll_ref,
    self.pitch_ref
    ) = Control.computeU1RollPitchRef(
      self.mu_x, 
      self.mu_y, 
      self.mu_z, 
      self.gains.mass_total_estimated,
      self.fp.G_acc,
      self.odein.yaw_ref
    )

    # Computes roll/pitch reference dot and ddot using state-space differentiators.
    (
    self.internal_state_differentiator_phi_ref_diff,
    self.internal_state_differentiator_theta_ref_diff,
    self.angular_position_ref_dot,
    self.angular_position_ref_ddot
    ) = Control.computeAngularReferenceSignals(
      self.fp,
      self.odein,
      self.roll_ref,
      self.pitch_ref,
      self.state_phi_ref_diff,
      self.state_theta_ref_diff
    ) 

    # Computes angular error and its derivative
    (
    self.angular_error,
    self.angular_position_dot,
    self.angular_error_dot
    ) = Control.computeAngularErrorAndDerivative(
      self.odein,
      self.roll_ref,
      self.pitch_ref,
      self.angular_position_ref_dot
    )

    (self.omega_cmd,
     self.omega_cmd_dot
    ) = self.computeOmegaCmdAndOmegaCmdDotInnerLoop()

    self.omega_ref_dot = self.computeReferenceModelInnerLoop()

    self.r_rot = self.computeReferenceCommandInputInnerLoop()

    self.Moment_baseline_PI = self.computeMomentPIbaselineInnerLoop()

    (self.Phi_adaptive_rot,
     self.Phi_adaptive_rot_augmented
    ) = self.computeRegressorVectorInnerLoop()

    # Precompute e^T*P*B for inner loop
    eTranspose_P_B_rot = self.compute_eTransposePB_InnerLoop()

    # Inner Loop Adaptive Laws
    self.computeAllAdaptiveLawsInnerLoop(eTranspose_P_B_rot)

    self.Moment_baseline = self.computeMomentBaselineInnerLoop()

    self.Moment_adaptive = self.computeMomentAdaptiveInnerLoop()

    (self.u2,
     self.u3,
     self.u4
    ) = self.computeU2_U3_U4()

    # Compute individual motor thrusts
    self.motor_thrusts = Control.computeMotorThrusts(self.fp, self.u1, self.u2, self.u3, self.u4)
  
  def ode(self, t, y, ode_input: OdeInput):
    """
    Function called by RK4. Assumes `computeControlAlgorithm` was called
    at the beginning of the integration step to update internal state.
    """
    self.dy[0:2] = self.internal_state_differentiator_phi_ref_diff
    self.dy[2:4] = self.internal_state_differentiator_theta_ref_diff
    self.dy[4:10] = self.x_ref_tran_dot
    self.dy[10:13] = self.translational_position_in_I_ref - self.odein.translational_position_in_I_user
    self.dy[13:31] = self.K_hat_x_tran_dot.reshape(18,1)
    self.dy[31:40] = self.K_hat_r_tran_dot.reshape(9,1)
    self.dy[40:58] = self.Theta_hat_tran_dot.reshape(18,1)
    self.dy[58:61] = self.omega_ref_dot
    self.dy[61:70] = self.K_hat_x_rot_dot.reshape(9,1)
    self.dy[70:79] = self.K_hat_r_rot_dot.reshape(9,1)
    self.dy[79:97] = self.Theta_hat_rot_dot.reshape(18,1)
    self.dy[97:100] = self.odein.angular_velocity - self.omega_ref
    self.dy[100:103] = self.angular_error
    self.dy[103:106] = self.omega_ref - self.omega_cmd

    return np.array(self.dy)
  
  def reshapeAdaptiveGainsToMatrices(self):
    """
    Reshapes all gain parameters to their correct (row, col) shape and converts them to np.matrix.
    This is intended to be called once after loading or updating gains stored as flat arrays.
    """
    self.K_hat_x_tran = np.matrix(self.K_hat_x_tran.reshape(6,3))
    self.K_hat_r_tran = np.matrix(self.K_hat_r_tran.reshape(3,3))
    self.Theta_hat_tran = np.matrix(self.Theta_hat_tran.reshape(6,3))
    self.K_hat_x_rot = np.matrix(self.K_hat_x_rot.reshape(3,3))
    self.K_hat_r_rot = np.matrix(self.K_hat_r_rot.reshape(3,3))
    self.Theta_hat_rot = np.matrix(self.Theta_hat_rot.reshape(6,3))

  def computeAllAdaptiveLawsOuterLoop(self, eTranspose_P_B_tran):
    self.K_hat_x_tran_dot = self.computeAdaptiveLawMRAC(
      -self.gains.Gamma_x_tran,
      self.x_tran,
      eTranspose_P_B_tran
    )
    self.K_hat_r_tran_dot = self.computeAdaptiveLawMRAC(
      -self.gains.Gamma_r_tran,
      self.r_tran,
      eTranspose_P_B_tran
    )
    self.Theta_hat_tran_dot = self.computeAdaptiveLawMRAC(
      self.gains.Gamma_Theta_tran,
      self.Phi_adaptive_tran_augmented,
      eTranspose_P_B_tran
    )

  def computeAllAdaptiveLawsInnerLoop(self, eTranspose_P_B_rot):
    self.K_hat_x_rot_dot = self.computeAdaptiveLawMRAC(
      -self.gains.Gamma_x_rot,
      self.odein.angular_velocity,
      eTranspose_P_B_rot
    )
    self.K_hat_r_rot_dot = self.computeAdaptiveLawMRAC(
      -self.gains.Gamma_r_rot,
      self.r_rot,
      eTranspose_P_B_rot
    )
    self.Theta_hat_rot_dot = self.computeAdaptiveLawMRAC(
      self.gains.Gamma_Theta_rot,
      self.Phi_adaptive_rot_augmented,
      eTranspose_P_B_rot
    )