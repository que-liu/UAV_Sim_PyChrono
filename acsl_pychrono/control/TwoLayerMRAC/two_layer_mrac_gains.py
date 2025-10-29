import math
import numpy as np
from numpy import linalg as LA
from scipy import linalg
import scipy
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.control.projection_operator import ProjectionOperator
from acsl_pychrono.control.base_mrac_gains import BaseMRACGains

class TwoLayerMRACGains(BaseMRACGains):
  def __init__(self, flight_params: FlightParams):
    # General vehicle properties
    self.I_matrix_estimated = flight_params.I_matrix_estimated
    self.mass_total_estimated = flight_params.mass_total_estimated
    self.air_density_estimated = flight_params.air_density_estimated
    self.surface_area_estimated = flight_params.surface_area_estimated
    self.drag_coefficient_matrix_estimated = flight_params.drag_coefficient_matrix_estimated

    # Number of states to be integrated by RK4
    self.number_of_states = 133
    # Length of the array vector that will be exported 
    self.size_DATA = 222

    # ----------------------------------------------------------------
    #                     Baseline Parameters
    # ----------------------------------------------------------------

    # **Translational** baseline parameters to let the reference model follow the user-defined model (mu_baseline_tran)
    self.KP_tran = np.matrix(1 * np.diag([5,5,6]))
    self.KD_tran = np.matrix(1 * np.diag([8,8,3]))
    self.KI_tran = np.matrix(1 * np.diag([1,1,0.1]))

    # **Translational** parameters for the PD baseline controller (mu_PD_baseline_tran)
    self.KP_tran_PD_baseline = np.matrix(1 * np.diag([5,5,6]))
    self.KD_tran_PD_baseline = np.matrix(1 * np.diag([8,8,3]))

    # **Rotational** baseline parameters
    self.KP_rot = np.matrix(3e0 * np.diag([10,10,5]))
    self.KI_rot = np.matrix(2e0 * np.diag([1,1,1]))

    # **Rotational** parameters for the PI baseline controller (Moment_baseline_PI)       
    self.KP_rot_PI_baseline = np.matrix(4.5e1 * np.diag([1,1,0.5]))
    self.KI_rot_PI_baseline = np.matrix(5.5e1 * np.diag([1,1,0.5]))

    self.K_P_omega_ref = np.matrix(3.8e1 * np.diag([0.8,0.8,1.2]))
    self.K_I_omega_ref = np.matrix(1e-1 * np.diag([5,5,1]))

    # ----------------------------------------------------------------
    #                   Translational Parameters MRAC
    # ----------------------------------------------------------------

    # Plant parameters **Translational** dynamics
    self.A_tran = np.block([[np.zeros((3, 3)),   np.identity(3)],
                        [np.zeros((3, 3)), np.zeros((3, 3))]])

    self.B_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                      [np.identity(3)]]))

    # **Translational** reference model parameters and estimates
    self.A_ref_tran = np.block([[np.zeros((3, 3)),  np.identity(3)],
                            [        -self.KP_tran,        -self.KD_tran]])

    self.B_ref_tran = np.matrix(np.block([[np.zeros((3, 3))],
                                      [(1/self.mass_total_estimated)*np.identity(3)]]))

    # **Translational** adaptive parameters
    self.Gamma_x_tran = np.matrix(3e3 * np.diag([1,1,10,1,1,10])) # Adaptive rates
    self.Gamma_r_tran = np.matrix(5e2 * np.diag([1,1,4])) # Adaptive rates
    self.Gamma_Theta_tran = np.matrix(1e3 * np.diag([1,1,2,1,1,2])) # Adaptive rates

    # **Translational** parameters Lyapunov equation
    self.Q_tran = np.matrix(2e-2 * np.diag([1,1,12,1,1,2]))
    
    # ----------------------------------------------------------------
    #                   Rotational Parameters MRAC
    # ----------------------------------------------------------------

    # Plant parameters **Rotational** dynamics
    self.A_rot = np.matrix(np.zeros((3,3)))
    self.B_rot = np.matrix(np.eye(3))

    # **Rotational** reference model parameters
    self.A_ref_rot = -self.K_P_omega_ref
    self.B_ref_rot = np.matrix(np.eye(3))

    # **Rotational** parameters Lyapunov equation
    self.Q_rot = np.matrix(1e-3 * np.diag([1,1,1]))
    
    # **Rotational** adaptive parameters
    self.Gamma_x_rot = np.matrix(1e4 * np.diag([1,1,1])) # Adaptive rates
    self.Gamma_r_rot = np.matrix(5e0 * np.diag([1,1,1])) # Adaptive rates
    self.Gamma_Theta_rot = np.matrix(2e3 * np.diag([1,1,1,1,1,1])) # Adaptive rates

    # ----------------------------------------------------------------
    #                   Two-Layer MRAC Parameters
    # ----------------------------------------------------------------

    # **Translational** second layer parameters
    poles_ref_tran = LA.eig(self.A_ref_tran)[0]
    poles_transient_tran = poles_ref_tran + 2*np.min(np.real(poles_ref_tran))
    K_transient_tran = scipy.signal.place_poles(self.A_tran, self.B_ref_tran, poles_transient_tran)
    K_transient_tran = np.matrix(K_transient_tran.gain_matrix)
    self.A_transient_tran = self.A_tran - self.B_ref_tran*K_transient_tran 

    self.P_tran = np.matrix(linalg.solve_continuous_lyapunov(self.A_transient_tran.T, -self.Q_tran))
    self.Gamma_g_tran = np.matrix(1e4 * np.diag([1,1,1,1,1,1])) # Adaptive rates

    # **Rotational** second layer parameters
    poles_ref_rot = LA.eig(self.A_ref_rot)[0]
    poles_transient_rot = poles_ref_rot + np.min(np.real(poles_ref_rot))
    K_transient_rot = scipy.signal.place_poles(self.A_rot, self.B_ref_rot, poles_transient_rot)
    K_transient_rot = np.matrix(K_transient_rot.gain_matrix)
    self.A_transient_rot = self.A_rot - self.B_ref_rot*K_transient_rot
    
    self.P_rot = np.matrix(linalg.solve_continuous_lyapunov(self.A_transient_rot.T, -self.Q_rot))
    self.Gamma_g_rot = np.matrix(1e2 * np.diag([2,2,1])) # Adaptive rates
    
    # ----------------------------------------------------------------
    #                   Safety Mechanism Parameters
    # ----------------------------------------------------------------
    self.use_safety_mechanism = True
    
    # Mu - sphere intersection
    self.sphereEpsilon = 1e-2
    self.maximumThrust = 85 # [N] 85
    
    # Mu - elliptic cone intersection
    self.EllipticConeEpsilon = 1e-2
    self.maximumRollAngle = math.radians(60) # [rad] 25 - 32
    self.maximumPitchAngle = math.radians(60) # [rad] 25 - 32
    
    # Mu - plane intersection
    self.planeEpsilon = 1e-2
    self.alphaPlane = 0.6 # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.

    # ----------------------------------------------------------------
    #                  Dead-Zone modification Parameters
    # ----------------------------------------------------------------
    self.use_dead_zone_modification = True

    self.dead_zone_delta_tran = 0.5
    self.dead_zone_e0_tran = 0.01

    self.dead_zone_delta_rot = 0.5
    self.dead_zone_e0_rot = 0.002

    # ----------------------------------------------------------------
    #                  e-modification Parameters
    # ----------------------------------------------------------------
    self.use_e_modification = True

    self.sigma_x_tran = 0.5
    self.sigma_r_tran = 0.5
    self.sigma_Theta_tran = 0.5
    self.sigma_g_tran = 0.5

    self.sigma_x_rot = 0.5
    self.sigma_r_rot = 0.5
    self.sigma_Theta_rot = 0.5
    self.sigma_g_rot = 0.5

    # ----------------------------------------------------------------
    #                  Projection Operator Parameters
    # ----------------------------------------------------------------
    self.use_projection_operator = True

    # K_x_hat translational
    self.x_e_x_tran = np.zeros((18, 1))
    self.S_diagonal_x_tran = 30 * np.ones((18, 1))
    self.alpha_x_tran = 0.1

    # K_r_hat translational
    self.x_e_r_tran = np.zeros((9, 1))
    self.S_diagonal_r_tran = 2.5 * np.ones((9, 1))
    self.alpha_r_tran = 0.1

    # Theta_hat translational
    self.x_e_Theta_tran = np.zeros((18, 1))
    self.S_diagonal_Theta_tran = 7.5 * np.ones((18, 1))
    self.alpha_Theta_tran = 0.1

    # K_g_hat translational
    self.x_e_g_tran = np.zeros((18, 1))
    self.S_diagonal_g_tran = 60 * np.ones((18, 1))
    self.alpha_g_tran = 0.1

    # K_x_hat rotational
    self.x_e_x_rot = np.zeros((9, 1))
    self.S_diagonal_x_rot = 5.0 * np.ones((9, 1))
    self.alpha_x_rot = 0.1

    # K_r_hat rotational
    self.x_e_r_rot = np.zeros((9, 1))
    self.S_diagonal_r_rot = 0.1 * np.ones((9, 1))
    self.alpha_r_rot = 0.1

    # Theta_hat rotational
    self.x_e_Theta_rot = np.zeros((18, 1))
    self.S_diagonal_Theta_rot = 10 * np.ones((18, 1))
    self.alpha_Theta_rot = 0.1

    # K_g_hat rotational
    self.x_e_g_rot = np.zeros((9, 1))
    self.S_diagonal_g_rot = 10.0 * np.ones((9, 1))
    self.alpha_g_rot = 0.1

    # Generate S matrices from diagonal
    self.S_x_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_x_tran.flatten())
    self.S_r_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_r_tran.flatten())
    self.S_Theta_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_Theta_tran.flatten())
    self.S_g_tran = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_g_tran.flatten())
    self.S_x_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_x_rot.flatten())
    self.S_r_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_r_rot.flatten())
    self.S_Theta_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_Theta_rot.flatten())
    self.S_g_rot = ProjectionOperator.generateEllipsoidMatrixFromDiagonal(self.S_diagonal_g_rot.flatten())

    # Compute epsilon values from alpha
    self.epsilon_x_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_x_tran)
    self.epsilon_r_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_r_tran)
    self.epsilon_Theta_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_Theta_tran)
    self.epsilon_g_tran = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_g_tran)
    self.epsilon_x_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_x_rot)
    self.epsilon_r_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_r_rot)
    self.epsilon_Theta_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_Theta_rot)
    self.epsilon_g_rot = ProjectionOperator.computeEpsilonFromAlpha(self.alpha_g_rot)


