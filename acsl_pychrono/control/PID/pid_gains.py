import math
from acsl_pychrono.simulation.flight_params import FlightParams

class PIDGains:
  def __init__(self, flight_params: FlightParams):
    # General vehicle properties
    self.I_matrix_estimated = flight_params.uav.I_matrix_estimated
    self.mass_total_estimated = flight_params.uav.mass_total_estimated
    self.air_density_estimated = flight_params.uav.air_density_estimated
    self.surface_area_estimated = flight_params.uav.surface_area_estimated
    self.drag_coefficient_matrix_estimated = flight_params.uav.drag_coefficient_matrix_estimated

    # Controller's numerical Parameters config filename
    gains_config_filename = flight_params.uav.controller_config_filename
    gains_config_file = flight_params.get_controller_config(gains_config_filename, flight_params.uav.name)

    # Number of states to be integrated by RK4
    self.number_of_states = 10
    
    # **Translational** PID parameters 
    self.KP_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "KP_tran")
    self.KD_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "KD_tran")
    self.KI_tran = flight_params.get_scaled_matrix_from_config(gains_config_file, "KI_tran")

    # **Rotational** PID parameters
    self.KP_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "KP_rot")
    self.KD_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "KD_rot")
    self.KI_rot = flight_params.get_scaled_matrix_from_config(gains_config_file, "KI_rot")
    
    # ----------------------------------------------------------------
    #                   Safety Mechanism Parameters
    # ----------------------------------------------------------------
    self.use_safety_mechanism = flight_params.get_scalar_from_config(gains_config_file, "use_safety_mechanism")
    
    # Mu - sphere intersection
    self.sphereEpsilon = flight_params.get_scalar_from_config(gains_config_file, "sphereEpsilon")
    self.maximumThrust = flight_params.get_scalar_from_config(gains_config_file, "maximumThrust") # [N] 85
    
    # Mu - elliptic cone intersection
    self.EllipticConeEpsilon = flight_params.get_scalar_from_config(gains_config_file, "EllipticConeEpsilon")
    self.maximumRollAngle = math.radians(flight_params.get_scalar_from_config(gains_config_file, "maximumRollAngle_deg")) # [rad] 25
    self.maximumPitchAngle = math.radians(flight_params.get_scalar_from_config(gains_config_file, "maximumPitchAngle_deg")) # [rad] 25
    
    # Mu - plane intersection
    self.planeEpsilon = flight_params.get_scalar_from_config(gains_config_file, "planeEpsilon")
    self.alphaPlane = flight_params.get_scalar_from_config(gains_config_file, "alphaPlane") # [-] coefficient for setting the 'height' of the bottom plane. Must be >0 and <1.
    
    print(f"[INFO] Successfully loaded PID Gains")