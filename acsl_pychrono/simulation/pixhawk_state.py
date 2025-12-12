import numpy as np
from dataclasses import dataclass, field
import pychrono as chrono

@dataclass
class PixhawkState:
  # Global coordinates of the pixhawk containing both its position and rotation (quaternion)
  coord_GLOB: chrono.ChCoordsysd = field(default_factory=chrono.ChCoordsysd)
  # Global velocities of the pixhawk derived from its position and rotation (quaternion)
  coord_dt_GLOB: chrono.ChCoordsysd = field(default_factory=chrono.ChCoordsysd)
  # Global accelerations of the pixhawk derived from its position and rotation (quaternion)
  coord_dtdt_GLOB: chrono.ChCoordsysd = field(default_factory=chrono.ChCoordsysd)
  # Angular velocity of the pixhawk respect to global coordinates, expressed in global coordinates
  Wvel_GLOB: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Angular acceleration of the pixhawk respect to global coordinates, expressed in global coordinates
  Wacc_GLOB: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Rotation matrix of the pixhawk given by pychrono (computed using pixhawk quaternion expressed in glob coord)
  rotmat: chrono.ChMatrix33d = field(default_factory=chrono.ChMatrix33d)
  # Rotation matrix of the pixhawk to go from Global to Local coordinates
  # (computed using Mattia's function: fun.rotmat_fromQ_Glob_to_Loc_asChMatrix33)
  rotmat_F: chrono.ChMatrix33d = field(default_factory=chrono.ChMatrix33d)
  # Local position of the pixhawk
  pos_LOC: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Local velocities of the pixhawk
  vel_LOC: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Local accelerations of the pixhawk 
  acc_LOC: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Local Angular velocity of the pixhawk 
  Wvel_LOC: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Local Angular acceleration of the pixhawk 
  Wacc_LOC: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Global position of pixhawk obtained starting from the Local position
  # and premultiplying times the rotation matrix
  pos_LOC_to_GLOB: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Global position of pixhawk in NED convention obtained starting from the Local position
  # and premultiplying times the rotation matrix
  pos_LOC_to_GLOB_NED: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Global velocity of pixhawk obtained starting from the Local velocity
  # and premultiplying times the rotation matrix
  vel_LOC_to_GLOB: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Global velocity of pixhawk in NED convention obtained starting from the Local position
  # and premultiplying times the rotation matrix
  vel_LOC_to_GLOB_NED: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Global angular velocity of pixhawk obtained starting from the Local position
  # and premultiplying times the rotation matrix
  Wvel_LOC_to_GLOB: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Global angular velocity of pixhawk in NED convention obtained starting from the Local position
  # and premultiplying times the rotation matrix
  Wvel_LOC_to_GLOB_NED: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))
  # Pixhawk quaternion with y and z components flipped
  quat_fixed: chrono.ChQuaterniond = field(default_factory=lambda: chrono.ChQuaterniond(1, 0, 0, 0))
  # 321 sequence of euler angle (roll, pitch, yaw)
  euler321: chrono.ChVector3d = field(default_factory=lambda: chrono.ChVector3d(0, 0, 0))

@dataclass
class VehicleState:
  roll: float = field(default_factory=lambda: 0.0)
  pitch: float = field(default_factory=lambda: 0.0)
  yaw: float = field(default_factory=lambda: 0.0)
  position_global: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
  velocity_global: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
  angular_velocity_local: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))