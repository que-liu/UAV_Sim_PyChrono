
clear all
close all
clc

% C = readcell('data_MRACwithBASELINE/value_1.csv');

M = readmatrix('data_MRACwithBASELINE/value_1.csv'); M=M(4:end,:);

MRAC.data.time = M(:,1);
MRAC.data.simulation_time = M(:,2);
MRAC.data.translational_position_in_I = M(:,3:5);
MRAC.data.translational_velocity_in_I = M(:,6:8);
MRAC.data.roll = M(:,9);
MRAC.data.pitch = M(:,10);
MRAC.data.yaw = M(:,11);
MRAC.data.angular_velocity = M(:,12:14);
MRAC.data.x_ref_tran = M(:,15:20);
MRAC.data.roll_ref = M(:,21);
MRAC.data.pitch_ref = M(:,22);
MRAC.data.yaw_ref = M(:,23);
MRAC.data.roll_ref_dot = M(:,24);
MRAC.data.pitch_ref_dot = M(:,25);
MRAC.data.yaw_ref_dot = M(:,26);
MRAC.data.roll_ref_ddot = M(:,27);
MRAC.data.pitch_ref_ddot = M(:,28);
MRAC.data.yaw_ref_ddot = M(:,29);
MRAC.data.omega_ref = M(:,30:32);
MRAC.data.translational_position_in_I_user = M(:,33:35);
MRAC.data.translational_velocity_in_I_user = M(:,36:38);
MRAC.data.translational_acceleration_in_I_user = M(:,39:41);
MRAC.data.mu_x = M(:,42);
MRAC.data.mu_y = M(:,43);
MRAC.data.mu_z = M(:,44);
MRAC.data.u1 = M(:,45);
MRAC.data.u2 = M(:,46);
MRAC.data.u3 = M(:,47);
MRAC.data.u4 = M(:,48);
MRAC.data.thrust = M(:,49:56);
MRAC.data.mu_baseline_tran = M(:,57:59);
MRAC.data.mu_adaptive_tran = M(:,60:62);
MRAC.data.mu_PD_baseline_tran = M(:,63:65);
MRAC.data.Moment_baseline = M(:,66:68);
MRAC.data.Moment_adaptive = M(:,69:71);
MRAC.data.Moment_baseline_PI = M(:,72:74);

MRAC.data.yaw_ref = rem((MRAC.data.yaw_ref + pi), 2*pi) - pi;

MRAC.traj_track_error = [MRAC.data.translational_position_in_I MRAC.data.translational_velocity_in_I]...
    - [MRAC.data.translational_position_in_I_user MRAC.data.translational_velocity_in_I_user];

MRAC.traj_track_error_norm = vecnorm(MRAC.traj_track_error')';

MRAC.traj_track_error_L2norm = sqrt(cumtrapz(MRAC.data.time, MRAC.traj_track_error_norm.^2));

MRAC.pos_track_error = [MRAC.data.translational_position_in_I]...
    - [MRAC.data.translational_position_in_I_user];

MRAC.pos_track_error_norm = vecnorm(MRAC.pos_track_error')';

%%
M = readmatrix('data_RobustMRACwithBASELINE/value_1.csv'); M=M(4:end,:);

RobMRAC.data.time = M(:,1);
RobMRAC.data.simulation_time = M(:,2);
RobMRAC.data.translational_position_in_I = M(:,3:5);
RobMRAC.data.translational_velocity_in_I = M(:,6:8);
RobMRAC.data.roll = M(:,9);
RobMRAC.data.pitch = M(:,10);
RobMRAC.data.yaw = M(:,11);
RobMRAC.data.angular_velocity = M(:,12:14);
RobMRAC.data.x_ref_tran = M(:,15:20);
RobMRAC.data.roll_ref = M(:,21);
RobMRAC.data.pitch_ref = M(:,22);
RobMRAC.data.yaw_ref = M(:,23);
RobMRAC.data.roll_ref_dot = M(:,24);
RobMRAC.data.pitch_ref_dot = M(:,25);
RobMRAC.data.yaw_ref_dot = M(:,26);
RobMRAC.data.roll_ref_ddot = M(:,27);
RobMRAC.data.pitch_ref_ddot = M(:,28);
RobMRAC.data.yaw_ref_ddot = M(:,29);
RobMRAC.data.omega_ref = M(:,30:32);
RobMRAC.data.translational_position_in_I_user = M(:,33:35);
RobMRAC.data.translational_velocity_in_I_user = M(:,36:38);
RobMRAC.data.translational_acceleration_in_I_user = M(:,39:41);
RobMRAC.data.mu_x = M(:,42);
RobMRAC.data.mu_y = M(:,43);
RobMRAC.data.mu_z = M(:,44);
RobMRAC.data.u1 = M(:,45);
RobMRAC.data.u2 = M(:,46);
RobMRAC.data.u3 = M(:,47);
RobMRAC.data.u4 = M(:,48);
RobMRAC.data.thrust = M(:,49:56);
RobMRAC.data.mu_baseline_tran = M(:,57:59);
RobMRAC.data.mu_adaptive_tran = M(:,60:62);
RobMRAC.data.mu_PD_baseline_tran = M(:,63:65);
RobMRAC.data.Moment_baseline = M(:,66:68);
RobMRAC.data.Moment_adaptive = M(:,69:71);
RobMRAC.data.Moment_baseline_PI = M(:,72:74);

RobMRAC.data.yaw_ref = rem((RobMRAC.data.yaw_ref + pi), 2*pi) - pi;

RobMRAC.traj_track_error = [RobMRAC.data.translational_position_in_I RobMRAC.data.translational_velocity_in_I]...
    - [RobMRAC.data.translational_position_in_I_user RobMRAC.data.translational_velocity_in_I_user];

RobMRAC.traj_track_error_norm = vecnorm(RobMRAC.traj_track_error')';

RobMRAC.traj_track_error_L2norm = sqrt(cumtrapz(RobMRAC.data.time, RobMRAC.traj_track_error_norm.^2));

RobMRAC.pos_track_error = [RobMRAC.data.translational_position_in_I]...
    - [RobMRAC.data.translational_position_in_I_user];

RobMRAC.pos_track_error_norm = vecnorm(RobMRAC.pos_track_error')';

%%
M = readmatrix('data_TwoLayerMRACwithBASELINE/value_1.csv'); M=M(4:end,:);

TwoLayerMRAC.data.time = M(:,1);
TwoLayerMRAC.data.simulation_time = M(:,2);
TwoLayerMRAC.data.translational_position_in_I = M(:,3:5);
TwoLayerMRAC.data.translational_velocity_in_I = M(:,6:8);
TwoLayerMRAC.data.roll = M(:,9);
TwoLayerMRAC.data.pitch = M(:,10);
TwoLayerMRAC.data.yaw = M(:,11);
TwoLayerMRAC.data.angular_velocity = M(:,12:14);
TwoLayerMRAC.data.x_ref_tran = M(:,15:20);
TwoLayerMRAC.data.roll_ref = M(:,21);
TwoLayerMRAC.data.pitch_ref = M(:,22);
TwoLayerMRAC.data.yaw_ref = M(:,23);
TwoLayerMRAC.data.roll_ref_dot = M(:,24);
TwoLayerMRAC.data.pitch_ref_dot = M(:,25);
TwoLayerMRAC.data.yaw_ref_dot = M(:,26);
TwoLayerMRAC.data.roll_ref_ddot = M(:,27);
TwoLayerMRAC.data.pitch_ref_ddot = M(:,28);
TwoLayerMRAC.data.yaw_ref_ddot = M(:,29);
TwoLayerMRAC.data.omega_ref = M(:,30:32);
TwoLayerMRAC.data.translational_position_in_I_user = M(:,33:35);
TwoLayerMRAC.data.translational_velocity_in_I_user = M(:,36:38);
TwoLayerMRAC.data.translational_acceleration_in_I_user = M(:,39:41);
TwoLayerMRAC.data.mu_x = M(:,42);
TwoLayerMRAC.data.mu_y = M(:,43);
TwoLayerMRAC.data.mu_z = M(:,44);
TwoLayerMRAC.data.u1 = M(:,45);
TwoLayerMRAC.data.u2 = M(:,46);
TwoLayerMRAC.data.u3 = M(:,47);
TwoLayerMRAC.data.u4 = M(:,48);
TwoLayerMRAC.data.thrust = M(:,49:56);
TwoLayerMRAC.data.mu_baseline_tran = M(:,57:59);
TwoLayerMRAC.data.mu_adaptive_tran = M(:,60:62);
TwoLayerMRAC.data.mu_PD_baseline_tran = M(:,63:65);
TwoLayerMRAC.data.Moment_baseline = M(:,66:68);
TwoLayerMRAC.data.Moment_adaptive = M(:,69:71);
TwoLayerMRAC.data.Moment_baseline_PI = M(:,72:74);

TwoLayerMRAC.data.yaw_ref = rem((TwoLayerMRAC.data.yaw_ref + pi), 2*pi) - pi;

TwoLayerMRAC.traj_track_error = [TwoLayerMRAC.data.translational_position_in_I TwoLayerMRAC.data.translational_velocity_in_I]...
    - [TwoLayerMRAC.data.translational_position_in_I_user TwoLayerMRAC.data.translational_velocity_in_I_user];

TwoLayerMRAC.traj_track_error_norm = vecnorm(TwoLayerMRAC.traj_track_error')';

TwoLayerMRAC.traj_track_error_L2norm = sqrt(cumtrapz(TwoLayerMRAC.data.time, TwoLayerMRAC.traj_track_error_norm.^2));


TwoLayerMRAC.pos_track_error = [TwoLayerMRAC.data.translational_position_in_I]...
    - [TwoLayerMRAC.data.translational_position_in_I_user];

TwoLayerMRAC.pos_track_error_norm = vecnorm(TwoLayerMRAC.pos_track_error')';


%%

M = readmatrix('data_RobustTwoLayerMRACwithBASELINE/value_1.csv'); M=M(4:end,:);

RobTwoLayerMRAC.data.time = M(:,1);
RobTwoLayerMRAC.data.simulation_time = M(:,2);
RobTwoLayerMRAC.data.translational_position_in_I = M(:,3:5);
RobTwoLayerMRAC.data.translational_velocity_in_I = M(:,6:8);
RobTwoLayerMRAC.data.roll = M(:,9);
RobTwoLayerMRAC.data.pitch = M(:,10);
RobTwoLayerMRAC.data.yaw = M(:,11);
RobTwoLayerMRAC.data.angular_velocity = M(:,12:14);
RobTwoLayerMRAC.data.x_ref_tran = M(:,15:20);
RobTwoLayerMRAC.data.roll_ref = M(:,21);
RobTwoLayerMRAC.data.pitch_ref = M(:,22);
RobTwoLayerMRAC.data.yaw_ref = M(:,23);
RobTwoLayerMRAC.data.roll_ref_dot = M(:,24);
RobTwoLayerMRAC.data.pitch_ref_dot = M(:,25);
RobTwoLayerMRAC.data.yaw_ref_dot = M(:,26);
RobTwoLayerMRAC.data.roll_ref_ddot = M(:,27);
RobTwoLayerMRAC.data.pitch_ref_ddot = M(:,28);
RobTwoLayerMRAC.data.yaw_ref_ddot = M(:,29);
RobTwoLayerMRAC.data.omega_ref = M(:,30:32);
RobTwoLayerMRAC.data.translational_position_in_I_user = M(:,33:35);
RobTwoLayerMRAC.data.translational_velocity_in_I_user = M(:,36:38);
RobTwoLayerMRAC.data.translational_acceleration_in_I_user = M(:,39:41);
RobTwoLayerMRAC.data.mu_x = M(:,42);
RobTwoLayerMRAC.data.mu_y = M(:,43);
RobTwoLayerMRAC.data.mu_z = M(:,44);
RobTwoLayerMRAC.data.u1 = M(:,45);
RobTwoLayerMRAC.data.u2 = M(:,46);
RobTwoLayerMRAC.data.u3 = M(:,47);
RobTwoLayerMRAC.data.u4 = M(:,48);
RobTwoLayerMRAC.data.thrust = M(:,49:56);
RobTwoLayerMRAC.data.mu_baseline_tran = M(:,57:59);
RobTwoLayerMRAC.data.mu_adaptive_tran = M(:,60:62);
RobTwoLayerMRAC.data.mu_PD_baseline_tran = M(:,63:65);
RobTwoLayerMRAC.data.Moment_baseline = M(:,66:68);
RobTwoLayerMRAC.data.Moment_adaptive = M(:,69:71);
RobTwoLayerMRAC.data.Moment_baseline_PI = M(:,72:74);

RobTwoLayerMRAC.data.yaw_ref = rem((RobTwoLayerMRAC.data.yaw_ref + pi), 2*pi) - pi;

RobTwoLayerMRAC.traj_track_error = [RobTwoLayerMRAC.data.translational_position_in_I RobTwoLayerMRAC.data.translational_velocity_in_I]...
    - [RobTwoLayerMRAC.data.translational_position_in_I_user RobTwoLayerMRAC.data.translational_velocity_in_I_user];

RobTwoLayerMRAC.traj_track_error_norm = vecnorm(RobTwoLayerMRAC.traj_track_error')';

RobTwoLayerMRAC.traj_track_error_L2norm = sqrt(cumtrapz(RobTwoLayerMRAC.data.time, RobTwoLayerMRAC.traj_track_error_norm.^2));

RobTwoLayerMRAC.pos_track_error = [RobTwoLayerMRAC.data.translational_position_in_I]...
    - [RobTwoLayerMRAC.data.translational_position_in_I_user];

RobTwoLayerMRAC.pos_track_error_norm = vecnorm(RobTwoLayerMRAC.pos_track_error')';

%%
M = readmatrix('data_PID_payload_aware/value_1.csv'); M=M(4:end,:);

PID.PayAwa.data.time = M(:,1);
PID.PayAwa.data.simulation_time = M(:,2);
PID.PayAwa.data.translational_position_in_I = M(:,3:5);
PID.PayAwa.data.translational_velocity_in_I = M(:,6:8);
PID.PayAwa.data.roll = M(:,9);
PID.PayAwa.data.pitch = M(:,10);
PID.PayAwa.data.yaw = M(:,11);
PID.PayAwa.data.angular_velocity = M(:,12:14);
PID.PayAwa.data.roll_ref = M(:,15);
PID.PayAwa.data.pitch_ref = M(:,16);
PID.PayAwa.data.yaw_ref = M(:,17);
PID.PayAwa.data.roll_ref_dot = M(:,18);
PID.PayAwa.data.pitch_ref_dot = M(:,19);
PID.PayAwa.data.yaw_ref_dot = M(:,20);
PID.PayAwa.data.roll_ref_ddot = M(:,21);
PID.PayAwa.data.pitch_ref_ddot = M(:,22);
PID.PayAwa.data.yaw_ref_ddot = M(:,23);
PID.PayAwa.data.translational_position_in_I_user = M(:,24:26);
PID.PayAwa.data.translational_velocity_in_I_user = M(:,27:29);
PID.PayAwa.data.translational_acceleration_in_I_user = M(:,30:32);
PID.PayAwa.data.mu_x = M(:,33);
PID.PayAwa.data.mu_y = M(:,34);
PID.PayAwa.data.mu_z = M(:,35);
PID.PayAwa.data.u1 = M(:,36);
PID.PayAwa.data.u2 = M(:,37);
PID.PayAwa.data.u3 = M(:,38);
PID.PayAwa.data.u4 = M(:,39);
PID.PayAwa.data.thrust = M(:,40:47);

PID.PayAwa.data.yaw_ref = rem((PID.PayAwa.data.yaw_ref + pi), 2*pi) - pi;

PID.PayAwa.traj_track_error = [PID.PayAwa.data.translational_position_in_I PID.PayAwa.data.translational_velocity_in_I]...
    - [PID.PayAwa.data.translational_position_in_I_user PID.PayAwa.data.translational_velocity_in_I_user];

PID.PayAwa.traj_track_error_norm = vecnorm(PID.PayAwa.traj_track_error')';

PID.PayAwa.traj_track_error_L2norm = sqrt(cumtrapz(PID.PayAwa.data.time, PID.PayAwa.traj_track_error_norm.^2));

PID.PayAwa.pos_track_error = [PID.PayAwa.data.translational_position_in_I]...
    - [PID.PayAwa.data.translational_position_in_I_user];

PID.PayAwa.pos_track_error_norm = vecnorm(PID.PayAwa.pos_track_error')';


%%
M = readmatrix('data_PID_payload_unaware/value_1.csv'); M=M(4:end,:);

PID.PayUnawa.data.time = M(:,1);
PID.PayUnawa.data.simulation_time = M(:,2);
PID.PayUnawa.data.translational_position_in_I = M(:,3:5);
PID.PayUnawa.data.translational_velocity_in_I = M(:,6:8);
PID.PayUnawa.data.roll = M(:,9);
PID.PayUnawa.data.pitch = M(:,10);
PID.PayUnawa.data.yaw = M(:,11);
PID.PayUnawa.data.angular_velocity = M(:,12:14);
PID.PayUnawa.data.roll_ref = M(:,15);
PID.PayUnawa.data.pitch_ref = M(:,16);
PID.PayUnawa.data.yaw_ref = M(:,17);
PID.PayUnawa.data.roll_ref_dot = M(:,18);
PID.PayUnawa.data.pitch_ref_dot = M(:,19);
PID.PayUnawa.data.yaw_ref_dot = M(:,20);
PID.PayUnawa.data.roll_ref_ddot = M(:,21);
PID.PayUnawa.data.pitch_ref_ddot = M(:,22);
PID.PayUnawa.data.yaw_ref_ddot = M(:,23);
PID.PayUnawa.data.translational_position_in_I_user = M(:,24:26);
PID.PayUnawa.data.translational_velocity_in_I_user = M(:,27:29);
PID.PayUnawa.data.translational_acceleration_in_I_user = M(:,30:32);
PID.PayUnawa.data.mu_x = M(:,33);
PID.PayUnawa.data.mu_y = M(:,34);
PID.PayUnawa.data.mu_z = M(:,35);
PID.PayUnawa.data.u1 = M(:,36);
PID.PayUnawa.data.u2 = M(:,37);
PID.PayUnawa.data.u3 = M(:,38);
PID.PayUnawa.data.u4 = M(:,39);
PID.PayUnawa.data.thrust = M(:,40:47);

PID.PayUnawa.data.yaw_ref = rem((PID.PayUnawa.data.yaw_ref + pi), 2*pi) - pi;

PID.PayUnawa.traj_track_error = [PID.PayUnawa.data.translational_position_in_I PID.PayUnawa.data.translational_velocity_in_I]...
    - [PID.PayUnawa.data.translational_position_in_I_user PID.PayUnawa.data.translational_velocity_in_I_user];

PID.PayUnawa.traj_track_error_norm = vecnorm(PID.PayUnawa.traj_track_error')';

PID.PayUnawa.traj_track_error_L2norm = sqrt(cumtrapz(PID.PayUnawa.data.time, PID.PayUnawa.traj_track_error_norm.^2));

PID.PayUnawa.pos_track_error = [PID.PayUnawa.data.translational_position_in_I]...
    - [PID.PayUnawa.data.translational_position_in_I_user];

PID.PayUnawa.pos_track_error_norm = vecnorm(PID.PayUnawa.pos_track_error')';

%%

M = readmatrix('data_HybridMRACwithBASELINE/DATA_HybridMRACwithBASELINE.csv'); M=M'; M=M(2:end,1:74);

HybridMRAC.data.time = M(:,1);
HybridMRAC.data.simulation_time = M(:,2);
HybridMRAC.data.translational_position_in_I = M(:,3:5);
HybridMRAC.data.translational_velocity_in_I = M(:,6:8);
HybridMRAC.data.roll = M(:,9);
HybridMRAC.data.pitch = M(:,10);
HybridMRAC.data.yaw = M(:,11);
HybridMRAC.data.angular_velocity = M(:,12:14);
HybridMRAC.data.x_ref_tran = M(:,15:20);
HybridMRAC.data.roll_ref = M(:,21);
HybridMRAC.data.pitch_ref = M(:,22);
HybridMRAC.data.yaw_ref = M(:,23);
HybridMRAC.data.roll_ref_dot = M(:,24);
HybridMRAC.data.pitch_ref_dot = M(:,25);
HybridMRAC.data.yaw_ref_dot = M(:,26);
HybridMRAC.data.roll_ref_ddot = M(:,27);
HybridMRAC.data.pitch_ref_ddot = M(:,28);
HybridMRAC.data.yaw_ref_ddot = M(:,29);
HybridMRAC.data.omega_ref = M(:,30:32);
HybridMRAC.data.translational_position_in_I_user = M(:,33:35);
HybridMRAC.data.translational_velocity_in_I_user = M(:,36:38);
HybridMRAC.data.translational_acceleration_in_I_user = M(:,39:41);
HybridMRAC.data.mu_x = M(:,42);
HybridMRAC.data.mu_y = M(:,43);
HybridMRAC.data.mu_z = M(:,44);
HybridMRAC.data.u1 = M(:,45);
HybridMRAC.data.u2 = M(:,46);
HybridMRAC.data.u3 = M(:,47);
HybridMRAC.data.u4 = M(:,48);
HybridMRAC.data.thrust = M(:,49:56);
HybridMRAC.data.mu_baseline_tran = M(:,57:59);
HybridMRAC.data.mu_adaptive_tran = M(:,60:62);
HybridMRAC.data.mu_PD_baseline_tran = M(:,63:65);
HybridMRAC.data.Moment_baseline = M(:,66:68);
HybridMRAC.data.Moment_adaptive = M(:,69:71);
HybridMRAC.data.Moment_baseline_PI = M(:,72:74);

HybridMRAC.data.yaw_ref = rem((HybridMRAC.data.yaw_ref + pi), 2*pi) - pi;

HybridMRAC.traj_track_error = [HybridMRAC.data.translational_position_in_I HybridMRAC.data.translational_velocity_in_I]...
    - [HybridMRAC.data.translational_position_in_I_user HybridMRAC.data.translational_velocity_in_I_user];

HybridMRAC.traj_track_error_norm = vecnorm(HybridMRAC.traj_track_error')';

HybridMRAC.traj_track_error_L2norm = sqrt(cumtrapz(HybridMRAC.data.time, HybridMRAC.traj_track_error_norm.^2));

HybridMRAC.pos_track_error = [HybridMRAC.data.translational_position_in_I]...
    - [HybridMRAC.data.translational_position_in_I_user];

HybridMRAC.pos_track_error_norm = vecnorm(HybridMRAC.pos_track_error')';




