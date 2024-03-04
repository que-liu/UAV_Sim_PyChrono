%% minjerkpolytraj 3D WE WANT THIS

clear all
close all
clc


% traj.wpts = [0  5  20  20   0  0;
%              0  0   0   5   5  0;
%              0 -1  -1  -1  -1 -1];

traj.wpts = [0  5  20  20   0  0;
             0  0   0   5   5  0;
             0 -1  -1  -1  -1 -1];

traj.alphaFactor = 2;

% traj.tpts = traj.alphaFactor*[0 6 10 16 22 32]';
traj.tpts = traj.alphaFactor*[0 4 8 14 20 30]';

traj.numsamples = 200;

[traj.q,traj.qd,traj.qdd,traj.qddd,traj.pp,traj.timepoints,traj.tsamples] = minjerkpolytraj(traj.wpts, traj.tpts, traj.numsamples);

set(figure,'Color','white')
plot(traj.tsamples,traj.q)
hold on
plot(traj.timepoints,traj.wpts,'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions','Z-positions')
hold off

set(figure,'Color','white')
plot(traj.q(1,:),traj.q(2,:),'.b',traj.wpts(1,:),traj.wpts(2,:),'or')
xlabel('X')
ylabel('Y')
ax = gca;
ax.YDir = 'reverse';
axis equal

set(figure,'Color','white')
plot3(traj.q(1,:),traj.q(2,:),traj.q(3,:),'.b',traj.wpts(1,:),traj.wpts(2,:),traj.wpts(3,:),'or')
xlabel('X')
ylabel('Y')
zlabel('Z')
ax = gca;
ax.YDir = 'reverse';
ax.ZDir = 'reverse';
axis equal

set(figure,'Color','white')
plot(traj.tsamples,traj.qd)
xlabel('t')
ylabel('Velocity')
legend('X-velocity','Y-velocity','Z-velocity')

set(figure,'Color','white')
plot(traj.tsamples,traj.qdd)
xlabel('t')
ylabel('Accelerations')
legend('X-acceleration','Y-acceleration','Z-acceleration')

set(figure,'Color','white')
plot(traj.tsamples,traj.qddd)
xlabel('t')
ylabel('Jerks')
legend('X-jerk','Y-jerk','Z-jerk')

% Evaluating the piecewise polynomial
poly_coeff.p1 = traj.pp.coefs(1,:);
poly_coeff.p2 = traj.pp.coefs(4,:);
poly_coeff.p3 = traj.pp.coefs(7,:);
poly_coeff.p4 = traj.pp.coefs(10,:);
poly_coeff.p5 = traj.pp.coefs(13,:);

traj.step = 0.01;

poly_t.p1 = (traj.tpts(1):traj.step:traj.tpts(2))-traj.tpts(1);
poly_t.p2 = (traj.tpts(2):traj.step:traj.tpts(3))-traj.tpts(2);
poly_t.p3 = (traj.tpts(3):traj.step:traj.tpts(4))-traj.tpts(3);
poly_t.p4 = (traj.tpts(4):traj.step:traj.tpts(5))-traj.tpts(4);
poly_t.p5 = (traj.tpts(5):traj.step:traj.tpts(6))-traj.tpts(5);

y.p1 = polyval(poly_coeff.p1,poly_t.p1);
y.p2 = polyval(poly_coeff.p2,poly_t.p2);
y.p3 = polyval(poly_coeff.p3,poly_t.p3);
y.p4 = polyval(poly_coeff.p4,poly_t.p4);
y.p5 = polyval(poly_coeff.p5,poly_t.p5);

set(figure,'Color','white')
plot(traj.tsamples,traj.q(1,:),'-k','LineWidth',0.75)
hold on
plot(poly_t.p1 + traj.tpts(1),y.p1,'--b','LineWidth',2)
plot(poly_t.p2 + traj.tpts(2),y.p2,'LineWidth',2,'LineStyle','--')
plot(poly_t.p3 + traj.tpts(3),y.p3,'LineWidth',2,'LineStyle','--')
plot(poly_t.p4 + traj.tpts(4),y.p4,'LineWidth',2,'LineStyle','--')
plot(poly_t.p5 + traj.tpts(5),y.p5,'LineWidth',2,'LineStyle','--')
xlabel('t')
ylabel('X-Position')
legend('MATLAB function','Custom function','Custom function','Custom function','Custom function')
hold off

% Computing derivative (velocities) from (position) polynomial coefficients
d_poly_coeff.p1 = polyder(traj.pp.coefs(1,:));
d_poly_coeff.p2 = polyder(traj.pp.coefs(4,:));
d_poly_coeff.p3 = polyder(traj.pp.coefs(7,:));
d_poly_coeff.p4 = polyder(traj.pp.coefs(10,:));
d_poly_coeff.p5 = polyder(traj.pp.coefs(13,:));

dy.p1 = polyval(d_poly_coeff.p1,poly_t.p1);
dy.p2 = polyval(d_poly_coeff.p2,poly_t.p2);
dy.p3 = polyval(d_poly_coeff.p3,poly_t.p3);
dy.p4 = polyval(d_poly_coeff.p4,poly_t.p4);
dy.p5 = polyval(d_poly_coeff.p5,poly_t.p5);

set(figure,'Color','white')
plot(traj.tsamples,traj.qd(1,:),'-k','LineWidth',0.75)
hold on
plot(poly_t.p1 + traj.tpts(1),dy.p1,'--b','LineWidth',2)
plot(poly_t.p2 + traj.tpts(2),dy.p2,'LineWidth',2,'LineStyle','--')
plot(poly_t.p3 + traj.tpts(3),dy.p3,'LineWidth',2,'LineStyle','--')
plot(poly_t.p4 + traj.tpts(4),dy.p4,'LineWidth',2,'LineStyle','--')
plot(poly_t.p5 + traj.tpts(5),dy.p5,'LineWidth',2,'LineStyle','--')
xlabel('t')
ylabel('X-Velocity')
legend('MATLAB function','Custom function','Custom function','Custom function','Custom function')
hold off

% Computing derivative (acceleration) from (velocity) polynomial coefficients
dd_poly_coeff.p1 = polyder(d_poly_coeff.p1);
dd_poly_coeff.p2 = polyder(d_poly_coeff.p2);
dd_poly_coeff.p3 = polyder(d_poly_coeff.p3);
dd_poly_coeff.p4 = polyder(d_poly_coeff.p4);
dd_poly_coeff.p5 = polyder(d_poly_coeff.p5);

ddy.p1 = polyval(dd_poly_coeff.p1,poly_t.p1);
ddy.p2 = polyval(dd_poly_coeff.p2,poly_t.p2);
ddy.p3 = polyval(dd_poly_coeff.p3,poly_t.p3);
ddy.p4 = polyval(dd_poly_coeff.p4,poly_t.p4);
ddy.p5 = polyval(dd_poly_coeff.p5,poly_t.p5);

set(figure,'Color','white')
plot(traj.tsamples,traj.qdd(1,:),'-k','LineWidth',0.75)
hold on
plot(poly_t.p1 + traj.tpts(1),ddy.p1,'--b','LineWidth',2)
plot(poly_t.p2 + traj.tpts(2),ddy.p2,'LineWidth',2,'LineStyle','--')
plot(poly_t.p3 + traj.tpts(3),ddy.p3,'LineWidth',2,'LineStyle','--')
plot(poly_t.p4 + traj.tpts(4),ddy.p4,'LineWidth',2,'LineStyle','--')
plot(poly_t.p5 + traj.tpts(5),ddy.p5,'LineWidth',2,'LineStyle','--')
xlabel('t')
ylabel('X-Acceleration')
legend('MATLAB function','Custom function','Custom function','Custom function','Custom function')
hold off

% Computing yaw reference 
ypos_poly_coeff.p1 = traj.pp.coefs(2,:);
ypos_poly_coeff.p2 = traj.pp.coefs(5,:);
ypos_poly_coeff.p3 = traj.pp.coefs(8,:);
ypos_poly_coeff.p4 = traj.pp.coefs(11,:);
ypos_poly_coeff.p5 = traj.pp.coefs(14,:);

dypos_poly_coeff.p1 = polyder(traj.pp.coefs(2,:));
dypos_poly_coeff.p2 = polyder(traj.pp.coefs(5,:));
dypos_poly_coeff.p3 = polyder(traj.pp.coefs(8,:));
dypos_poly_coeff.p4 = polyder(traj.pp.coefs(11,:));
dypos_poly_coeff.p5 = polyder(traj.pp.coefs(14,:));

yaw_ref.p1 = atan2(polyval(dypos_poly_coeff.p1,poly_t.p1), polyval(d_poly_coeff.p1,poly_t.p1));
yaw_ref.p2 = atan2(polyval(dypos_poly_coeff.p2,poly_t.p2), polyval(d_poly_coeff.p2,poly_t.p2));
yaw_ref.p3 = atan2(polyval(dypos_poly_coeff.p3,poly_t.p3), polyval(d_poly_coeff.p3,poly_t.p3));
yaw_ref.p4 = atan2(polyval(dypos_poly_coeff.p4,poly_t.p4), polyval(d_poly_coeff.p4,poly_t.p4));
yaw_ref.p5 = atan2(polyval(dypos_poly_coeff.p5,poly_t.p5), polyval(d_poly_coeff.p5,poly_t.p5));

set(figure,'Color','white')
plot(poly_t.p1 + traj.tpts(1),yaw_ref.p1,'--b','LineWidth',2)
hold on
plot(poly_t.p2 + traj.tpts(2),yaw_ref.p2,'LineWidth',2,'LineStyle','--')
plot(poly_t.p3 + traj.tpts(3),yaw_ref.p3,'LineWidth',2,'LineStyle','--')
plot(poly_t.p4 + traj.tpts(4),yaw_ref.p4,'LineWidth',2,'LineStyle','--')
plot(poly_t.p5 + traj.tpts(5),yaw_ref.p5,'LineWidth',2,'LineStyle','--')
xlabel('t')
ylabel('Yaw-reference [rad]')
legend('Custom function','Custom function','Custom function','Custom function')
hold off

set(figure,'Color','white')
plot(poly_t.p1 + traj.tpts(1),rad2deg(yaw_ref.p1),'--b','LineWidth',2)
hold on
plot(poly_t.p2 + traj.tpts(2),rad2deg(yaw_ref.p2),'LineWidth',2,'LineStyle','--')
plot(poly_t.p3 + traj.tpts(3),rad2deg(yaw_ref.p3),'LineWidth',2,'LineStyle','--')
plot(poly_t.p4 + traj.tpts(4),rad2deg(yaw_ref.p4),'LineWidth',2,'LineStyle','--')
plot(poly_t.p5 + traj.tpts(5),rad2deg(yaw_ref.p5),'LineWidth',2,'LineStyle','--')
xlabel('t')
ylabel('Yaw-reference [deg]')
legend('Custom function','Custom function','Custom function','Custom function')
hold off

%% Frenet-Serret reference frame

Delta_time = 0.01;
% time = 5.5;
time = 3;
time_prev = time - Delta_time;

theta = deg2rad(37);
R_thetax = [1          0           0;
            0 cos(theta) -sin(theta);
            0 sin(theta)  cos(theta)];

R_init = [1,0,0;
          0,1,0;
          0,0,1];
quat_init = rotm2quat(R_init);

trajectory_x = traj.pp.coefs(1,:);
trajectory_y = traj.pp.coefs(2,:);
trajectory_z = traj.pp.coefs(3,:);

% trajectory_x = traj.pp.coefs(4,:);
% trajectory_y = traj.pp.coefs(5,:);
% trajectory_z = traj.pp.coefs(6,:);


R_fs = FrenetSerretFrame(time, trajectory_x, trajectory_y, trajectory_z);
quat_fs = rotm2quat(R_fs);
pos_fs = ComputePosition(time, trajectory_x, trajectory_y, trajectory_z);

quat_fs_thetax = rotm2quat(R_fs*R_thetax);

R_fs_prev = FrenetSerretFrame(time_prev, trajectory_x, trajectory_y, trajectory_z);

R_rm = RotationMinimizingFrame(time, ...
                               trajectory_x, ...
                               trajectory_y, ...
                               trajectory_z, ...
                               time_prev, ...
                               R_fs_prev); % R_fs_prev - R_init
% quat_rm = rotm2quat(R_rm);
quat_rm = rotm2quat(R_rm);


set(figure,'Color','white')
plot3(traj.q(1,:),traj.q(2,:),traj.q(3,:),'.b',traj.wpts(1,:),traj.wpts(2,:),traj.wpts(3,:),'or')
xlabel('X')
ylabel('Y')
zlabel('Z')
ax = gca;
ax.YDir = 'reverse';
ax.ZDir = 'reverse';
hold on
ax1 = plotTransforms(pos_fs',quat_fs,"FrameSize",2,"FrameAxisLabels",'on');
% ax2 = plotTransforms(pos_fs',quat_rm,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor","black");
ax2 = plotTransforms(pos_fs',quat_fs_thetax,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor","black");
% ax3 = plotTransforms(pos_fs',quat_init,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor","blue");
axis equal

%% Simulate trajectory
% 
% time_sim = poly_t.p1;
% trajectory_x = traj.pp.coefs(1,:);
% trajectory_y = traj.pp.coefs(2,:);
% trajectory_z = traj.pp.coefs(3,:);

% time_sim = poly_t.p2;
% trajectory_x = traj.pp.coefs(4,:);
% trajectory_y = traj.pp.coefs(5,:);
% trajectory_z = traj.pp.coefs(6,:);

time_sim = poly_t.p3;
trajectory_x = traj.pp.coefs(7,:);
trajectory_y = traj.pp.coefs(8,:);
trajectory_z = traj.pp.coefs(9,:);

% time_sim = poly_t.p4;
% trajectory_x = traj.pp.coefs(10,:);
% trajectory_y = traj.pp.coefs(11,:);
% trajectory_z = traj.pp.coefs(12,:);

% time_sim = poly_t.p5;
% trajectory_x = traj.pp.coefs(13,:);
% trajectory_y = traj.pp.coefs(14,:);
% trajectory_z = traj.pp.coefs(15,:);

set(figure,'Color','white')
h = plot3(traj.q(1,:),traj.q(2,:),traj.q(3,:),'.b',traj.wpts(1,:),traj.wpts(2,:),traj.wpts(3,:),'or');
ax = gca;
xlabel('X')
ylabel('Y')
zlabel('Z')
% title(ax,'Time: 0 s')
title(ax, "Time: " + num2str(time_sim(1)) + " s")
ax.YDir = 'reverse';
ax.ZDir = 'reverse';
axis equal
hold on

InitialValueForLoop = 2;
k = InitialValueForLoop;
for i = InitialValueForLoop:length(time_sim)
    title(ax, "Time: " + num2str(time_sim(i)) + " s")

    pos_fs = ComputePosition(time_sim(i), trajectory_x, trajectory_y, trajectory_z);

    R_fs = FrenetSerretFrame(time_sim(i), trajectory_x, trajectory_y, trajectory_z);
    quat_fs = rotm2quat(R_fs);
    eulZYX_fs(i,:) = rad2deg(rotm2eul(R_fs,"ZYX"));

    quat_fs_thetax = rotm2quat(R_fs*R_thetax);
    eulZYX_fs_thetax(i,:) = rad2deg(rotm2eul(R_fs*R_thetax,"ZYX"));

    % R_fs_check = GenerateRotMat(deg2rad(eulZYX_fs(i,3)), deg2rad(eulZYX_fs(i,2)), deg2rad(eulZYX_fs(i,1)));
    R_fs_check = GenerateRotMat(0, 0, deg2rad(eulZYX_fs(i,1)));
    quat_fs_check = rotm2quat(R_fs_check);

    R_fs_prev = FrenetSerretFrame(time_sim(i-1), trajectory_x, trajectory_y, trajectory_z);

    R_rm = RotationMinimizingFrame(time_sim(i), ...
                               trajectory_x, ...
                               trajectory_y, ...
                               trajectory_z, ...
                               time_sim(i-1), ...
                               R_fs_prev); % R_fs_prev - R_init
    quat_rm = rotm2quat(R_rm);
    eulZYX_rm(i,:) = rad2deg(rotm2eul(R_rm,"ZYX"));

    if k == i
        ax1 = plotTransforms(pos_fs',quat_fs,"FrameSize",1,"FrameAxisLabels",'on');
        % ax2 = plotTransforms(pos_fs',quat_rm,"FrameSize",2,"FrameAxisLabels",'on');
        % ax2 = plotTransforms(pos_fs',quat_rm,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor",[0.8500 0.3250 0.0980]);
        ax2 = plotTransforms(pos_fs',quat_fs_thetax,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor",[0.8500 0.3250 0.0980]);
        % ax2 = plotTransforms(pos_fs',quat_fs_check,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor",[0.8500 0.3250 0.0980]);
        drawnow
        k = k + 30; % How often do you want to plot the reference frame
    end

    % ax3 = plotTransforms(pos_fs',quat_init,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor","blue");
    % ax2 = plotTransforms(pos_fs',quat_rm,"FrameSize",2,"FrameAxisLabels",'on',"FrameColor",[0.8500 0.3250 0.0980]);
    
    % pause(0.001)
    % drawnow
end

font_size = 20;
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(time_sim,eulZYX_fs(:,3),'b-','LineWidth',2)
hold on
plot(time_sim,eulZYX_rm(:,3),'r-.','LineWidth',2)
legend('Frenet-Serret', 'Rotation Minimizing');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('ZYX rotation sequence','interpreter','latex','fontsize',font_size)
axis tight
hold off

subplot(3,1,2)
plot(time_sim,eulZYX_fs(:,2),'b-','LineWidth',2)
hold on
plot(time_sim,eulZYX_rm(:,2),'r-.','LineWidth',2)
legend('Frenet-Serret', 'Rotation Minimizing');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off

subplot(3,1,3)
plot(time_sim,eulZYX_fs(:,1),'b-','LineWidth',2)
hold on
plot(time_sim,eulZYX_rm(:,1),'r-.','LineWidth',2)
% plot(time_sim,eulXYZ_fs(:,3),'k','LineWidth',1)
% plot(poly_t.p5,rad2deg(yaw_ref.p5),'LineWidth',2,'LineStyle','--')
plot(poly_t.p2,rad2deg(yaw_ref.p2),'LineWidth',2,'LineStyle','--')
legend('Frenet-Serret', 'Rotation Minimizing', 'Using atan2()');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off

set(figure,'Color','white','WindowState','maximized')
plot(time_sim,eulZYX_fs(:,1) - eulZYX_rm(:,1))

set(figure,'Color','white','WindowState','maximized')
plot(time_sim(2:end),diff(eulZYX_fs(:,1))/Delta_time,'b-','LineWidth',2)
hold on
plot(time_sim(2:end),diff(eulZYX_rm(:,1))/Delta_time,'r-.','LineWidth',2)
legend('Frenet-Serret', 'Rotation Minimizing');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw rate [deg/s]','interpreter','latex','fontsize',font_size)
axis tight
ylim([0,50])
hold off

set(figure,'Color','white','WindowState','maximized')
plot(time_sim(3:end),diff(diff(eulZYX_fs(:,1)))/Delta_time^2,'b-','LineWidth',2)
hold on
plot(time_sim(3:end),diff(diff(eulZYX_rm(:,1)))/Delta_time^2,'r-.','LineWidth',2)
legend('Frenet-Serret', 'Rotation Minimizing');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw rate rate [deg/s$^2$]','interpreter','latex','fontsize',font_size)
axis tight
ylim([-50,50])
hold off


set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(time_sim,eulZYX_fs(:,3),'b-','LineWidth',2)
hold on
plot(time_sim,eulZYX_fs_thetax(:,3),'r-.','LineWidth',2)
legend('Frenet-Serret', 'Frenet-Serret Rotated');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('ZYX rotation sequence','interpreter','latex','fontsize',font_size)
axis tight
hold off

subplot(3,1,2)
plot(time_sim,eulZYX_fs(:,2),'b-','LineWidth',2)
hold on
plot(time_sim,eulZYX_fs_thetax(:,2),'r-.','LineWidth',2)
legend('Frenet-Serret', 'Frenet-Serret Rotated');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off

subplot(3,1,3)
plot(time_sim,eulZYX_fs(:,1),'b-','LineWidth',2)
hold on
plot(time_sim,eulZYX_fs_thetax(:,1),'r-.','LineWidth',2)
plot(poly_t.p2,rad2deg(yaw_ref.p2),'LineWidth',2,'LineStyle','--')
legend('Frenet-Serret', 'Frenet-Serret Rotated','Using atan2()');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off

set(figure,'Color','white','WindowState','maximized')
plot(time_sim,eulZYX_fs_thetax(:,3) - eulZYX_fs(:,3))




%% waypointTrajectory

clear all
close all
clc

wps = [0 0 0;
      20 0 0;
      20 5 0;
      0  5 0;
      0  0 0];

vels = [2 0 0;
        2 0 0;
       -2 0 0;
       -2 0 0;
        2 0 0];

t = cumsum([0 20/2 5*pi/2/2 20/2 5*pi/2/2]');

eulerAngs = [0 0 0;
             0 0 0;
           180 0 0;
           180 0 0;
             0 0 0]; % Angles in degrees.
% Convert Euler angles to quaternions.
quats = quaternion(eulerAngs,"eulerd","ZYX","frame");

fs = 100;

traj = waypointTrajectory(wps,SampleRate=fs, ...
        Velocities=vels,...
        TimeOfArrival=t,...
        Orientation=quats);

% traj = waypointTrajectory(wps,SampleRate=fs, ...
%         Velocities=vels,...
%         JerkLimit=0,...
%         Orientation=quats);

% traj = waypointTrajectory(wps,SampleRate=fs, ...
%         Velocities=vels,...
%         Orientation=quats);

% traj = waypointTrajectory(wps,SampleRate=fs, ...
%         TimeOfArrival=t,...
%         Orientation=quats);

[pos, orient, vel, acc, angvel] = traj();
i = 1;

spf = traj.SamplesPerFrame;
while ~isDone(traj)
    idx = (i+1):(i+spf);
    [pos(idx,:), orient(idx,:), ...
        vel(idx,:), acc(idx,:), angvel(idx,:)] = traj();
    i = i+spf;
end

plot(pos(:,1),pos(:,2), wps(:,1),wps(:,2), "--o")
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
legend({"Trajectory", "Waypoints"})
axis equal

t_sampled = (0:(1/fs):(t(end)-(1/fs)))';

figure
plot(t_sampled,pos(:,1),t_sampled,pos(:,2),t_sampled,pos(:,3))

figure
plot(t_sampled,vel(:,1),t_sampled,vel(:,2),t_sampled,vel(:,3))

figure
plot(t_sampled,acc(:,1),t_sampled,acc(:,2),t_sampled,acc(:,3))

%% quinticpolytraj

clear all
close all
clc


wpts = [1 4 4 3 -2 0;
        0 1 2 4 3 1];

% tpts = 0:5;
tpts = [0,0.5,1,1.2,4,5];

tvec = 0:0.01:5;

[q, qd, qdd, pp] = quinticpolytraj(wpts, tpts, tvec);

plot(tvec, q)
hold all
plot(tpts, wpts, 'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions')
hold off

figure
plot(q(1,:),q(2,:),'.b',wpts(1,:),wpts(2,:),'or')
xlabel('X')
ylabel('Y')

%% minjerkpolytraj 2D WE WANT THIS

clear all
close all
clc


% wpts = [1 4 4 3 -2 0;
%         0 1 2 4 3 1];

% tpts = 0:5;
% tpts = 0:2:10;
% tpts = [0,0.5,1,2.5,4,5];

wpts = [0 20 20 0 0;
        0 0  5  5 0];

% tpts = cumsum([0 20/2 5*pi/2/2 20/2 5*pi/2/2]');

tpts = [0 10 16 22 32]';

numsamples = 100;

[q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts,tpts,numsamples);

plot(tsamples,q)
hold on
plot(timepoints,wpts,'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions')
hold off

figure
plot(q(1,:),q(2,:),'.b',wpts(1,:),wpts(2,:),'or')
xlabel('X')
ylabel('Y')
axis equal

figure
plot(tsamples,qd)
xlabel('t')
ylabel('Velocity')
legend('X-velocity','Y-velocity')

figure
plot(tsamples,qdd)
xlabel('t')
ylabel('Accelerations')
legend('X-acceleration','Y-acceleration')

figure
plot(tsamples,qddd)
xlabel('t')
ylabel('Jerks')
legend('X-jerk','Y-jerk')

