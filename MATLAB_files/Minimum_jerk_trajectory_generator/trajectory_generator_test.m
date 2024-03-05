clear all
close all
clc


% traj.wpts = [0  5  20  20   0  0;
%              0  0   0   5   5  0;
%              0 -1  -1  -1  -1 -1];

% Trajectory waypoints
traj.wpts = [0  5  20  20   0  0;
             0  0   0   5   5  0;
             0 -1  -1  -1  -1 -1];

% Coefficient to scale the waypoints' times
traj.alphaFactor = 1.5;
% traj.alphaFactor = 0.5;

% Waypoints' times
% traj.tpts = traj.alphaFactor*[0 4 8 14 20 30]';
% traj.tpts = traj.alphaFactor*[0 4 8 11 15 22]';
traj.tpts = traj.alphaFactor*[0 2 5 10 16 28]';

% Number of samples used to evaluate the trajectory
traj.numsamples = 200;

% Minimum Jerk trajectory
[traj.q,traj.qd,traj.qdd,traj.qddd,traj.pp,traj.timepoints,traj.tsamples] = ...
    minjerkpolytraj(traj.wpts, traj.tpts, traj.numsamples);

% Plotting trajectory in 3D
set(figure,'Color','white')
plot3(traj.q(1,:),traj.q(2,:),traj.q(3,:),'.b',...
      traj.wpts(1,:),traj.wpts(2,:),traj.wpts(3,:),'or')
xlabel('X')
ylabel('Y')
zlabel('Z')
ax = gca;
ax.YDir = 'reverse';
ax.ZDir = 'reverse';
axis equal

% TRAJECTORY POSITION 
% Assigning the polynomial coefficients to the designated variables
[position.coef.x,...
 position.coef.y,...
 position.coef.z] = PolyCoefAssigning(traj.pp.coefs);

% TRAJECTORY VELOCITY 
% Assigning the polynomial coefficients to the designated variables
velocity.coef.x = PolyderMatrix(position.coef.x);
velocity.coef.y = PolyderMatrix(position.coef.y);
velocity.coef.z = PolyderMatrix(position.coef.z);

% TRAJECTORY ACCELERATION 
% Assigning the polynomial coefficients to the designated variables
acceleration.coef.x = PolyderMatrix(velocity.coef.x);
acceleration.coef.y = PolyderMatrix(velocity.coef.y);
acceleration.coef.z = PolyderMatrix(velocity.coef.z);

% TRAJECTORY JERK 
% Assigning the polynomial coefficients to the designated variables
jerk.coef.x = PolyderMatrix(acceleration.coef.x);
jerk.coef.y = PolyderMatrix(acceleration.coef.y);
jerk.coef.z = PolyderMatrix(acceleration.coef.z);

traj.step = 0.01;
t=traj.tpts(1):traj.step:traj.tpts(end);

for i = 1:length(t)
[t_adjusted(i),segment(i)] = PolyTimeAdjusted(traj.tpts,t(i));

position.x(i) = polyval(position.coef.x(segment(i),:), t_adjusted(i));
position.y(i) = polyval(position.coef.y(segment(i),:), t_adjusted(i));
position.z(i) = polyval(position.coef.z(segment(i),:), t_adjusted(i));

velocity.x(i) = polyval(velocity.coef.x(segment(i),:), t_adjusted(i));
velocity.y(i) = polyval(velocity.coef.y(segment(i),:), t_adjusted(i));
velocity.z(i) = polyval(velocity.coef.z(segment(i),:), t_adjusted(i));
velocity.norm2D(i) = Norm2D(velocity.coef.x(segment(i),:),...
                         velocity.coef.y(segment(i),:),...
                         t_adjusted(i));

acceleration.x(i) = polyval(acceleration.coef.x(segment(i),:), t_adjusted(i));
acceleration.y(i) = polyval(acceleration.coef.y(segment(i),:), t_adjusted(i));
acceleration.z(i) = polyval(acceleration.coef.z(segment(i),:), t_adjusted(i));

jerk.x(i) = polyval(jerk.coef.x(segment(i),:), t_adjusted(i));
jerk.y(i) = polyval(jerk.coef.y(segment(i),:), t_adjusted(i));
jerk.z(i) = polyval(jerk.coef.z(segment(i),:), t_adjusted(i));

if t_adjusted(i) == 0
    yaw(i) = 0;
    yaw_dot(i) = 0;
    yaw_dot_dot(i) = 0;

elseif t_adjusted(i) > 0 & velocity.norm2D(i) < 1e-5
    yaw(i) = yaw(i-1);
    yaw_dot(i) = 0;
    yaw_dot_dot(i) = 0;

else

    yaw(i) = YawComputation(velocity.coef.x(segment(i),:),...  
                            velocity.coef.y(segment(i),:),...
                            t_adjusted(i));
    
    yaw_dot(i) = YawDotComputation(velocity.coef.x(segment(i),:),...
                                   velocity.coef.y(segment(i),:),...
                                   acceleration.coef.x(segment(i),:),...
                                   acceleration.coef.y(segment(i),:),...
                                   t_adjusted(i));
    
    yaw_dot_dot(i) = YawDotDotComputation(velocity.coef.x(segment(i),:),...
                                          velocity.coef.y(segment(i),:),...
                                          acceleration.coef.x(segment(i),:),...
                                          acceleration.coef.y(segment(i),:),...
                                          jerk.coef.x(segment(i),:),...
                                          jerk.coef.y(segment(i),:),...
                                          t_adjusted(i));
end

end

%% Plots
font_size = 20;

set(figure,'Color','white','WindowState','maximized')
plot(t,yaw,'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [rad]','interpreter','latex','fontsize',font_size)
axis tight

set(figure,'Color','white','WindowState','maximized')
plot(t,yaw_dot,'b-','LineWidth',2)
hold on
plot(t(2:end),diff(yaw)/traj.step,'r--','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw rate [rad/s]','interpreter','latex','fontsize',font_size)
legend('Complex numbers','Differentiating','interpreter','latex','fontsize',font_size)
ylim([-2,2])
% axis tight
hold off

set(figure,'Color','white','WindowState','maximized')
plot(t,yaw_dot_dot,'b-','LineWidth',2)
hold on
plot(t(3:end),diff(diff(yaw))/traj.step^2,'r--','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw acceleration [rad/s$^2$]','interpreter','latex','fontsize',font_size)
legend('Complex numbers','Differentiating','interpreter','latex','fontsize',font_size)
ylim([-2,2])
% axis tight
hold off

%% Exporting trajectory piecewise polynomial coefficient matrix

% Flipping the matrix rows to conform to Python convention of 
% polynomial coefficients
traj.pp.coefs_inverted = flip(traj.pp.coefs,2);

writematrix(traj.pp.coefs_inverted,'trajectory_PolynomialCoefficientMatrix.csv') 
writematrix(traj.tpts,'trajectory_WaypointTimes.csv') 





