

clear all
close all
clc

load('Workspace');

% Overall properties
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesFontSize',20);


font_size = 20;
font_size_title = 22;

%% Plot thrust vs time PID Unaware of payload
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayUnawa.data.time,sum(PID.PayUnawa.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$u_1(t)$','Motor failure'); 
% str={'Motor failure'};
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');




% axis ([min(PID.PayUnawa.data.time), max(PID.PayUnawa.data.time), min(sum(PID.PayUnawa.data.thrust,2)),max(sum(PID.PayUnawa.data.thrust,2))])

set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,2),'r-.','LineWidth',2)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,3),'g--','LineWidth',2)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,4),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$','');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
str={'Motor failure'};
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,6),'r-.','LineWidth',2)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,7),'g--','LineWidth',2)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.thrust(:,8),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$','Motor failure');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.275, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot angles vs. time PID Unaware of payload
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayUnawa.data.time,rad2deg(PID.PayUnawa.data.roll),'b-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,rad2deg(PID.PayUnawa.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(PID.PayUnawa.data.time,rad2deg(PID.PayUnawa.data.pitch),'b-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,rad2deg(PID.PayUnawa.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.45, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayUnawa.data.time,rad2deg(PID.PayUnawa.data.yaw),'b-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,rad2deg(PID.PayUnawa.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot position vs time PID Unaware of payload

set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
legend('$r_x(t)$','$r_{\rm user,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.75, 0.2, 0.2], 'EdgeColor', 'none');


subplot(3,1,2)
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,PID.PayUnawa.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
legend('$r_y(t)$','$r_{\rm user,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.45, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayUnawa.data.time,-PID.PayUnawa.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(PID.PayUnawa.data.time,-PID.PayUnawa.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
legend('$r_z(t)$','$r_{\rm user,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.655, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================

%% Plot thrust vs time PID Aware of payload
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,sum(PID.PayAwa.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$u_1(t)$','Motor failure'); 
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');




% axis ([min(PID.PayUnawa.data.time), max(PID.PayUnawa.data.time), min(sum(PID.PayUnawa.data.thrust,2)),max(sum(PID.PayUnawa.data.thrust,2))])

set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,2),'r-.','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,3),'g--','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,4),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$','');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
str={'Motor failure'};
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,6),'r-.','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,7),'g--','LineWidth',2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.thrust(:,8),'k-','LineWidth',0.5)
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$','Motor failure');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.28, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot angles vs. time PID Aware of payload
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.roll),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.pitch),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.yaw),'b-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,rad2deg(PID.PayAwa.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------
%% Plot position vs time PID Aware of payload

set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
legend('$r_x(t)$','$r_{\rm user,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('PID','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');


subplot(3,1,2)
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,PID.PayAwa.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
legend('$r_y(t)$','$r_{\rm user,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(PID.PayAwa.data.time,-PID.PayAwa.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(PID.PayAwa.data.time,-PID.PayAwa.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
legend('$r_z(t)$','$r_{\rm user,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%% Plot thrust vs time MRAC
set(figure,'Color','white','WindowState','maximized')
plot(MRAC.data.time,sum(MRAC.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
legend('$u_1(t)$');
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');




set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(MRAC.data.time,MRAC.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(MRAC.data.time,MRAC.data.thrust(:,2),'r-.','LineWidth',2)
plot(MRAC.data.time,MRAC.data.thrust(:,3),'g--','LineWidth',2)
plot(MRAC.data.time,MRAC.data.thrust(:,4),'k-','LineWidth',0.5)
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(MRAC.data.time,MRAC.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(MRAC.data.time,MRAC.data.thrust(:,6),'r-.','LineWidth',2)
plot(MRAC.data.time,MRAC.data.thrust(:,7),'g--','LineWidth',2)
plot(MRAC.data.time,MRAC.data.thrust(:,8),'k-','LineWidth',0.5)
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.28, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot angles vs. time MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(MRAC.data.time,rad2deg(MRAC.data.roll),'b-','LineWidth',2)
hold on
plot(MRAC.data.time,rad2deg(MRAC.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(MRAC.data.time,rad2deg(MRAC.data.pitch),'b-','LineWidth',2)
hold on
plot(MRAC.data.time,rad2deg(MRAC.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(MRAC.data.time,rad2deg(MRAC.data.yaw),'b-','LineWidth',2)
hold on
plot(MRAC.data.time,rad2deg(MRAC.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot position vs time (with reference model) MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(MRAC.data.time,MRAC.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(MRAC.data.time,MRAC.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
plot(MRAC.data.time,MRAC.data.x_ref_tran(:,1),'k','LineWidth',1)
legend('$r_x(t)$','$r_{\rm user,x}(t)$','$r_{\rm ref,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(MRAC.data.time,MRAC.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(MRAC.data.time,MRAC.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
plot(MRAC.data.time,MRAC.data.x_ref_tran(:,2),'k','LineWidth',1)
legend('$r_y(t)$','$r_{\rm user,y}(t)$','$r_{\rm ref,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(MRAC.data.time,-MRAC.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(MRAC.data.time,-MRAC.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
plot(MRAC.data.time,-MRAC.data.x_ref_tran(:,3),'k','LineWidth',1)
legend('Actual position','User-defined position','Reference model position');
legend('$r_z(t)$','$r_{\rm user,z}(t)$','$r_{\rm ref,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================


%% Plot thrust vs time Two-Layer MRAC
set(figure,'Color','white','WindowState','maximized')
plot(TwoLayerMRAC.data.time,sum(TwoLayerMRAC.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
legend('$u_1(t)$');
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');




set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,2),'r-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,3),'g--','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,4),'k-','LineWidth',0.5)
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,6),'r-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,7),'g--','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.thrust(:,8),'k-','LineWidth',0.5)
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.28, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot angles vs. time Two-Layer MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(TwoLayerMRAC.data.time,rad2deg(TwoLayerMRAC.data.roll),'b-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,rad2deg(TwoLayerMRAC.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(TwoLayerMRAC.data.time,rad2deg(TwoLayerMRAC.data.pitch),'b-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,rad2deg(TwoLayerMRAC.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(TwoLayerMRAC.data.time,rad2deg(TwoLayerMRAC.data.yaw),'b-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,rad2deg(TwoLayerMRAC.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot position vs time (with reference model) Two-Layer MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.x_ref_tran(:,1),'k','LineWidth',1)
legend('$r_x(t)$','$r_{\rm user,x}(t)$','$r_{\rm ref,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.data.x_ref_tran(:,2),'k','LineWidth',1)
legend('$r_y(t)$','$r_{\rm user,y}(t)$','$r_{\rm ref,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(TwoLayerMRAC.data.time,-TwoLayerMRAC.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(TwoLayerMRAC.data.time,-TwoLayerMRAC.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,-TwoLayerMRAC.data.x_ref_tran(:,3),'k','LineWidth',1)
legend('Actual position','User-defined position','Reference model position');
legend('$r_z(t)$','$r_{\rm user,z}(t)$','$r_{\rm ref,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================


%% Plot thrust vs time Robust MRAC
set(figure,'Color','white','WindowState','maximized')
plot(RobMRAC.data.time,sum(RobMRAC.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('Robust MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
legend('$u_1(t)$');
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');




set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,2),'r-.','LineWidth',2)
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,3),'g--','LineWidth',2)
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,4),'k-','LineWidth',0.5)
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('Robust MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,6),'r-.','LineWidth',2)
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,7),'g--','LineWidth',2)
plot(RobMRAC.data.time,RobMRAC.data.thrust(:,8),'k-','LineWidth',0.5)
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.28, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot angles vs. time Robust MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(RobMRAC.data.time,rad2deg(RobMRAC.data.roll),'b-','LineWidth',2)
hold on
plot(RobMRAC.data.time,rad2deg(RobMRAC.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('Robust MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(RobMRAC.data.time,rad2deg(RobMRAC.data.pitch),'b-','LineWidth',2)
hold on
plot(RobMRAC.data.time,rad2deg(RobMRAC.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(RobMRAC.data.time,rad2deg(RobMRAC.data.yaw),'b-','LineWidth',2)
hold on
plot(RobMRAC.data.time,rad2deg(RobMRAC.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot position vs time (with reference model) Robust MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(RobMRAC.data.time,RobMRAC.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(RobMRAC.data.time,RobMRAC.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
plot(RobMRAC.data.time,RobMRAC.data.x_ref_tran(:,1),'k','LineWidth',1)
legend('$r_x(t)$','$r_{\rm user,x}(t)$','$r_{\rm ref,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('Robust MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(RobMRAC.data.time,RobMRAC.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(RobMRAC.data.time,RobMRAC.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
plot(RobMRAC.data.time,RobMRAC.data.x_ref_tran(:,2),'k','LineWidth',1)
legend('$r_y(t)$','$r_{\rm user,y}(t)$','$r_{\rm ref,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(RobMRAC.data.time,-RobMRAC.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(RobMRAC.data.time,-RobMRAC.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
plot(RobMRAC.data.time,-RobMRAC.data.x_ref_tran(:,3),'k','LineWidth',1)
legend('Actual position','User-defined position','Reference model position');
legend('$r_z(t)$','$r_{\rm user,z}(t)$','$r_{\rm ref,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================


%% Plot thrust vs time Robust Two-Layer MRAC
set(figure,'Color','white','WindowState','maximized')
plot(RobTwoLayerMRAC.data.time,sum(RobTwoLayerMRAC.data.thrust,2),'b-','LineWidth',2)
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Total thrust [N]','interpreter','latex','fontsize',font_size)
title('Robust Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
legend('$u_1(t)$');
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');




set(figure,'Color','white','WindowState','maximized')
subplot(2,1,1)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,1),'b-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,2),'r-.','LineWidth',2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,3),'g--','LineWidth',2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,4),'k-','LineWidth',0.5)
legend('$T_1(t)$','$T_2(t)$','$T_3(t)$','$T_4(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
title('Robust Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',str,'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(2,1,2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,5),'b-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,6),'r-.','LineWidth',2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,7),'g--','LineWidth',2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.thrust(:,8),'k-','LineWidth',0.5)
legend('$T_5(t)$','$T_6(t)$','$T_7(t)$','$T_8(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Thrust motor $i$ [N]','interpreter','latex','fontsize',font_size)
hold off
axis tight
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.28, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot angles vs. time Robust Two-Layer MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(RobTwoLayerMRAC.data.time,rad2deg(RobTwoLayerMRAC.data.roll),'b-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,rad2deg(RobTwoLayerMRAC.data.roll_ref),'r-.','LineWidth',2)
legend('$$\phi(t)$$','$$\phi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Roll [deg]','interpreter','latex','fontsize',font_size)
title('Robust Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(RobTwoLayerMRAC.data.time,rad2deg(RobTwoLayerMRAC.data.pitch),'b-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,rad2deg(RobTwoLayerMRAC.data.pitch_ref),'r-.','LineWidth',2)
legend('$$\theta(t)$$','$$\theta_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Pitch [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(RobTwoLayerMRAC.data.time,rad2deg(RobTwoLayerMRAC.data.yaw),'b-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,rad2deg(RobTwoLayerMRAC.data.yaw_ref),'r-.','LineWidth',2)
legend('$$\psi(t)$$','$$\psi_{\rm ref}(t)$$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Yaw [deg]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot position vs time (with reference model) Robust Two-Layer MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.x_ref_tran(:,1),'k','LineWidth',1)
legend('$r_x(t)$','$r_{\rm user,x}(t)$','$r_{\rm ref,x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('Robust Two-Layer MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.data.x_ref_tran(:,2),'k','LineWidth',1)
legend('$r_y(t)$','$r_{\rm user,y}(t)$','$r_{\rm ref,y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(RobTwoLayerMRAC.data.time,-RobTwoLayerMRAC.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(RobTwoLayerMRAC.data.time,-RobTwoLayerMRAC.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
plot(RobTwoLayerMRAC.data.time,-RobTwoLayerMRAC.data.x_ref_tran(:,3),'k','LineWidth',1)
legend('Actual position','User-defined position','Reference model position');
legend('$r_z(t)$','$r_{\rm user,z}(t)$','$r_{\rm ref,z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================


%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================
%=============================================================================================================================================================


%% Plot thrust vs time ALL CONTROLLERS
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,sum(PID.PayAwa.data.thrust,2),'Color',"#0072BD",'LineStyle','-','LineWidth',2)
hold on
plot(MRAC.data.time,sum(MRAC.data.thrust,2),'Color',"#FF0000",'LineStyle','-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,sum(TwoLayerMRAC.data.thrust,2),'Color',"#77AC30",'LineStyle','--','LineWidth',1.5)
plot(RobMRAC.data.time,sum(RobMRAC.data.thrust,2),'Color',"#000000",'LineStyle','-','LineWidth',0.5)
plot(RobTwoLayerMRAC.data.time,sum(RobTwoLayerMRAC.data.thrust,2),'Color',"#7E2F8E",'LineStyle',':','LineWidth',1.5)
legend('PID', 'MRAC with Baseline', 'Two-Layer MRAC with Baseline', 'Robust MRAC with Baseline', 'Robust Two-Layer MRAC with Baseline');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('$$\sum_{i=1}^8T_i(t)$$ [N]','interpreter','latex','fontsize',font_size)
title('Total thrust','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

%% Plot Tracking error vs time ALL CONTROLLERS
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,PID.PayAwa.traj_track_error_norm,'Color',"#0072BD",'LineStyle','-','LineWidth',2)
hold on
plot(MRAC.data.time,MRAC.traj_track_error_norm,'Color',"#FF0000",'LineStyle','-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.traj_track_error_norm,'Color',"#77AC30",'LineStyle','--','LineWidth',1.5)
plot(RobMRAC.data.time,RobMRAC.traj_track_error_norm,'Color',"#000000",'LineStyle','-','LineWidth',0.5)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.traj_track_error_norm,'Color',"#7E2F8E",'LineStyle',':','LineWidth',1.5)
plot(HybridMRAC.data.time,HybridMRAC.traj_track_error_norm,'Color',"#EDB120",'LineStyle','--','LineWidth',1.5)
legend('PID', 'MRAC with Baseline', 'Two-Layer MRAC with Baseline', 'Robust MRAC with Baseline', 'Robust Two-Layer MRAC with Baseline','Hybrid MRAC with Baseline');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('$\|x(t) - x_{\rm user}(t)\|$','interpreter','latex','fontsize',font_size)
title('Norm of trajectory tracking error','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');


%% Plot Integral of Tracking error vs time ALL CONTROLLERS
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,PID.PayAwa.traj_track_error_L2norm,'Color',"#0072BD",'LineStyle','-','LineWidth',2)
hold on
plot(MRAC.data.time,MRAC.traj_track_error_L2norm,'Color',"#FF0000",'LineStyle','-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.traj_track_error_L2norm,'Color',"#77AC30",'LineStyle','--','LineWidth',1.5)
plot(RobMRAC.data.time,RobMRAC.traj_track_error_L2norm,'Color',"#000000",'LineStyle','-','LineWidth',0.5)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.traj_track_error_L2norm,'Color',"#7E2F8E",'LineStyle',':','LineWidth',1.5)
plot(HybridMRAC.data.time,HybridMRAC.traj_track_error_L2norm,'Color',"#EDB120",'LineStyle','--','LineWidth',1.5)
legend('PID', 'MRAC with Baseline', 'Two-Layer MRAC with Baseline', 'Robust MRAC with Baseline', 'Robust Two-Layer MRAC with Baseline','Hybrid MRAC with Baseline');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('$$\sqrt{\int_{t_0}^t\|x(\tau) - x_{\rm user}(\tau)\|^2\mathrm{d}\tau}$$','interpreter','latex','fontsize',font_size)
title('$\mathcal{L}_2$-norm of trajectory tracking error','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

%% 3D PLOT

set(figure,'Color','white','WindowState','maximized')
plot3(RobTwoLayerMRAC.data.translational_position_in_I_user(:,1),-RobTwoLayerMRAC.data.translational_position_in_I_user(:,2),-RobTwoLayerMRAC.data.translational_position_in_I_user(:,3),'Color',"#4DBEEE",'LineStyle','-','LineWidth',2)
hold on
plot3(PID.PayAwa.data.translational_position_in_I(:,1),-PID.PayAwa.data.translational_position_in_I(:,2),-PID.PayAwa.data.translational_position_in_I(:,3),'Color',"#0072BD",'LineStyle','-','LineWidth',2)
plot3(MRAC.data.translational_position_in_I(:,1),-MRAC.data.translational_position_in_I(:,2),-MRAC.data.translational_position_in_I(:,3),'Color',"#FF0000",'LineStyle','-.','LineWidth',2)
plot3(TwoLayerMRAC.data.translational_position_in_I(:,1),-TwoLayerMRAC.data.translational_position_in_I(:,2),-TwoLayerMRAC.data.translational_position_in_I(:,3),'Color',"#77AC30",'LineStyle','--','LineWidth',1.5)
plot3(RobMRAC.data.translational_position_in_I(:,1),-RobMRAC.data.translational_position_in_I(:,2),-RobMRAC.data.translational_position_in_I(:,3),'Color',"#000000",'LineStyle','-','LineWidth',0.5)
plot3(RobTwoLayerMRAC.data.translational_position_in_I(:,1),-RobTwoLayerMRAC.data.translational_position_in_I(:,2),-RobTwoLayerMRAC.data.translational_position_in_I(:,3),'Color',"#7E2F8E",'LineStyle',':','LineWidth',1.5)
plot3(HybridMRAC.data.translational_position_in_I(:,1),-HybridMRAC.data.translational_position_in_I(:,2),-HybridMRAC.data.translational_position_in_I(:,3),'Color',"#EDB120",'LineStyle','--','LineWidth',1.5)
l = legend('$r_{\rm user}$','PID', 'MRAC with Baseline', 'Two-Layer MRAC with Baseline', 'Robust MRAC with Baseline', 'Robust Two-Layer MRAC with Baseline');
axis equal
title('Position 3D','interpreter','latex','fontsize',18)
grid on
% xlim([-5 5])
% ylim([-5 5])
% zlim([0 5])
xlabel('X [m]','interpreter','latex','fontsize',16)
ylabel('Y [m]','interpreter','latex','fontsize',16)
zlabel('Z [m]','interpreter','latex','fontsize',16)
set(l,'interpreter','latex','fontsize',18)
set(gca, 'fontsize', 20);
hold off


%% Plot position vs time (with reference model) Hybrid MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(HybridMRAC.data.time,HybridMRAC.data.translational_position_in_I(:,1),'r-','LineWidth',2)
hold on
plot(HybridMRAC.data.time,HybridMRAC.data.translational_position_in_I_user(:,1),'b-.','LineWidth',2)
plot(HybridMRAC.data.time,HybridMRAC.data.x_ref_tran(:,1),'k','LineWidth',1)
legend('$r_x(t)$','$r_{{\rm user},x}(t)$','$r_{{\rm ref},x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X position [m]','interpreter','latex','fontsize',font_size)
title('Hybrid MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(HybridMRAC.data.time,HybridMRAC.data.translational_position_in_I(:,2),'r-','LineWidth',2)
hold on
plot(HybridMRAC.data.time,HybridMRAC.data.translational_position_in_I_user(:,2),'b-.','LineWidth',2)
plot(HybridMRAC.data.time,HybridMRAC.data.x_ref_tran(:,2),'k','LineWidth',1)
legend('$r_y(t)$','$r_{{\rm user},y}(t)$','$r_{{\rm ref},y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(HybridMRAC.data.time,-HybridMRAC.data.translational_position_in_I(:,3),'r-','LineWidth',2)
hold on
plot(HybridMRAC.data.time,-HybridMRAC.data.translational_position_in_I_user(:,3),'b-.','LineWidth',2)
plot(HybridMRAC.data.time,-HybridMRAC.data.x_ref_tran(:,3),'k','LineWidth',1)
legend('Actual position','User-defined position','Reference model position');
legend('$r_z(t)$','$r_{{\rm user},z}(t)$','$r_{{\rm ref},z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z position [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------


%% Plot position vs time (with reference model) Hybrid MRAC
set(figure,'Color','white','WindowState','maximized')
subplot(3,1,1)
plot(HybridMRAC.data.time,HybridMRAC.data.translational_velocity_in_I(:,1),'r-','LineWidth',2)
hold on
plot(HybridMRAC.data.time,HybridMRAC.data.translational_velocity_in_I_user(:,1),'b-.','LineWidth',2)
plot(HybridMRAC.data.time,HybridMRAC.data.x_ref_tran(:,4),'k','LineWidth',1)
legend('$r_x(t)$','$r_{{\rm user},x}(t)$','$r_{{\rm ref},x}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('X velocity [m]','interpreter','latex','fontsize',font_size)
title('Hybrid MRAC with Baseline','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,2)
plot(HybridMRAC.data.time,HybridMRAC.data.translational_velocity_in_I(:,2),'r-','LineWidth',2)
hold on
plot(HybridMRAC.data.time,HybridMRAC.data.translational_velocity_in_I_user(:,2),'b-.','LineWidth',2)
plot(HybridMRAC.data.time,HybridMRAC.data.x_ref_tran(:,5),'k','LineWidth',1)
legend('$r_y(t)$','$r_{{\rm user},y}(t)$','$r_{{\rm ref},y}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Y velocity [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.455, 0.2, 0.2], 'EdgeColor', 'none');

subplot(3,1,3)
plot(HybridMRAC.data.time,-HybridMRAC.data.translational_velocity_in_I(:,3),'r-','LineWidth',2)
hold on
plot(HybridMRAC.data.time,-HybridMRAC.data.translational_velocity_in_I_user(:,3),'b-.','LineWidth',2)
plot(HybridMRAC.data.time,-HybridMRAC.data.x_ref_tran(:,6),'k','LineWidth',1)
legend('Actual position','User-defined position','Reference model position');
legend('$r_z(t)$','$r_{{\rm user},z}(t)$','$r_{{\rm ref},z}(t)$');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('Z velocity [m]','interpreter','latex','fontsize',font_size)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.155, 0.2, 0.2], 'EdgeColor', 'none');

%--------------------------------------------------------------------------

%% Plot Position error vs time ALL CONTROLLERS
set(figure,'Color','white','WindowState','maximized')
plot(PID.PayAwa.data.time,PID.PayAwa.pos_track_error_norm,'Color',"#0072BD",'LineStyle','-','LineWidth',2)
hold on
plot(MRAC.data.time,MRAC.pos_track_error_norm,'Color',"#FF0000",'LineStyle','-.','LineWidth',2)
plot(TwoLayerMRAC.data.time,TwoLayerMRAC.pos_track_error_norm,'Color',"#77AC30",'LineStyle','--','LineWidth',1.5)
plot(RobMRAC.data.time,RobMRAC.pos_track_error_norm,'Color',"#000000",'LineStyle','-','LineWidth',0.5)
plot(RobTwoLayerMRAC.data.time,RobTwoLayerMRAC.pos_track_error_norm,'Color',"#7E2F8E",'LineStyle',':','LineWidth',1.5)
plot(HybridMRAC.data.time,HybridMRAC.pos_track_error_norm,'Color',"#EDB120",'LineStyle','--','LineWidth',1.5)
legend('PID', 'MRAC with Baseline', 'Two-Layer MRAC with Baseline', 'Robust MRAC with Baseline', 'Robust Two-Layer MRAC with Baseline','Hybrid MRAC with Baseline');
xlabel('$t$ [s]','interpreter','latex','fontsize',font_size)
ylabel('$\|x(t) - x_{\rm user}(t)\|$','interpreter','latex','fontsize',font_size)
title('Norm of trajectory tracking error','interpreter','latex','fontsize',font_size_title)
axis tight
hold off
xl = xline(3,':','LineWidth',2, 'HandleVisibility', 'off');
annotation('textbox','interpreter','latex','String',{'Motor failure'},'FitBoxToText','on',...
    'FontSize', 16,'Position', [0.195, 0.755, 0.2, 0.2], 'EdgeColor', 'none');


