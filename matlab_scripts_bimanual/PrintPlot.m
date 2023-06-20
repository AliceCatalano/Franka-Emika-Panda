function [ ] = PrintPlot( plt,pandaArmL, pandaArm )

% some predefined plots
% you can add your own

% Desired object velocity CASE 1
figure(1);
subplot(2,1,1)
hplot = plot(plt.xdot_tool1_goTograsp(7,:), plt.xdot_tool1_goTograsp(1:6,:));
title('Left Desired object velocity goToGrasp CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.xdot_tool2_goTograsp(7,:), plt.xdot_tool2_goTograsp(1:6,:));
title('Right Desired object velocity CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
saveas(gcf, 'armDesired.jpg');

figure(2);
subplot(2,1,1)
hplot = plot(plt.xdot_tool1_bringToPoint(7,:), plt.xdot_tool1_bringToPoint(1:6,:));
title('Left desired object velocity moveObject CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.xdot_tool2_bringToPoint(7,:), plt.xdot_tool2_bringToPoint(1:6,:));
title('Right desired object velocity moveObject CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
saveas(gcf, 'armDesired1.jpg');

% Non-cooperative Cartesian velocity CASE 1
figure(3);
subplot(2,1,1)
hplot = plot(plt.xdot_tool1_cartVel1(7,:), plt.xdot_tool1_cartVel1(1:6,:));
title('Left Non-cooperative Cartesian velocity CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.xdot_tool2_cartVel1(7,:), plt.xdot_tool2_cartVel1(1:6,:));
title('Right Non-cooperative Cartesian velocity CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
saveas(gcf, 'armCartesian1.jpg');

% Cooperative velocity CASE 1
figure(4);
subplot(2,1,1)
hplot = plot(plt.xdot_tool1_coopVel1(7,:), plt.xdot_tool1_coopVel1(1:6,:));
title('Left Cooperative velocity CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.xdot_tool2_coopVel1(7,:), plt.xdot_tool2_coopVel1(1:6,:));
title('Right Cooperative velocity CASE 1')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
saveas(gcf, 'armCooperative1.jpg');

% Desired object velocity CASE 2
figure(5);
subplot(2,1,1)
hplot = plot(plt.xdot_tool1_jointLim(6,:), plt.xdot_tool1_jointLim(1:6,:));
title('Left Desired object velocity jointLimits Test CASE 2')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.xdot_tool2_jointLim(6,:), plt.xdot_tool2_jointLim(1:6,:));
title('Right Desired object velocity jointLimits Test CASE 2')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
saveas(gcf, 'jointTest.jpg');


% Non-cooperative Cartesian velocity CASE 2
figure(6);
subplot(2,1,1)
hplot = plot(plt.xdot_tool1_cartVel2(6,:), plt.xdot_tool1_cartVel2(1:6,:));
title('Left Non-cooperative Cartesian velocity CASE 2')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.xdot_tool2_cartVel2(6,:), plt.xdot_tool2_cartVel2(1:6,:));
title('Right Non-cooperative Cartesian velocity CASE 2')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
saveas(gcf, 'armCartesian2.jpg');

% Cooperative velocity CASE 2
figure(7);
subplot(2,1,1)
hplot = plot(plt.xdot_tool1_coopVel2(6,:), plt.xdot_tool1_coopVel2(1:6,:));
title('Left Cooperative velocity CASE 2')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.xdot_tool2_coopVel2(6,:), plt.xdot_tool2_coopVel2(1:6,:));
title('Right Cooperative velocity CASE 2')
xlabel('time [s]');
ylabel('omega [rad/s] xdot [m/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
saveas(gcf, 'armCooperative2.jpg');

end

