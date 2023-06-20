function [ plt ] = UpdateDataPlot( plt,pandaArmL, pandaArm, t, loop, mission )

% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

% Desired object velocity CASE 1
if mission.phase == 1
    plt.xdot_tool1_goTograsp(:, loop) = [pandaArmL.x_dot.ArmL.v_o ;pandaArmL.x_dot.ArmL.v_d; t];
    plt.xdot_tool2_goTograsp(:, loop) = [pandaArm.x_dot.ArmR.v_o ;pandaArm.x_dot.ArmR.v_d; t];

elseif mission.phase == 2
    % Desired object velocity CASE 1
    plt.xdot_tool1_bringToPoint(:, loop) = [pandaArmL.x_dot.ArmL.v_o2 ;pandaArmL.x_dot.ArmL.v_d2; t];
    plt.xdot_tool2_bringToPoint(:, loop) = [pandaArm.x_dot.ArmR.v_o2 ;pandaArm.x_dot.ArmR.v_d2; t];
    
    % Non-cooperative Cartesian velocity
    plt.xdot_tool1_cartVel1(:, loop) = [pandaArmL.ArmL.x_dot.m; t]; 
    plt.xdot_tool2_cartVel1(:, loop) = [pandaArm.ArmR.x_dot.m; t];
    
    % Cooperative velocity
    plt.xdot_tool1_coopVel1(:, loop) = [pandaArmL.ArmL.x_dot.coop; t];
    plt.xdot_tool2_coopVel1(:, loop) = [pandaArm.ArmR.x_dot.coop; t];


elseif mission.phase == 3

    % Desired object velocity CASE 2
    plt.xdot_tool1_jointLim(:, loop) = [pandaArmL.x_dot.ArmL.v_o3 ;pandaArmL.x_dot.ArmL.v_d3; t];
    plt.xdot_tool2_jointLim(:, loop) = [pandaArm.x_dot.ArmR.v_o3 ;pandaArm.x_dot.ArmR.v_d3; t];
    
    % Non-cooperative Cartesian velocity
    plt.xdot_tool1_cartVel2(:, loop) = [pandaArmL.ArmL.x_dot.m; t]; 
    plt.xdot_tool2_cartVel2(:, loop) = [pandaArm.ArmR.x_dot.m; t];
    
    % Cooperative velocity
    plt.xdot_tool1_coopVel2(:, loop) = [pandaArmL.ArmL.x_dot.coop; t];
    plt.xdot_tool2_coopVel2(:, loop) = [pandaArm.ArmR.x_dot.coop; t];

end
end