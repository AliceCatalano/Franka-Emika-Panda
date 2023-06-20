function [pandaArm] = InitRobotRight(model,wTb2)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm.ArmR = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.ArmR.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.ArmR.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');
pandaArm.ArmR.wTb = wTb2;
% joint limits corresponding to the actual Panda by Franka arm configuration
pandaArm.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
pandaArm.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Init relevance Jacobians
pandaArm.ArmR.bJe = eye(6,7);
pandaArm.Jjl = eye(6);
%% ... TO HERE
% Init Task Reference vectors
pandaArm.xdot.jl = [];

pandaArm.xdot.ArmR.tool = [];
pandaArm.x_dot.ArmR.v_d = [];
pandaArm.x_dot.ArmR.v_o = [];
pandaArm.xdot.jl = [];
pandaArm.x_bar_dot.ArmR= [];
pandaArm.x_dot.ArmR.v_o2 = [];
pandaArm.x_dot.ArmR.v_d2 = [];


pandaArm.ArmR.wTrg = [];
pandaArm.ArmR.wTt = [];
pandaArm.wTog = []; 
pandaArm.ArmR.wTr1g = [];
pandaArm.ArmR.wTog = [];
% Init Activation Functions
% Activation function for activate or deactivate tasks
pandaArm.A.jldx = zeros(7,7);
pandaArm.Aa.toolR = zeros(6,6);
pandaArm.Aa.toolR_lim = zeros(6,6);
pandaArm.ArmR.Jtool = [];


end

