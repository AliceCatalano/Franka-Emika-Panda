function [pandaArm] = InitRobotLeft(model,wTb1)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm.ArmL = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.ArmL.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.ArmL.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');
pandaArm.ArmL.wTb = wTb1;

% joint limits corresponding to the actual Panda by Franka arm configuration
pandaArm.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
pandaArm.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Init relevance Jacobians
pandaArm.ArmL.bJe = eye(6,7);
pandaArm.Jjl = eye(6);
%% ... TO HERE
% Init Task Reference vectors
pandaArm.xdot.jl = [];
pandaArm.xdot.ArmL.tool = [];
pandaArm.x_dot.ArmL.v_d = [];
pandaArm.x_dot.ArmL.v_o = [];
pandaArm.xdot.jl = [];
pandaArm.ArmL.x_bar_dot1 = [];
pandaArm.x_dot.ArmL.v_o2 = [];
pandaArm.x_dot.ArmL.v_d2 = [];

pandaArm.ArmL.wTrg = [];
pandaArm.ArmL.wTt = [];
pandaArm.wTog = []; 
pandaArm.ArmL.wTr1g = [];
pandaArm.ArmL.wTog = [];
pandaArm.ArmL.wTtg = [];


% Init Activation Functions
% Activation function for activate or deactivate tasks
pandaArm.A.jl = zeros(7,7);
pandaArm.Aa.toolL = zeros(6,6);
pandaArm.Aa.toolL_lim = zeros(6,6);



end

