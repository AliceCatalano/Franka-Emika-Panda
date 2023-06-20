 %% 
function MainPandaArmBimanual
addpath('./simulation_scripts');
clc;
clear;
close all

%% Initialization - DON'T CHANGE ANYTHING
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 30;
loop = 1;
maxloops = ceil(end_time/deltat);
mission.phase = 1;
mission.phase_time = 0;
model = load("panda.mat"); %load the model - DO NOT CHANGE panda.mat

% UDP Connection with Franka Interface - DO NOT CHANGE
hudps = dsp.UDPSender('RemoteIPPort',1500);
hudps.RemoteIPAddress = '127.0.0.1';

% Init robot model
%
% first armL base-world transformation
wTb1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% second armL base - world transformation
wTb2 = [rotation(0, 0, pi) [1.05; 0; 0]; 0 0 0 1];%[-1 0 0 1.05; 0 -1 0 0; 0 0 1 0; 0 0 0 1]; forse è giusta ma non sono sicuro quindi la ricalcolo

plt = InitDataPlot(maxloops);

pandaArmBimanual_Left = InitRobotLeft(model,wTb1);
pandaArmBimanual = InitRobotRight(model,wTb2);

% Init object and tools frames
obj_length = 0.1;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);

% Define trasnformation matrix from ee to tool.
eTtL = [rotation(0, 0, -43.1937 * ((2 * pi)/360)) [0; 0; 0.1]; 0 0 0 1];
eTtR = [rotation(0, 0, -43.1937 * ((2 * pi)/360)) [0; 0; 0.1]; 0 0 0 1];

pandaArmBimanual_Left.ArmL.eTt = eTtL; 
pandaArmBimanual.ArmR.eTt = eTtR; 
%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
pandaArmBimanual_Left.wTtg = [rotation(0, pi, pi) [0.45; 0; 0.59]; 0 0 0 1]; % final goal wrt world
pandaArmBimanual.wTtg = [rotation(0, pi, 0) [0.55; 0; 0.59]; 0 0 0 1]; % final goal wrt world

wTo = [1 0 0 0.5;0 1 0 0;0 0 1 0.59; 0 0 0 1];
LtTo = [rotation(0, -pi, pi) [0.05; 0; 0]; 0 0 0 1];
RtTo = [rotation(0, pi, 0) [0.05; 0; 0]; 0 0 0 1];


% Transformation matrix from world to endeffector.
pandaArmBimanual_Left.ArmL.wTe = wTb1 * pandaArmBimanual_Left.ArmL.bTe  ; 
pandaArmBimanual.ArmR.wTe = wTb2 * pandaArmBimanual.ArmR.bTe;
% Transformation matrix from world to tool.
pandaArmBimanual_Left.ArmL.wTt = pandaArmBimanual_Left.ArmL.wTe * eTtL;
pandaArmBimanual.ArmR.wTt = pandaArmBimanual.ArmR.wTe * eTtR;
% Transformation matrix from world to object.
pandaArmBimanual_Left.ArmL.wTo = pandaArmBimanual_Left.ArmL.wTt * LtTo;
pandaArmBimanual.ArmR.wTo = pandaArmBimanual.ArmR.wTt * RtTo;

% Second goal move the object
pandaArmBimanual_Left.ArmL.wTtg2 = [1 0 0 0.5; 0 1 0 -0.5; 0 0 1 0.5; 0 0 0 1];
pandaArmBimanual.ArmR.wTtg2 = [1 0 0 0.5; 0 1 0 -0.5; 0 0 1 0.5; 0 0 0 1];

jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Third goal stress the joint limits
jl = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
pandaArmBimanual_Left.ArmL.wTtg3 = [rotation(0, jl(1), jl(2)) [jl(3); jl(4); jl(5)]; 0 0 0 1];
pandaArmBimanual.ArmR.wTtg3 = [rotation(0, jl(6), jl(7)) [jl(3); jl(4); jl(5)]; 0 0 0 1];


%% CONTROL LOOP

for t = 0:deltat:end_time
    %update all the involved variables left
    [pandaArmBimanual_Left, pandaArmBimanual] = UpdateTransforms(pandaArmBimanual_Left, pandaArmBimanual);
    [pandaArmBimanual_Left, pandaArmBimanual] = ComputeJacobians(pandaArmBimanual_Left, pandaArmBimanual);
    [pandaArmBimanual_Left, pandaArmBimanual] = ComputeActivationFunctions(pandaArmBimanual_Left, pandaArmBimanual, mission);
    [pandaArmBimanual_Left, pandaArmBimanual] = ComputeTaskReferences(pandaArmBimanual_Left, pandaArmBimanual);
    

    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(7,1);
    Qp = eye(7);
    % ADD minimum distance from table
    % add all the other tasks here!
    
    %           Desired task References     %
    % NON-cooperative phase
    pandaArmBimanual_Left.ArmL.x_bar_dot = [pandaArmBimanual_Left.x_dot.ArmL.v_o; pandaArmBimanual_Left.x_dot.ArmL.v_d];    
    pandaArmBimanual.ArmR.x_bar_dot = [pandaArmBimanual.x_dot.ArmR.v_o; pandaArmBimanual.x_dot.ArmR.v_d];
    % Cooperative phase
    pandaArmBimanual_Left.ArmL.x_bar_dot1 = [pandaArmBimanual_Left.x_dot.ArmL.v_o2; pandaArmBimanual_Left.x_dot.ArmL.v_d2];
    pandaArmBimanual.ArmR.x_bar_dot1 = [pandaArmBimanual.x_dot.ArmR.v_o2; pandaArmBimanual.x_dot.ArmR.v_d2];
    % Joint Limits test
    pandaArmBimanual_Left.ArmL.x_bar_dot2 = [pandaArmBimanual_Left.x_dot.ArmL.v_o3; pandaArmBimanual_Left.x_dot.ArmL.v_d3];
    pandaArmBimanual.ArmR.x_bar_dot2 = [pandaArmBimanual.x_dot.ArmR.v_o3; pandaArmBimanual.x_dot.ArmR.v_d3];
    
    % the sequence of iCAT_task calls defines the priority
    
    [QpLi, ydotbarLi] = iCAT_task(pandaArmBimanual_Left.A.jl, pandaArmBimanual_Left.Jjl, Qp, ydotbar, pandaArmBimanual_Left.ArmL.xdot.jl,  0.0001,   0.01, 10);% JOINT LIMITS TASK
    [QpLi, ydotbarLi] = iCAT_task(pandaArmBimanual_Left.Aa.toolLi,   pandaArmBimanual_Left.ArmL.Jtool, QpLi, ydotbarLi,  pandaArmBimanual_Left.ArmL.x_bar_dot,  0.0001,   0.01, 10);% LEFT ARM TOOL CONTROL TASK indipendent
    [QpLi, ydotbarLi] = iCAT_task(pandaArmBimanual_Left.Aa.toolL,   pandaArmBimanual_Left.ArmL.wToJ, QpLi, ydotbarLi,  pandaArmBimanual_Left.ArmL.x_bar_dot1,  0.0001,   0.01, 10);% LEFT ARM TOOL CONTROL TASK 
    [QpLi, ydotbarLi] = iCAT_task(pandaArmBimanual_Left.Aa.toolL_lim,   pandaArmBimanual_Left.ArmL.Jtool, QpLi, ydotbarLi,  pandaArmBimanual_Left.ArmL.x_bar_dot2,  0.0001,   0.01, 10);%LEFT ARM REACH JOINT LIMIT
    
    [QpRi, ydotbarRi] = iCAT_task(pandaArmBimanual.A.jl, pandaArmBimanual.Jjl, Qp, ydotbar, pandaArmBimanual.ArmR.xdot.jl,  0.0001,   0.01, 10);% JOINT LIMITS TASK
    [QpRi, ydotbarRi] = iCAT_task(pandaArmBimanual.Aa.toolRi,  pandaArmBimanual.ArmR.Jtool, QpRi, ydotbarRi, pandaArmBimanual.ArmR.x_bar_dot,  0.0001,   0.01, 10);% RIGHT ARM TOOL CONTROL TASK indipendent 
    [QpRi, ydotbarRi] = iCAT_task(pandaArmBimanual.Aa.toolR, pandaArmBimanual.ArmR.wToJ, QpRi, ydotbarRi, pandaArmBimanual.ArmR.x_bar_dot1,  0.0001,   0.01, 10);% RIGHT ARM TOOL CONTROL TASK
    [QpRi, ydotbarRi] = iCAT_task(pandaArmBimanual.Aa.toolR_lim,   pandaArmBimanual.ArmR.Jtool, QpRi, ydotbarRi,  pandaArmBimanual.ArmR.x_bar_dot2,  0.0001,   0.01, 10);%RIGHT ARM REACH JOINT LIMIT
    
    %non coop velocity 
    pandaArmBimanual_Left.ArmL.x_dot.m = pandaArmBimanual_Left.ArmL.Jtool * ydotbarLi;
    pandaArmBimanual.ArmR.x_dot.m = pandaArmBimanual.ArmR.Jtool * ydotbarRi; 
    
    %cooperative tool frame velocity
    mu_o = 0.001;
    mu_a = mu_o + norm( pandaArmBimanual_Left.ArmL.x_bar_dot - pandaArmBimanual_Left.ArmL.x_dot.m);
    mu_b = mu_o + norm( pandaArmBimanual.ArmR.x_bar_dot - pandaArmBimanual.ArmR.x_dot.m);
    
    pandaArmBimanual.x_dot.mcoop = (1/(mu_a + mu_b))*( mu_a*pandaArmBimanual_Left.ArmL.x_dot.m + mu_b*pandaArmBimanual.ArmR.x_dot.m);
    pandaArmBimanual.x_dot.fcoop = pandaArmBimanual.H*(eye(12) - pandaArmBimanual.inv_C*pandaArmBimanual.C)*[pandaArmBimanual.x_dot.mcoop; pandaArmBimanual.x_dot.mcoop];
    % Cooperative velocities of each arm
    pandaArmBimanual_Left.ArmL.x_dot.fcoop = pandaArmBimanual.x_dot.fcoop(1:6, :);
    pandaArmBimanual.ArmR.x_dot.fcoop =pandaArmBimanual.x_dot.fcoop(7:12,:);
    
    % Cooperative TPIK
    [QpLc, ydotbarLc] = iCAT_task(pandaArmBimanual_Left.Aa.toolL,   pandaArmBimanual_Left.ArmL.wToJ, Qp, ydotbar, pandaArmBimanual_Left.ArmL.x_dot.fcoop,  0.0001,   0.01, 10);% LEFT ARM TOOL CONTROL TASK 
    [QpLc, ydotbarLc] = iCAT_task(pandaArmBimanual_Left.Aa.toolLi,   pandaArmBimanual_Left.ArmL.Jtool, QpLc, ydotbarLc,  pandaArmBimanual_Left.ArmL.x_bar_dot,  0.0001,   0.01, 10);% LEFT ARM TOOL CONTROL TASK indipendent 
    [QpLc, ydotbarLc] = iCAT_task(pandaArmBimanual_Left.Aa.toolL_lim,   pandaArmBimanual_Left.ArmL.wToJ, QpLc, ydotbarLc, pandaArmBimanual_Left.ArmL.x_dot.fcoop,  0.0001,   0.01, 10); % JOINT LIMITS TEST TASK
    [QpLc, ydotbarLc] = iCAT_task(pandaArmBimanual_Left.A.jl, pandaArmBimanual_Left.Jjl, QpLc, ydotbarLc, pandaArmBimanual_Left.ArmL.xdot.jl,  0.0001,   0.01, 10);% JOINT LIMITS TASK
    [QpLc, ydotbarLc] = iCAT_task(eye(7), eye(7), QpLc, ydotbarLc, zeros(7,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    [QpRc, ydotbarRc] = iCAT_task(pandaArmBimanual.Aa.toolR,  pandaArmBimanual.ArmR.wToJ, Qp, ydotbar, pandaArmBimanual.ArmR.x_dot.fcoop,  0.0001,   0.01, 10);% RIGHT ARM TOOL CONTROL TASK 
    [QpRc, ydotbarRc] = iCAT_task(pandaArmBimanual.Aa.toolRi,  pandaArmBimanual.ArmR.Jtool, QpRc, ydotbarRc, pandaArmBimanual.ArmR.x_bar_dot,  0.0001,   0.01, 10);% RIGHT ARM TOOL CONTROL TASK indipendent 
    [QpRc, ydotbarRc] = iCAT_task(pandaArmBimanual.Aa.toolR_lim,   pandaArmBimanual.ArmR.wToJ, QpRc, ydotbarRc, pandaArmBimanual.ArmR.x_dot.fcoop,  0.0001,   0.01, 10); % JOINT LIMITS TEST TASK
    [QpRc, ydotbarRc] = iCAT_task(pandaArmBimanual.A.jl, pandaArmBimanual.Jjl, QpRc, ydotbarRc, pandaArmBimanual.ArmR.xdot.jl,  0.0001,   0.01, 10); % JOINT LIMITS TASK
    [QpRc, ydotbarRc] = iCAT_task(eye(7), eye(7), QpRc, ydotbarRc, zeros(7,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % Cooperative velocity
    pandaArmBimanual_Left.ArmL.x_dot.coop = pandaArmBimanual_Left.ArmL.wToJ * ydotbarLc;
    pandaArmBimanual.ArmR.x_dot.coop = pandaArmBimanual.ArmR.wToJ * ydotbarRc;


    % get the two variables for integration
    pandaArmBimanual_Left.ArmL.q_dot = ydotbarLc;
    pandaArmBimanual.ArmR.q_dot = ydotbarRc;
   
    % Integration
	pandaArmBimanual_Left.ArmL.q = pandaArmBimanual_Left.ArmL.q(1:7) + pandaArmBimanual_Left.ArmL.q_dot*deltat;    
    pandaArmBimanual.ArmR.q = pandaArmBimanual.ArmR.q(1:7) + pandaArmBimanual.ArmR.q_dot*deltat;
    
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    
    [pandaArmBimanual_Left, pandaArmBimanual,mission] = UpdateMissionPhase(pandaArmBimanual_Left, pandaArmBimanual, mission);
    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    posMsg = [pandaArmBimanual_Left.ArmL.q;pandaArmBimanual.ArmR.q];
    step(hudps,posMsg)
    % Update data plot
    plt = UpdateDataPlot(plt,pandaArmBimanual_Left,pandaArmBimanual,t,loop, mission);
    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        t 
        phase = mission.phase
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    %SlowdownToRealtime(deltat);
    
end
PrintPlot(plt,pandaArmBimanual_Left, pandaArmBimanual);
end
