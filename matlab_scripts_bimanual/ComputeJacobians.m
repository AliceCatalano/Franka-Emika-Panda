function [pandaArmL, pandaArm] = ComputeJacobians(pandaArmL, pandaArm)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, -0.0305686, 1.53975, 0.753872] ) 
%
% remember: the control vector is:
% [q_dot] 
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% ydot = [qdot_1_l qdot_2_l qdot3.... qdot_7_l qdot1_r qdot2_l ....
% qdot7_r]' 14 x 1

%           Left Arm Jacobian    %
% Base to ee 
pandaArmL.ArmL.bJe = geometricJacobian(pandaArmL.ArmL.franka,[pandaArmL.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
% Common 
pandaArmL.Jjl = [eye(7)];
% World to tool
pandaArmL.ArmL.skewt = [eye(3) zeros(3); -skew(pandaArmL.ArmL.wTe(1:3,1:3) * pandaArmL.ArmL.eTt(1:3, 4)) eye(3)];
pandaArmL.ArmL.Jtool = pandaArmL.ArmL.skewt*[pandaArmL.ArmL.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArmL.ArmL.wTb(1:3,1:3)]*pandaArmL.ArmL.bJe(:,1:7);
% World to object 
LtTo = [rotation(0, -pi, pi) [0.05; 0; 0]; 0 0 0 1];
pandaArmL.ArmL.skewo = [eye(3) zeros(3); -skew(pandaArmL.ArmL.wTt(1:3,1:3) * LtTo(1:3, 4)) eye(3)];
pandaArmL.ArmL.wToJ = pandaArmL.ArmL.skewo*pandaArmL.ArmL.Jtool;

%           Right Arm Jacobian    %

% Base to ee 
pandaArm.ArmR.bJe = geometricJacobian(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT
% Common Jacobians
pandaArm.Jjl = [eye(7)];
% World to tool
pandaArm.ArmR.skewt = [eye(3) zeros(3); -skew(pandaArm.ArmR.wTt(1:3,1:3)*pandaArm.ArmR.eTt(1:3, 4)) eye(3)];
pandaArm.ArmR.Jtool = pandaArm.ArmR.skewt*[pandaArm.ArmR.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArm.ArmR.wTb(1:3,1:3)]*pandaArm.ArmR.bJe(:,1:7); 
% World to object 
RtTo = [rotation(0, pi, 0) [0.05; 0; 0]; 0 0 0 1];
pandaArm.ArmR.skewo = [eye(3) zeros(3); -skew(pandaArm.ArmR.wTt(1:3,1:3) * RtTo(1:3, 4)) eye(3)];
pandaArm.ArmR.wToJ = pandaArm.ArmR.skewo*pandaArm.ArmR.Jtool;

% Subspace of the combined end effector velocities
pandaArmL.ArmL.inv_wToJ = pinv(pandaArmL.ArmL.wToJ);
pandaArmL.ArmL.H = pandaArmL.ArmL.wToJ*pandaArmL.ArmL.inv_wToJ;

pandaArm.ArmR.inv_wToJ = pinv(pandaArm.ArmR.wToJ);
pandaArm.ArmR.H = pandaArm.ArmR.wToJ*pandaArm.ArmR.inv_wToJ;

% Total H matrix
pandaArm.H = [pandaArmL.ArmL.H zeros(6); zeros(6) pandaArm.ArmR.H];
% Cartesian constraint
pandaArm.C = [pandaArmL.ArmL.H, - pandaArm.ArmR.H];
pandaArm.inv_C = pinv(pandaArm.C);

end