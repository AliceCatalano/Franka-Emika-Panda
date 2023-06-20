function [pandaArmL, pandaArm] = UpdateTransforms(pandaArmL, pandaArm)
% the function updates all the transformations
% Left arm transformations
pandaArmL.ArmL.bTe = getTransform(pandaArmL.ArmL.franka,[pandaArmL.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
% Right arm transformations
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% First armL base-world transformation
wTb1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% second armR base - world transformation
wTb2 = [rotation(0, 0, pi) [1.05; 0; 0]; 0 0 0 1];

% Transformation matrix from world to end effector.
pandaArmL.ArmL.wTe = wTb1 * pandaArmL.ArmL.bTe  ; 

% Transformation matrix from world to end effector.
pandaArm.ArmR.wTe = wTb2 * pandaArm.ArmR.bTe;

% Transformation matrix from world to tool.
pandaArmL.ArmL.wTt = pandaArmL.ArmL.wTe * pandaArmL.ArmL.eTt;

% Transformation matrix from world to tool.
pandaArm.ArmR.wTt = pandaArm.ArmR.wTe * pandaArm.ArmR.eTt ;

% transformation matrix between the tool and the object reference frame
LtTo = [rotation(0, -pi, pi) [0.05; 0; 0]; 0 0 0 1];

% transformation matrix between the tool and the object reference frame
RtTo = [rotation(0, pi, 0) [0.05; 0; 0]; 0 0 0 1];

% Transformation matrix from world to object
pandaArmL.ArmL.wTo = pandaArmL.ArmL.wTt * LtTo;
pandaArm.ArmR.wTo = pandaArm.ArmR.wTt * RtTo;

