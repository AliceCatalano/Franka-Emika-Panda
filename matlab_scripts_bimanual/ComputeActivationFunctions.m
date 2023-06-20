function [pandaArmL, pandaArm] = ComputeActivationFunctions(pandaArmL, pandaArm,mission)
        
        switch mission.phase
            
            case 1  % REACH GOAL FRAME
                pandaArmL.Aa.toolL = zeros(6);
                pandaArmL.Aa.toolLi = eye(6);
                pandaArm.Aa.toolRi = eye(6);
                pandaArm.Aa.toolR = zeros(6);
                pandaArmL.Aa.toolL_lim = zeros(6);
                pandaArm.Aa.toolR_lim = zeros(6);
            case 2 % GRASP AND MOVE THE OBJECT
                pandaArmL.Aa.toolL = eye(6);
                pandaArmL.Aa.toolLi = zeros(6);
                pandaArm.Aa.toolRi = zeros(6);
                pandaArm.Aa.toolR = eye(6);
                pandaArmL.Aa.toolL_lim = zeros(6);
                pandaArm.Aa.toolR_lim = zeros(6);
            case 3 % JOINT CONSTRAINT VERIFICATION
                pandaArmL.Aa.toolL_lim = eye(6);
                pandaArm.Aa.toolR_lim = eye(6);
                pandaArmL.Aa.toolL = zeros(6);
                pandaArmL.Aa.toolLi = zeros(6);
                pandaArm.Aa.toolRi = zeros(6);
                pandaArm.Aa.toolR = zeros(6);
            case 4 % STOP any motion  
                pandaArmL.Aa.toolL = zeros(6);
                pandaArmL.Aa.toolLi = zeros(6);
                pandaArm.Aa.toolRi = zeros(6);
                pandaArm.Aa.toolR = zeros(6);
                pandaArmL.Aa.toolL_lim = zeros(6);
                pandaArm.Aa.toolR_lim = zeros(6);
        end

% Control tasks (equality)
pandaArm.A.Jtool = pandaArm.Aa.toolR;
pandaArmL.A.Jtool = pandaArmL.Aa.toolL;

% Activation function: two combined sigmoids, which are at their maximum at the joint limits and approach zero between them 
pandaArmL.A.jl = eye(7) .* (DecreasingBellShapedFunction(pandaArmL.jlmin - 0.25 ,pandaArmL.jlmin,0,1, pandaArmL.ArmL.q) + IncreasingBellShapedFunction(pandaArmL.jlmax, pandaArmL.jlmax + 0.25, 0, 1, pandaArmL.ArmL.q));
pandaArm.A.jl = eye(7) .* (DecreasingBellShapedFunction(pandaArm.jlmin - 0.25 ,pandaArm.jlmin,0,1, pandaArm.ArmR.q) + IncreasingBellShapedFunction(pandaArm.jlmax,pandaArm.jlmax+0.25,0,1,pandaArm.ArmR.q));

end
