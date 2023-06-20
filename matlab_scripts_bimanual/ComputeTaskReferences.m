function [pandaArmL, pandaArm] = ComputeTaskReferences(pandaArmL, pandaArm)
% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
 % left reference
[v_rho_L, v_dist_L] = CartError(pandaArmL.wTtg, pandaArmL.ArmL.wTt); 
pandaArmL.x_dot.ArmL.v_d  = 0.2 * (v_dist_L);
pandaArmL.x_dot.ArmL.v_o = 0.2 * (v_rho_L);

% right reference
[v_rho_R, v_dist_R] = CartError(pandaArm.wTtg, pandaArm.ArmR.wTt);
pandaArm.x_dot.ArmR.v_d  =  0.2 * (v_dist_R);
pandaArm.x_dot.ArmR.v_o = 0.2 * (v_rho_R);

%joint reference
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

pandaArm.ArmR.xdot.jl = zeros(7, 1);
for i =1:size(jlmin, 1)
    mean(i) = (pandaArm.jlmin(i) + pandaArm.jlmax(i)) / 2;
    if (pandaArm.ArmR.q(i) >= mean(i))
        pandaArm.ArmR.xdot.jl(i) = 0.2 * (pandaArm.jlmax(i) - pandaArm.ArmR.q(i));
    else
        pandaArm.ArmR.xdot.jl(i) = 0.2 * (pandaArm.jlmin(i) - pandaArm.ArmR.q(i));
    end
end

pandaArmL.ArmL.xdot.jl = zeros(7, 1);
for i =1:size(jlmin, 1)
    mean(i) = (pandaArm.jlmin(i) + pandaArm.jlmax(i)) / 2;
    if (pandaArmL.ArmL.q(i) >= mean(i))
        pandaArmL.ArmL.xdot.jl(i) = 0.2 * (pandaArm.jlmax(i) - pandaArmL.ArmL.q(i));
    else
        pandaArmL.ArmL.xdot.jl(i) = 0.2 * (pandaArm.jlmin(i) - pandaArmL.ArmL.q(i));
    end
end 

% left reference
[v_rho_L2, v_dist_L2] = CartError(pandaArmL.ArmL.wTtg2, pandaArmL.ArmL.wTo); 
pandaArmL.x_dot.ArmL.v_d2  = 0.2 * (v_dist_L2);
pandaArmL.x_dot.ArmL.v_o2 = 0.2 * (v_rho_L2);

% right reference
[v_rho_R2, v_dist_R2] = CartError(pandaArm.ArmR.wTtg2, pandaArm.ArmR.wTo);
pandaArm.x_dot.ArmR.v_d2  =  0.2 * (v_dist_R2);
pandaArm.x_dot.ArmR.v_o2 = 0.2 * (v_rho_R2);

% left reference
[v_rho_L3, v_dist_L3] = CartError(pandaArmL.ArmL.wTtg3, pandaArmL.ArmL.wTo); 
pandaArmL.x_dot.ArmL.v_d3  = 0.2 * (v_dist_L3);
pandaArmL.x_dot.ArmL.v_o3 = 0.2 * (v_rho_L3);

% right reference
[v_rho_R3, v_dist_R3] = CartError(pandaArm.ArmR.wTtg3, pandaArm.ArmR.wTo);
pandaArm.x_dot.ArmR.v_d3  =  0.2 * (v_dist_R3);
pandaArm.x_dot.ArmR.v_o3 = 0.2 * (v_rho_R3);

end
