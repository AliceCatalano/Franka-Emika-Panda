function [pandaArmL, pandaArm, mission] = UpdateMissionPhase(pandaArmL, pandaArm, mission) 
   
        switch mission.phase
            case 1  
     
                % Go To Grasping Points
                 if (norm(pandaArm.x_dot.ArmR.v_d) < 0.01 && norm(pandaArmL.x_dot.ArmL.v_d) < 0.01 && norm(pandaArmL.x_dot.ArmL.v_o) < 0.01 && norm(pandaArm.x_dot.ArmR.v_o))  
                    mission.phase = 2;
                 end  
            case 2  
                % BimanualRigidGrasping
                 if (norm(pandaArm.x_dot.ArmR.v_d2) < 0.01 && norm(pandaArmL.x_dot.ArmL.v_d2) < 0.01 && norm(pandaArmL.x_dot.ArmL.v_o2) < 0.01 && norm(pandaArm.x_dot.ArmR.v_o2))  
                    mission.phase = 3;
                 end  

            case 3    
                % JL costraint
                activation = 0;
                for i = 1:7
                    activation = activation + abs(pandaArmL.A.jl(i, i)) + abs(pandaArm.A.jl(i, i));
                end
                if (activation > 0.1)
                    mission.phase = 4;
                end  
                
            case 4   
                % End of movement
                
                
        end
end

