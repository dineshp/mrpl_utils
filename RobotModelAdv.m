classdef RobotModelAdv
    %robotModel A convenience class for storing robot physical 
    % and performing related kinematic transforms. You can reference the
    % defined constants via the class name with robotModel.W2 for
    % example because they are constant properties and therefore associated
    % with the class rather than any instance. Similiarly, the kinematics
    % routines are referenced from the class name as well.
    
    properties (Constant)
        ModelW = .08943;
        %.08943;
        ModelW2 = RobotModelAdv.ModelW/2;
        maxWheelVelocity = 0.5; % max of either wheel in m/sec
       
        tdelay = 0.5;          % comms delay (bidirectional)
    end
    
    properties(Access = private)

    end
    
    properties(Access = public)
    end
    
    methods(Static = true)
        
        function [V , w] = vlvrToVw(vl, vr)
        % Converts wheel speeds to body linear and angular velocity.
            V = (vr + vl)/2.0;
            w = (vr - vl)/RobotModelAdv.ModelW;
        end
        
        function [vl , vr] = VwTovlvr(V, w)
        % Converts body linear and angular velocity to wheel speeds.
            vr = V + RobotModelAdv.ModelW2*w;
            vl = V - RobotModelAdv.ModelW2*w;
        end
               
        function [vl , vr] = limitWheelVelocities(ctrVec)
        % Limits the speed of both wheels
            vl = ctrVec(1);
            vr = ctrVec(2);
            scale = abs(vr) / RobotModelAdv.maxWheelVelocity;
            if(scale > 1.0)
                vr = vr/scale;
                vl = vl/scale;
            end
            scale = abs(vl) / RobotModelAdv.maxWheelVelocity;
            if(scale > 1.0)
                vr = vr/scale;
                vl = vl/scale;
            end
        end
        
        function r_pose = integrateDiffEqDs(kurv, ds, p_pose)
            
            p_th = p_pose.th;
            p_x = p_pose.x;
            p_y = p_pose.y;

            th = p_th + ((kurv)*ds);

            k00 = cos(p_th);
            k01 = sin(p_th);

            k10 = cos(p_th + (ds/2.0) * kurv);
            k11 = sin(p_th + (ds/2.0) * kurv);

            k20 = k10;
            k21 = k11;

            k30 = cos(th);
            k31 = sin(th);                
            x = p_x + ((ds/6.0) * (k00 + 2*(k10 + k20) + k30));
            y = p_y + ((ds/6.0) * (k01 + 2*(k11 + k21) + k31));   
            r_pose = Pose(x, y, th);
        end
        
        function r_pose = integrateDiffEq(V, w, dt, p_pose)
            disp(p_pose.getPoseVec());
            p_x = p_pose.x;
            p_y = p_pose.y; 
            p_th = p_pose.th;
            
            th = p_th + w*dt;

            k00 = V * cos(p_th);
            k01 = V * sin(p_th);

            k10 = V * cos(p_th + (dt/2.0) * w);
            k11 = V * sin(p_th + (dt/2.0) * w);

            k20 = k10;
            k21 = k11;

            k30 = V * cos(p_th + w*dt);
            k31 = V * sin(p_th + w*dt);

            x = p_x + ((dt/6.0) * (k00 + 2*(k10 + k20) + k30));
            y = p_y + ((dt/6.0) * (k01 + 2*(k11 + k21) + k31));
            
            r_pose = Pose(x,y,th);
        end
        
    end
    
    methods(Access = private)
        
    end
            
    methods(Access = public)
        

    end
end