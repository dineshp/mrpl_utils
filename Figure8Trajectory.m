classdef Figure8Trajectory < handle
    % CubicSpiral Implements a planar trajectory specified in terms of three 
    % coefficients that adjust the terminal pose of the robot. The initial
    % pose is assumed to be the origin with zero curvature. The terminal
    % curvature is forced to be zero.
    
    properties(Constant)
        k_s = 3.0;
        s_f = 1.0*Figure8Trajectory.k_s;
        k_th = (2.0*pi/Figure8Trajectory.s_f);
        k_k = (15.1084/Figure8Trajectory.k_s);
    end
    
    properties(Access = private)
        rampLength = 0.05;
    end
    
    properties(Access = public)
        numSamples = 0;
        distArray = [];
        timeArray = [];
        poseArray = [];
        curvArray = [];
        vlArray = []
        vrArray = []
        VArray = [];
        wArray = [];
    end
    
    methods(Access = public)
        
        function obj = Figure8Trajectory(n)
            obj.numSamples = n;
        end
    
        function integrateCommands(obj)
            len = obj.numSamples;
            obj.distArray  = zeros(1,len);
            obj.poseArray  = zeros(3,len);
            obj.curvArray  = zeros(1,len);

            % Place robot in initial state
            obj.distArray(1) = 0.0;
            obj.poseArray(1,1) = 0.0;
            obj.poseArray(2,1) = 0.0;
            obj.poseArray(3,1) = 0.0;
            obj.curvArray(1) = 0.0;

            ds = Figure8Trajectory.s_f/(obj.numSamples-1);
            for i=1:(obj.numSamples-1)             
                s_i = (i-1)*ds;
                obj.distArray(i) = s_i;
                
                kurv_s = obj.k_k*sin(obj.k_th*s_i);
                obj.curvArray(i) = kurv_s;
                
                p_pose = Pose(obj.poseArray(1,i), obj.poseArray(2,i), obj.poseArray(3,i));
                pose = RobotModelAdv.integrateDiffEqDs(kurv_s, ds, p_pose);
                
                obj.poseArray(3,i+1) = pose.th;
                obj.poseArray(1,i+1) = pose.x;
                obj.poseArray(2,i+1) = pose.y;     
            end
            i = obj.numSamples;
            s = (i-1)*ds;  
            obj.distArray(i) = s;
            obj.curvArray(i) = 0.0;
        end 
         
        function plot(obj)
            plotArray1 = obj.poseArray(1,:);
            plotArray2 = obj.poseArray(2,:);
            plot(plotArray1,plotArray2,'r');
        end      
                
        function planVelocities(obj,Vmax,sgn)
            % Plan the highest possible velocity for the path where no
            % wheel may exceed Vmax in absolute value.
            for i=1:obj.numSamples
                Vbase = Vmax;
                % Add velocity ramps for first and last 5 cm
                s = obj.distArray(i);
                sf = obj.distArray(obj.numSamples);
                if(abs(sf) > 2.0*obj.rampLength) % no ramp for short trajectories
                    sUp = abs(s);
                    sDn = abs(sf-s);
                    if(sUp < obj.rampLength) % ramp up
                        Vbase = Vbase * sUp/obj.rampLength;
                    elseif(sDn < 0.05) % ramp down
                        Vbase = Vbase * sDn/obj.rampLength;
                    end
                end
                % Now proceed with base velocity 
                %disp(Vbase);
                V = Vbase*sgn; % Drive forward or backward as desired.
                K = obj.curvArray(i);
                w = K*V;
                
                [vl , vr] = RobotModelAdv.VwTovlvr(V, w);               
                if(abs(vr) > Vbase)
                    vrNew = Vbase * sign(vr);
                    vl = vl * vrNew/vr;
                    vr = vrNew;
                end
                if(abs(vl) > Vbase)
                    vlNew = Vbase * sign(vl);
                    vr = vr * vlNew/vl;
                    vl = vlNew;
                end
                obj.vlArray(i) = vl;
                obj.vrArray(i) = vr;
                [V_i, w_i] = RobotModelAdv.vlvrToVw(vl, vr);
                obj.VArray(i) = V_i;
                obj.wArray(i) = w_i;                
            end
            % Now compute the times that are implied by the velocities and
            % the distances.
            obj.computeTimeSeries();
        end
        
        function computeTimeSeries(obj)
            % Find the times implied by the distances and the velocities
            len = obj.numSamples;
            obj.timeArray  = zeros(1,len);

            % Place robot in initial state
            obj.timeArray(1) = 0.0;

            for i=1:obj.numSamples-1
                ds = obj.distArray(i+1) - obj.distArray(i);
                V = obj.VArray(i);
                % Avoid division by zero
                if(abs(V) < 0.001); V = obj.VArray(i+1); end;

                obj.timeArray(i+1)= obj.timeArray(i)+ds/V;
            end
        end
        
        function pose  = getFinalPose(obj)
            pose  = obj.poseArray(:,obj.numSamples);  
        end  
        
        function time = getTimeAtDist(obj, s)
            if( s < obj.distArray(1))
                time = 0;
            elseif(s > obj.getTrajectoryDistance())
                time = obj.getTrajectoryDuration();
            else
                time = interp1(obj.distArray,obj.timeArray,s,'pchip','extrap');
            end
        end
        
        function dist = getDistAtTime(obj, t)
            if( t < obj.timeArray(1))
                dist = 0;
            elseif (t > obj.getTrajectoryDuration())
                dist = obj.getTrajectoryDistance();
            else
                dist = interp1(obj.timeArray,obj.distArray,t,'pchip','extrap');
            end
        end
               
        function pose  = getPoseAtDist(obj,s)
            if( s < obj.distArray(1))
                pose = Pose(0,0,0);
            elseif (s > obj.getTrajectoryDistance())
                pose = Pose(obj.poseArray(1,obj.numSamples), obj.poseArray(2,obj.numSamples), obj.poseArray(3,obj.numSamples));
            else
                x = interp1(obj.distArray,obj.poseArray(1,:),s,'pchip','extrap');
                y = interp1(obj.distArray,obj.poseArray(2,:),s,'pchip','extrap');
                th = interp1(obj.distArray,obj.poseArray(3,:),s,'pchip','extrap');
                pose  = Pose(x, y, th);
            end
        end
        
        function V = getVAtDist(obj,s)
            if( s < obj.distArray(1) || s > obj.getTrajectoryDistance())
                V = 0.0;
            else
                V = interp1(obj.distArray,obj.VArray,s,'pchip','extrap');
            end
        end

        function w = getwAtDist(obj,s)
            if( s < obj.distArray(1) || s > obj.getTrajectoryDistance())
                w = 0.0;
            else
                w = interp1(obj.distArray,obj.wArray,s,'pchip','extrap');
            end
        end
        
        function time = getTrajectoryDuration(obj)
            time = obj.timeArray(:,obj.numSamples);  
        end
        
        function dist  = getTrajectoryDistance(obj)
            dist = obj.distArray(:,obj.numSamples);  
        end
        
        
        function V  = getVAtTime(obj,t)
            if(t < obj.timeArray(1) || t > obj.getTrajectoryDuration())
                V = 0.0;
            else
                V  = interp1(obj.timeArray,obj.VArray,t,'pchip','extrap');  
            end
        end
            
        function w  = getwAtTime(obj,t)
            if( t < obj.timeArray(1) || t > obj.getTrajectoryDuration())
                w = 0.0;
            else
                w  = interp1(obj.timeArray,obj.wArray,t,'pchip','extrap');  
            end
        end
            
        function pose  = getPoseAtTime(obj,t)
            if( t < obj.timeArray(1))
                pose = Pose(0,0,0);
            elseif (t > obj.getTrajectoryDuration())
                pose = Pose(obj.poseArray(1,obj.numSamples), obj.poseArray(2,obj.numSamples), obj.poseArray(3,obj.numSamples));
            else
                x = interp1(obj.timeArray,obj.poseArray(1,:),t,'pchip','extrap');
                y = interp1(obj.timeArray,obj.poseArray(2,:),t,'pchip','extrap');
                th = interp1(obj.timeArray,obj.poseArray(3,:),t,'pchip','extrap');
                pose  = Pose(x, y, th);  
            end
        end  
        
    end
    
end