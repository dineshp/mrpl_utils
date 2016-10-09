classdef Figure8Trajectory < handle
    % CubicSpiral Implements a planar trajectory specified in terms of three 
    % coefficients that adjust the terminal pose of the robot. The initial
    % pose is assumed to be the origin with zero curvature. The terminal
    % curvature is forced to be zero.
    
    properties(Constant)
        k_k = 15.1084;
        s_f = 1.0;
        k_th = 2*pi/Figure8ReferenceControl.s_f;
    end
    
    properties(Access = private)
        rampLength = 0.05;
    end
    
    properties(Access = public)
        numSamples = 0;
        k_s;
        k_v;
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
        
        function obj = Figure8Trajectory(k_s, k_v, n)
            obj.k_s = k_s;
            obj.k_v = k_v;
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
            for i=1:obj.numSamples-1             
                s_i = (i-1)*ds;
                obj.distArray(i) = s_i;
                
                kurv_s = (Figure8ReferenceControl.k_k/obj.k_s)*sin(Figure8ReferenceControl.k_th*s_i);
                obj.curvArray(i) = kurv_s;
                
                p_th = obj.poseArray(3,i);
                p_x = obj.poseArray(1,i);
                p_y = obj.poseArray(2,i);
                
                th = p_th + (kurv_s)*ds;
                   
                k00 = cos(p_th);
                k01 = sin(p_th);

                k10 = cos(p_th + (ds/2.0) * kurv_s);
                k11 = sin(p_th + (ds/2.0) * kurv_s);

                k20 = k10;
                k21 = k11;

                k30 = cos(th);
                k31 = sin(th);                
                x = p_x + ((ds/6.0) * (k00 + 2*(k10 + k20) + k30));
                y = p_y + ((ds/6.0) * (k01 + 2*(k11 + k21) + k31));
                
                obj.poseArray(3,i+1) = th;
                obj.poseArray(1,i+1) = x;
                obj.poseArray(2,i+1) = y;
                      
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
            xf = obj.poseArray(1,obj.numSamples);
            yf = obj.poseArray(2,obj.numSamples);
            r = max([abs(xf) abs(yf)]);
            xlim([-2*r 2*r]);
            ylim([-2*r 2*r]);
        end      
                
        function planVelocities(obj,Vmax)
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
                V = Vbase*obj.sgn; % Drive forward or backward as desired.
                K = obj.curvArray(i);
                w = K*V;
                %RobotModel Not Defined
                vr = V + RobotModel.ModelW*w;
                vl = V - RobotModel.ModelW*w;               
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
                obj.VArray(i) = (vr + vl)/2.0;
                obj.wArray(i) = (vr - vl)/RobotModel.ModelW;                
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
        
        function time = getTimeAtDist(obj, s)
            time = interp1(obj.distArray,obj.timeArray,s,'pchip','extrap');
        end
        
        function dist = getDistAtTime(obj, t)
            dist = interp1(obj.timeArray,obj.distArray,t,'pchip','extrap');
        end
               
        function pose  = getPoseAtDist(obj,s)
            x = interp1(obj.distArray,obj.poseArray(1,:),s,'pchip','extrap');
            y = interp1(obj.distArray,obj.poseArray(2,:),s,'pchip','extrap');
            th = interp1(obj.distArray,obj.poseArray(3,:),s,'pchip','extrap');
            pose  = Pose(x, y, th);  
        end
        
        function V = getVAtDist(obj,s)
            V = interp1(obj.distArray,obj.VArray,s,'pchip','extrap');
        end

        function w = getwAtDist(obj,s)
            w = interp1(obj.distArray,obj.wArray,s,'pchip','extrap');
        end
        
        function time = getTrajectoryDuration(obj)
            time = obj.timeArray(:,obj.numSamples);  
        end
        
        function dist  = getTrajectoryDistance(obj)
            dist = obj.distArray(:,obj.numSamples);  
        end
        
        function t
        
        function V  = getVAtTime(obj,t)
            if( t < obj.timeArray(1))
                V = 0.0;
            else
                V  = interp1(obj.timeArray,obj.VArray,t,'pchip','extrap');  
            end
        end
            
        function w  = getwAtTime(obj,t)
            if(t < obj.timeArray(1))
                w = 0.0;
            else
                w  = interp1(obj.timeArray,obj.wArray,t,'pchip','extrap');  
            end
        end
            
        function pose  = getPoseAtTime(obj,t)
            x = interp1(obj.timeArray,obj.poseArray(1,:),t,'pchip','extrap');
            y = interp1(obj.timeArray,obj.poseArray(2,:),t,'pchip','extrap');
            th = interp1(obj.timeArray,obj.poseArray(3,:),t,'pchip','extrap');
            pose  = Pose(x, y, th);  
        end  
        
    end
    
end