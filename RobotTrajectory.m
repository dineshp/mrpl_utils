classdef RobotTrajectory < handle

    properties(Constant)
        dt = .001;
    end
    
    properties(Access = public)
        robotState
        refC
        refCRobotCommands
        numSamples
        t_f
        t_f_robot_commands
    end

    properties(Access = private)

    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)  
        
        function plotIntDensityTraj(obj)
            n = obj.numSamples;
            x_g = zeros(1,n);
            y_g = zeros(1,n);
            fig1 = plot(x_g, y_g, 'b');
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            xlabel('X');
            ylabel('Y');
            title(['Simulated Trajectory at integration density dt = ' num2str(RobotTrajectory.dt) '']);
            
            for j=1:1:n
                x_g(j) = obj.robotState.x(j);
                y_g(j) = obj.robotState.y(j);
                set(fig1, 'xdata', [get(fig1,'xdata') -y_g(j)], 'ydata', [get(fig1,'ydata') x_g(j)]);
            end
        end
        
        function plotInterpTraj(obj, n)
            dt_i = obj.t_f/(n-1);
            
            x_g = zeros(1,n);
            y_g = zeros(1,n);
            fig1 = plot(x_g, y_g, 'b-o');
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            xlabel('X');
            ylabel('Y');
            title(['Simulated Trajectory at n = ' num2str(n) ' interpolated points']);
            
            x_g(1) = 0;
            y_g(1) = 0; 
            for i=1:1:n-1
                t_i = i*dt_i;
                p = obj.getPoseAtTime(t_i);
                x_g(i+1) = p.x;
                y_g(i+1) = p.y;  
                set(fig1, 'xdata', [get(fig1,'xdata') -y_g(i)], 'ydata', [get(fig1,'ydata') x_g(i)]);
            end
        end
        
        function obj = RobotTrajectory(refC, refCRobotCommands)
            obj.refC = refC;
            obj.refCRobotCommands = refCRobotCommands;
            obj.t_f = refC.getTrajectoryDuration;
            obj.t_f_robot_commands = refCRobotCommands.getTrajectoryDuration;
            obj.numSamples = floor(obj.t_f/RobotTrajectory.dt)+1;
            n = obj.numSamples;
            
            obj.robotState = RobotState(n);
  
            for i=1:1:n-1
                t_i = (i-1)*RobotTrajectory.dt;
                obj.robotState.t(i) = t_i;
                
                [V_i_ref, w_i_ref] = refC.computeControl(t_i);
                obj.robotState.V(i) = V_i_ref;
                obj.robotState.w(i) = w_i_ref;
        
                obj.robotState.s(i+1) = obj.robotState.s(i) + (V_i_ref*RobotTrajectory.dt);
                
                p_i = Pose(obj.robotState.x(i), obj.robotState.y(i), obj.robotState.th(i));
                p_i_plus1 = RobotModelAdv.integrateDiffEq(V_i_ref, w_i_ref, RobotTrajectory.dt, p_i);
                obj.robotState.x(i+1) = p_i_plus1.x;
                obj.robotState.y(i+1) = p_i_plus1.y;
                obj.robotState.th(i+1) = p_i_plus1.th;
            end
            
            t_n = obj.robotState.t(n-1) + RobotTrajectory.dt;
            obj.robotState.t(n) = t_n;
            [V_n_ref, w_n_ref] = refC.computeControl(t_n);
            obj.robotState.V(n) = V_n_ref;
            obj.robotState.w(n) = w_n_ref;
        end
        
        function pose = getPoseAtTime(obj,index)  
            x_t = interp1(obj.robotState.t, transpose(obj.robotState.x), index);
            y_t = interp1(obj.robotState.t, transpose(obj.robotState.y), index);
            th_t = interp1(obj.robotState.t, transpose(obj.robotState.th), index);
            pose = Pose(x_t,y_t,th_t);
        end
        
        function [V, w] = getVelocitiesAtTime(obj,timeNow)
            [V, w] = obj.refCRobotCommands.computeControl(timeNow);
        end
    end

    methods(Access = private)

    end
    
    
end

