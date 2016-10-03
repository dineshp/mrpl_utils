classdef TrajectoryFollower

    properties(Constant)
        UpdatePause = .05;
    end
    
    properties(Access = public)
        robotState;
        controller;
        fig1;
        fig2;
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)       
        function obj = TrajectoryFollower(robot, robotTrajectoryModel, controller)
            obj.controller = controller;    
            t_f = robotTrajectoryModel.t_f + 1; %add extra second
            n = floor(t_f/TrajectoryFollower.UpdatePause)+1;
            obj.robotState = RobotState(n);
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            x_g_ref = zeros(1,n);
            y_g_ref = zeros(1,n);
            subplot(2,2,1);
            obj.fig1 = plot(x_g_ref, y_g_ref, 'k-');
            hold on;
                                    
            x_g_act = zeros(1,n);
            y_g_act = zeros(1,n);
            obj.fig2 = plot(x_g_act, y_g_act, 'b-o');
            hold on;
                        
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            xlabel('X');
            ylabel('Y');
            title(['Estimated Robot (blue) & Reference (black) Trajectory']);
                  
            subplot(2,2,2);
            axis auto;
            hold on;
            xlabel('Time');
            ylabel('Error in mm');
            title(['Error between Estimated Robot & Reference Trajectory']);
            
            subplot(2,2,3);
            axis auto;
            hold on;
            xlabel('Time');
            ylabel('Error in degrees');
            title(['Error between Estimated Robot Heading & Reference Heading']);
            
            t = 0;
            tStart = 0;
            dt_i = 0;
            prevX = getX;
            prevY = getY;
            pT = getT;
            cT = pT;
            curX = prevX;
            curY = prevY;
            firstIteration = 0;
            while(t <= t_f)
                if(firstIteration == 0)
                    while(eq(cT, pT))     
                        cT = getT;
                        curX = getX;
                        curY = getY;
                        pause(.001);
                    end
                    dt_i = cT - pT; 
                    firstIteration = 1;
                    obj.robotState.iPlusPlus;
                end
                             
                t = t + dt_i;
                obj.robotState.t(obj.robotState.i) = t;
                count = 0;
                while(eq(cT, pT))     
                    count = count + 1;
                    cT = getT;
                    curX = getX;
                    curY = getY;
                    pause(.001);
                end

                dt_i = cT - pT;  
                vl_i = (curX-prevX)/dt_i;
                vr_i = (curY-prevY)/dt_i;
                pT = cT;
                prevX = curX;
                prevY = curY;

                [V_i , w_i] = RobotModelAdv.vlvrToVw(vl_i, vr_i);

                obj.robotState.V(obj.robotState.i) = V_i;
                obj.robotState.w(obj.robotState.i) = w_i;

                obj.robotState.s(obj.robotState.i) = obj.robotState.s(obj.robotState.i-1) + (V_i*dt_i);

                p_prev = Pose(obj.robotState.x(obj.robotState.i-1), obj.robotState.y(obj.robotState.i-1), obj.robotState.th(obj.robotState.i-1));
                p_i_act = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev);
                obj.robotState.x(obj.robotState.i) = p_i_act.x;
                obj.robotState.y(obj.robotState.i) = p_i_act.y;
                obj.robotState.th(obj.robotState.i) = p_i_act.th;

                p_i_ref = robotTrajectoryModel.getPoseAtTime(t);

                [u_ref_V, u_ref_w] = robotTrajectoryModel.getVelocitiesAtTime(t);
                %compute error and proportional control here
                [u_p_V, u_p_w] = obj.controller.feedback(p_i_act,p_i_ref,dt_i);  
                V_i = u_ref_V + u_p_V;
                w_i = u_ref_w + u_p_w;
                [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i, w_i);
                [v_l_U , v_r_U] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);

                robot.sendVelocity(v_l_U, v_r_U); 

                subplot(2,2,1);
                x_g_ref(obj.robotState.i) = p_i_ref.x;
                y_g_ref(obj.robotState.i) = p_i_ref.y;
                set(obj.fig1, 'xdata', [get(obj.fig1,'xdata') -y_g_ref(obj.robotState.i)], 'ydata', [get(obj.fig1,'ydata') x_g_ref(obj.robotState.i)]);

                x_g_act(obj.robotState.i) = p_i_act.x;
                y_g_act(obj.robotState.i) = p_i_act.y;
                set(obj.fig2, 'xdata', [get(obj.fig2,'xdata') -y_g_act(obj.robotState.i)], 'ydata', [get(obj.fig2,'ydata') x_g_act(obj.robotState.i)]);

                
                errorD = sqrt(((p_i_ref.x - p_i_act.x)^2 + (p_i_ref.y - p_i_act.y)^2));
                subplot(2,2,2);
                scatter(t, errorD*1000, 'X');
               
                subplot(2,2,3);
                scatter(t, rad2deg(p_i_ref.th-p_i_act.th), 'X');
                
                obj.robotState.iPlusPlus;

                pause(max(0, (TrajectoryFollower.UpdatePause-(count*.001))));
            end
            robot.stop();
        end
    end

    methods(Access = private)

    end
    
    
end

