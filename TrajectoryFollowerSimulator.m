classdef TrajectoryFollowerSimulator

    properties(Constant)
        UpdatePause = .05;
    end
    
    properties(Access = public)
        robotState;
        fig1;
        fig2;
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)       
        function obj = TrajectoryFollowerSimulator(robotTrajectoryModel, refC)
                   
            t_f = refC.getTrajectoryDuration; %add extra second
            n = floor(t_f/TrajectoryFollowerSimulator.UpdatePause)+1;
            obj.robotState = RobotState(n);
            
            x_g_ref = zeros(1,n);
            y_g_ref = zeros(1,n);
            obj.fig1 = plot(x_g_ref, y_g_ref, 'k');
            hold on;
                                    
            x_g_act = zeros(1,n);
            y_g_act = zeros(1,n);
            obj.fig2 = plot(x_g_act, y_g_act, 'b-o');
            hold on;
                        
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            xlabel('X');
            ylabel('Y');
            title(['Estimated (black) & Reference (blue) Robot Trajectory']);
                    
            t = 0;
            tStart = 0;
            dt_i = 0;
            while(1)
                firstIteration = 0;
                if(firstIteration == 0)
                    t = 0;
                    tStart = tic;
                    dt_i = toc(tStart);
   
                    obj.robotState.iPlusPlus;
                   
                    
                    
                    
                    
                    
                    firstIteration = 1;
                end
                
                
                while(t <= t_f)
                    t = t + dt_i;
                    obj.robotState.t(obj.robotState.i) = t;
                    
                               
                    dt_i = toc(tStart); 
                    [u_i_V, u_i_w] = refC.computeControl(t);
                    [vl_i, vr_i] = RobotModelAdv.VwTovlvr(u_i_V, u_i_w);


                    [V_i , w_i] = RobotModelAdv.vlvrToVw(vl_i, vr_i);
                    
                    obj.robotState.V(obj.robotState.i) = V_i;
                    obj.robotState.w(obj.robotState.i) = w_i;

                    obj.robotState.s(obj.robotState.i) = obj.robotState.s(obj.robotState.i-1) + (V_i*dt_i);

                    p_prev = Pose(obj.robotState.x(obj.robotState.i-1), obj.robotState.y(obj.robotState.i-1), obj.robotState.th(obj.robotState.i-1));
                    p_i_act = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev);
                    obj.robotState.x(obj.robotState.i) = p_i_act.x;
                    obj.robotState.y(obj.robotState.i) = p_i_act.y;
                    obj.robotState.th(obj.robotState.i) = p_i_act.th;
                    
                    p_i_del = robotTrajectoryModel.getPoseAtTime(t);
                                      
                    [u_ref_V, u_ref_w] = refC.computeControl(t);
                    %compute error and proportional control here
                    u_p_V = 0;
                    u_p_w = 0;
                    
                    V_i = u_ref_V + u_p_V;
                    w_i = u_ref_w + u_p_w;
                    [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i, w_i);
                    
                    
                    %robot.sendVelocity(v_l_U, v_r_U); 
                    tStart = tic;
                    
                    x_g_ref(obj.robotState.i) = p_i_del.x;
                    y_g_ref(obj.robotState.i) = p_i_del.y;
                    set(obj.fig1, 'xdata', [get(obj.fig1,'xdata') -y_g_ref(obj.robotState.i)], 'ydata', [get(obj.fig1,'ydata') x_g_ref(obj.robotState.i)]);
                    
                    x_g_act(obj.robotState.i) = p_i_act.x;
                    y_g_act(obj.robotState.i) = p_i_act.y;
                    set(obj.fig2, 'xdata', [get(obj.fig2,'xdata') -y_g_act(obj.robotState.i)], 'ydata', [get(obj.fig2,'ydata') x_g_act(obj.robotState.i)]);

                    obj.robotState.iPlusPlus;
                    
                    pause(TrajectoryFollowerSimulator.UpdatePause);
                end
                %robot.stop();
                break;
            end
           
        end
    end

    methods(Access = private)

    end
    
    
end

