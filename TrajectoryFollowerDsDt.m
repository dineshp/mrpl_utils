classdef TrajectoryFollowerDsDt
    
    properties(Constant)
        k_x_p = .5;
        k_y_p = .5;
        k_th_p = .5;
        UpdatePause = .01;
    end
    
    properties(Access = public)
        curve;
        enableFeedback;
        
        distArray = [];
        timeArray = [];
        poseArray = [];
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)  
        function obj = TrajectoryFollowerDsDt(curve, enableFeedback)
            %constructor  
            obj.enableFeedback = enableFeedback;
            obj.curve = curve;
        end
        
        function [u_V, u_w] = feedback(obj,p_i_act,p_i_ref)
             r_r_p = p_i_act.aToB()*(p_i_ref.getPoseVec() - p_i_act.getPoseVec());
             u_p = [obj.k_x_p 0 0; 0 obj.k_y_p obj.k_th_p]*r_r_p;
             u_V = u_p(1);
             u_w = u_p(2);             
        end
        
        function executeTrajectory(obj, robot)            
            curve = obj.curve;
            
            t_f = curve.getTrajectoryDuration();
            s_f = curve.getTrajectoryDistance();
            
            maxState = floor(t_f/TrajectoryFollower.UpdatePause)+1;
            obj.distArray = zeros(1, maxState);
            obj.timeArray = zeros(1, maxState);
            obj.poseArray = zeros(3, maxState);
            t = obj.timeArray;
            s = obj.distArray;
            x = obj.poseArray(1,:);
            y = obj.poseArray(2,:);
            th = obj.poseArray(3,:);
            
            x_g_ref = zeros(1,maxState);
            y_g_ref = zeros(1,maxState);
            th_g_ref = zeros(1,maxState);
            err_x_g_ref = zeros(1,maxState);
            err_y_g_ref = zeros(1,maxState);
            err_th_g_ref = zeros(1,maxState);
            
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            hold on;
            signal1 = plot(t, x_g_ref, 'm-^', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal2 = plot(t, y_g_ref, 'm-p', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal3 = plot(t, th_g_ref, 'm-o', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal4 = plot(t, x, 'c-^', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal5 = plot(t, y, 'c-p', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal6 = plot(t, th, 'c-o', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            axis auto;
            xlabel('Time');
            ylabel('X Y TH');
            legend('x_r_e_f', 'y_r_e_f', 'th_r_e_f', 'x_a_c_t', 'y_a_c_t', 'th_a_c_t');
            title(['Reference (magenta) & Actual (cyan) Trajectory']);                                         
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            hold on;
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title(['Reference (magenta circles) & Actual (cyan line) Trajectory (x vs. y) in world coord.']);
            signal7 = plot(x_g_ref, y_g_ref, 'm-o', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal8 = plot(x, y, 'c-', 'Linewidth', 1);
            hold on;
            xlabel('X');
            ylabel('Y');
            legend('ref', 'act');           
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            hold on;
            axis auto;
            title(['Error in body coord']);
            signal9 = plot(t, err_x_g_ref, 'r-^', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal10 = plot(t, err_y_g_ref, 'r-p', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            signal11 = plot(t, err_th_g_ref, 'r-o', 'Linewidth', 1, 'MarkerSize', 10);
            xlabel('Time');
            ylabel('X Y TH');
            legend('x_e_r_r', 'y_e_r_r', 'th_e_r_r');
            
            set(signal1, 'xdata', [get(signal1,'xdata') t(1)], 'ydata', [get(signal1,'ydata') x_g_ref(1)]);
            set(signal2, 'xdata', [get(signal2,'xdata') t(1)], 'ydata', [get(signal2,'ydata') y_g_ref(1)]);
            set(signal3, 'xdata', [get(signal3,'xdata') t(1)], 'ydata', [get(signal3,'ydata') th_g_ref(1)]);
            set(signal4, 'xdata', [get(signal4,'xdata') t(1)], 'ydata', [get(signal4,'ydata') x(1)]);
            set(signal5, 'xdata', [get(signal5,'xdata') t(1)], 'ydata', [get(signal5,'ydata') y(1)]);
            set(signal6, 'xdata', [get(signal6,'xdata') t(1)], 'ydata', [get(signal6,'ydata') th(1)]) 
            set(signal7, 'xdata', [get(signal7,'xdata') x_g_ref(1)], 'ydata', [get(signal7,'ydata') y_g_ref(1)]);
            set(signal8, 'xdata', [get(signal8,'xdata') x(1)], 'ydata', [get(signal8,'ydata') y(1)]);
            set(signal9, 'xdata', [get(signal9,'xdata') t(1)], 'ydata', [get(signal9,'ydata') err_x_g_ref(1)]);
            set(signal10, 'xdata', [get(signal10,'xdata') t(1)], 'ydata', [get(signal10,'ydata') err_y_g_ref(1)]);
            set(signal11, 'xdata', [get(signal11,'xdata') t(1)], 'ydata', [get(signal11,'ydata') err_th_g_ref(1)]);
            
            ds_i = 0;
            dt_i = 0;
            prevX = getX;
            prevY = getY;
            pT = getT;
            curX = prevX;
            curY = prevY;
            cT = pT;
            i = 1;
            firstIteration = false;
            t_delay = 0;
            while((t(i) <= t_f))              
                i = i + 1;
                
                if(firstIteration == false)               
                    while(eq(cT, pT))     
                        cT = getT;
                        curX = getX;
                        curY = getY;
                        pause(.001);
                    end
                    
                    %WARNING, THIS IS HACK-Y
                    firstIteration = true;
                    ds_i = .001;
                else
                    % 1. UPDATE DISTANCE
                    s(i) = s(i-1) + ds_i;
                    t(i) = t(i-1) + dt_i;

                    % 2. WAIT FOR ENCODER CHANGE
                    while(eq(cT, pT))     
                        cT = getT;
                        curX = getX;
                        curY = getY;
                        pause(.001);
                    end
                    dt_i = cT - pT; 
                    
                    dsl_i = (curX-prevX);
                    dsr_i = (curY-prevY);
                    ds_i = (dsl_i + dsr_i)/2.0;

                    pT = cT;
                    prevX = curX;
                    prevY = curY;

                    % 3. UPDATE STATE (DEAD RECKONING)
                    V_i = ds_i/dt_i;
                    w_i = (dsr_i - dsl_i)/(dt_i*RobotModelAdv.ModelW);
                    p_prev = Pose(x(i-1), y(i-1), th(i-1));
                    p_i_act = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev);

                    if(t(i) > t_f)
                        fprintf('t_i - t_f = %f\n',t(i)-t_f);
                    end
                    x(i) = p_i_act.x;
                    y(i) = p_i_act.y;
                    th(i) = p_i_act.th;

                    % 4. UPDATE CONTROL
                    p_i_ref = curve.getPoseAtDist(s(i));

                    %get velocity from open loop 
                    u_ref_V = curve.getVAtTime(t(i)-.6);
                    u_ref_w = curve.getwAtTime(t(i)-.6);
                    [u_p_V, u_p_w] = obj.feedback(p_i_act,p_i_ref);
                    V_i_U = u_ref_V + (obj.enableFeedback*u_p_V);
                    w_i_U = u_ref_w + (obj.enableFeedback*u_p_w);

                    [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i_U, w_i_U);
                    [v_l_U_lim , v_r_U_lim] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);
                    
                    if(v_l_U == 0 && v_r_U == 0)
                        fprintf('at time=%f and distance=%s, we are sending zero velocities to robot', t(i), s(i));
                    else    
                        robot.sendVelocity(v_l_U_lim, v_r_U_lim);
                    end
                    
                    %6. UPDATE GRAPHS
                    x_g_ref(i) = p_i_ref.x;
                    y_g_ref(i) = p_i_ref.y;
                    th_g_ref(i) = p_i_ref.th; 
                    set(signal1, 'xdata', [get(signal1,'xdata') t(i)], 'ydata', [get(signal1,'ydata') x_g_ref(i)]);
                    set(signal2, 'xdata', [get(signal2,'xdata') t(i)], 'ydata', [get(signal2,'ydata') y_g_ref(i)]);
                    set(signal3, 'xdata', [get(signal3,'xdata') t(i)], 'ydata', [get(signal3,'ydata') th_g_ref(i)]);
                    set(signal4, 'xdata', [get(signal4,'xdata') t(i)], 'ydata', [get(signal4,'ydata') x(i)]);
                    set(signal5, 'xdata', [get(signal5,'xdata') t(i)], 'ydata', [get(signal5,'ydata') y(i)]);
                    set(signal6, 'xdata', [get(signal6,'xdata') t(i)], 'ydata', [get(signal6,'ydata') th(i)]) 
                    set(signal7, 'xdata', [get(signal7,'xdata') x_g_ref(i)], 'ydata', [get(signal7,'ydata') y_g_ref(i)]);
                    set(signal8, 'xdata', [get(signal8,'xdata') x(i)], 'ydata', [get(signal8,'ydata') y(i)]);
                    %computer error in body coord.
                    r_r_p = p_i_act.aToB()*(p_i_ref.getPoseVec() - p_i_act.getPoseVec());
                    err_x_g_ref(i) = r_r_p(1);
                    err_y_g_ref(i) = r_r_p(2);
                    err_th_g_ref(i) = r_r_p(3);
                    set(signal9, 'xdata', [get(signal9,'xdata') t(i)], 'ydata', [get(signal9,'ydata') err_x_g_ref(i)]);
                    set(signal10, 'xdata', [get(signal10,'xdata') t(i)], 'ydata', [get(signal10,'ydata') err_y_g_ref(i)]);
                    set(signal11, 'xdata', [get(signal11,'xdata') t(i)], 'ydata', [get(signal11,'ydata') err_th_g_ref(i)]);
                    
                end

                %7. DELAY MAC CLOCK
                pause(TrajectoryFollower.UpdatePause);
            end
            robot.stop();
        end
        
    end
    
    
    
end