classdef TrajectoryFollowerStateE < handle
    
    properties(Constant)
        k_x_p = .01;
        k_y_p = .01;
        k_th_p = .01;
        UpdatePause = .01;
    end
    
    properties(Access = public)
        robotState;
        curve;
        tform_init_w2s;
        init_pose;
        global_error;
        global_dist_error;
        global_kurv_error;
        initialized = 0;
        A;
        localizer;
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)    
                
        function obj = TrajectoryFollowerStateE(init_pose, lml)
            obj.initialized = 0; 
            obj.init_pose = init_pose;
            obj.localizer = lml;
        end
        
        function rcMode(obj, robot)
            obj.initialized = 1;
            obj.init_pose = init_pose;
            
            bodyPts = bodyGraph();
            % Set up lines
            p1 = [0 ; 0];
            p2 = [ 48*.0254 ; 0];
            p3 = [0 ; 48*.0254 ];
            fh = figure(1);
            title(['Localization In a Known Map']);
            xlabel('X (meters)');
            ylabel('Y (meters)');
            hold on;
            rd = RobotKeypressDriver(fh);
            hold on;
            axis([-.25 1.5 -.25 1.5]);
            hold on;
            lines_p1 = [p2 p1];
            lines_p2 = [p1 p3];
            lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);
            obj.localizer = lml;
            plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
            hold on;
            plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
            hold on;
            signal8 = plot(0, 0, 'g-', 'Linewidth', 1, 'DisplayName', 'robot');
            hold on;
            %set(signal8, 'xdata', [get(signal8,'xdata') x_g_act(i)], 'ydata', [get(signal8,'ydata') y_g_act(i)]);
            signal7 = scatter(0,0, 'r', 'filled','DisplayName', 'laser');
            hold on;
            thePose = obj.init_pose;
            %legend('show');
            while(1)
                rd.drive(robot,1);
                scan = robot.laser.LatestMessage.Ranges;
                x_l = zeros(1,1);
                y_l = zeros(1,1);
                for n=1:1:360
                    if(abs(scan(n)) <= 1.5 && abs(scan(n)) > .06)
                        x_l(n) = scan(n)*cos((n-1)*pi/180);
                        y_l(n) = scan(n)*sin((n-1)*pi/180);
                    else
                       x_l(n) = 0;
                       y_l(n) = 0;
                    end
                end
                x_l = x_l(1:10:end);
                y_l = y_l(1:10:end);
                lidsx = (x_l ~= 0);
                lidsy = (y_l ~= 0);
                lids = lidsx .* lidsy;
                lids = logical(lids);
                x_l = x_l(lids);
                y_l = y_l(lids);
                w_l = ones(1,length(x_l));
                laserPts = [x_l; y_l; w_l];
                ids = lml.throwOutliers(thePose,laserPts);
                laserPts(:,ids) = [];
                [success,outPose] = lml.refinePose(thePose,laserPts, 100);
                thePose = outPose;
                worldBodyPoints = thePose.bToA()*bodyPts;
                delete(signal8);
                signal8 = plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'g-', 'Linewidth', 1);
                hold on;

                laserPtsPlot = thePose.bToA()*laserPts;
                delete(signal7);
                signal7 = scatter(laserPtsPlot(1,:), laserPtsPlot(2,:), 'filled', 'r');
                hold on;
                %disp(thePose.getPoseVec);
                pause(.05);
            end

        end
        
        function [state, i] = executeTrajectory(obj, robot, curve, t_delay)
            %get reference to reference and actual state
            if(obj.initialized == 0)
                obj.tform_init_w2s = obj.init_pose.bToA();
                obj.global_dist_error = 0;
                obj.global_kurv_error = 0;
                obj.initialized = 1;
                dim = [.14 .6 .3 .3];
                S1 = sprintf('x_e_r_r=%f, y_e_r_r=%f, th_e_r_r=%f', 0, 0, 0);
                S2 = sprintf('s_e_r_r%f, kurv_e_r_r=%f', 0, 0);
                str = {S1, S2}; 
                obj.A = annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on');
            end
                        
            t_f = curve.getTrajectoryDuration();
            final_pose = curve.getFinalPoseAsPose();
            if((isinf(t_f) || isnan(t_f)) || (final_pose.x == 0 && final_pose.y == 0 && final_pose.th == 0))
                state = [];
                i = 0;
                return;
            end
            n = floor(t_f/TrajectoryFollowerStateE.UpdatePause)+1;
            obj.robotState = RobotState(n, obj.init_pose);
            
            actual_robot = obj.robotState;
            t = actual_robot.t;
            w = actual_robot.w;
            V = actual_robot.V;
            s = actual_robot.s;
            x = actual_robot.x;
            y = actual_robot.y;
            th = actual_robot.th;
            
            x_g_act = actual_robot.x_g_act;
            y_g_act = actual_robot.y_g_act;
            th_g_act = actual_robot.th_g_act;
            x_g_ref = actual_robot.x_g_ref;
            y_g_ref = actual_robot.y_g_ref;
            th_g_ref = actual_robot.th_g_ref;
            err_x_g_ref = actual_robot.err_x_g_ref;
            err_y_g_ref = actual_robot.err_y_g_ref;
            err_th_g_ref = actual_robot.err_th_g_ref;
            x_g_lid = actual_robot.x_g_lid;
            y_g_lid = actual_robot.y_g_lid;
            th_g_lid = actual_robot.th_g_lid;
            
            hold on;
            
            axis auto;
            title(['Reference (magenta line) & Actual (cyan line) Trajectory (x vs. y)']);
            signal7 = plot(x_g_ref, y_g_ref, 'm-', 'Linewidth', 3);
            hold on;
            signal8 = plot(x_g_act, y_g_act, 'c-', 'Linewidth', 3);
            hold on;
            signal9 = plot(x_g_lid, y_g_lid, 'g-', 'Linewidth', 3);
            hold on;
            xlabel('X (meters)');
            ylabel('Y (meters)');
            legend('ref', 'act', 'lid');
                                             
            i = actual_robot.i;
            
            set(signal7, 'xdata', [get(signal7,'xdata') x_g_ref(i)], 'ydata', [get(signal7,'ydata') y_g_ref(i)]);
            set(signal8, 'xdata', [get(signal8,'xdata') x_g_act(i)], 'ydata', [get(signal8,'ydata') y_g_act(i)]);
            set(signal9, 'xdata', [get(signal9,'xdata') x_g_lid(i)], 'ydata', [get(signal9,'ydata') y_g_lid(i)]);
                    
            
            kurv_error = 0;
            dist_error = 0;
            t_i = 0;
            dt_i = 0;
            prevX = getX;
            prevY = getY;
            pT = getT;
            cT = pT;
            curX = prevX;
            curY = prevY;
            sf = curve.distArray(end);
            system_delay = 1.5;
            
            lml = obj.localizer;
            %legend('show');
            
            
            while(t_i <= (t_f+t_delay+system_delay))
                
                
                
                % 1. UPDATE TIME
                t_i = t_i + dt_i;
                i = actual_robot.i;
                
                
                scan = robot.laser.LatestMessage.Ranges;
                x_l = zeros(1,1);
                y_l = zeros(1,1);
                for n=1:1:360
                    if(abs(scan(n)) <= 1.5 && abs(scan(n)) > .06)
                        x_l(n) = scan(n)*cos((n-1)*pi/180);
                        y_l(n) = scan(n)*sin((n-1)*pi/180);
                    else
                       x_l(n) = 0;
                       y_l(n) = 0;
                    end
                end
                x_l = x_l(1:10:end);
                y_l = y_l(1:10:end);
                lidsx = (x_l ~= 0);
                lidsy = (y_l ~= 0);
                lids = lidsx .* lidsy;
                lids = logical(lids);
                x_l = x_l(lids);
                y_l = y_l(lids);
                w_l = ones(1,length(x_l));
                laserPts = [x_l; y_l; w_l];
                p_lid = Pose(x_g_act(i), y_g_act(i), th_g_act(i));
                ids = lml.throwOutliers(p_lid,laserPts);
                laserPts(:,ids) = [];
                [success,outPose] = lml.refinePose(p_lid,laserPts, 100);
                x_g_lid(i+1) = outPose.x;
                y_g_lid(i+1) = outPose.y;
                th_g_lid(i+1) = outPose.th;
        
                % 2. WAIT FOR ENCODER CHANGE
                count = 0;
                while(eq(cT, pT))     
                    if(count > 1000)
                        S = sprintf('Encoders not changing at cT=%f, pT=%f, curX%f, curY%f, t_i=%f', cT, pT, curX, curY, t_i);
                        errorStruct.message = S;
                        errorStruct.identifier = 'TRJFC:ENCODER_ISSUE';
                        error(errorStruct);
                        break;
                    end
                    count = count + 1;
                    cT = getT;
                    curX = getX;
                    curY = getY;
                    pause(.001);
                end

                % 3. UPDATE STATE (DEAD RECKONING)
                dt_i = cT - pT; 
                vl_i = 0;
                vr_i = 0;
                if(dt_i > 0)
                    vl_i = (curX-prevX)/dt_i;
                    vr_i = (curY-prevY)/dt_i;
                end
                pT = cT;
                prevX = curX;
                prevY = curY;

                [V_i , w_i] = RobotModelAdv.vlvrToVw(vl_i, vr_i);
                ds_i = V_i*dt_i;
                p_prev = Pose(x(i), y(i), th(i));
                p_i_act = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev);
                
                tform_p_i_act_s2g = p_i_act.bToA();
                tform_p_i_act_w2g = obj.tform_init_w2s*tform_p_i_act_s2g;
                t_p_i_act = Pose.matToPoseVecAsPose(tform_p_i_act_w2g);
                x_g_act(i+1) = t_p_i_act.x;
                y_g_act(i+1) = t_p_i_act.y;
                th_g_act(i+1) = t_p_i_act.th;
                t(i+1) = t_i;
                V(i+1) = V_i;
                w(i+1) = w_i;
                s(i+1) = s(i) + (V_i*dt_i);
                x(i+1) = p_i_act.x;
                y(i+1) = p_i_act.y;
                th(i+1) = p_i_act.th;
                
                p_i_ref = curve.getPoseAtTime(t_i - system_delay);
                tform_p_i_ref_s2g = p_i_ref.bToA();
                tform_p_i_ref_w2g = obj.tform_init_w2s*tform_p_i_ref_s2g;
                t_p_i_ref = Pose.matToPoseVecAsPose(tform_p_i_ref_w2g);
                x_g_ref(i+1) = t_p_i_ref.x;
                y_g_ref(i+1) = t_p_i_ref.y;
                th_g_ref(i+1) = t_p_i_ref.th;           
                
                %compute error
                r_r_p = t_p_i_act.aToB()*(t_p_i_ref.getPoseVec() - t_p_i_act.getPoseVec());
                err_x_g_ref(i+1) = r_r_p(1);
                err_y_g_ref(i+1) = r_r_p(2);
                err_th_g_ref(i+1) = r_r_p(3);
                kurv_i = 0;
                if(ds_i ~= 0)    
                    kurv_i = w_i*dt_i/ds_i;
                end
                kurv_i_ref = curve.getCurvAtDist(s(i+1));
                kurv_error = kurv_i_ref - kurv_i + obj.global_kurv_error;
                dist_error = curve.getDistAtTime(t_i) - s(i+1) + obj.global_dist_error;
              
                % 4. UPDATE CONTROL             
                %get velocity from open loop 
                V_i = curve.getVAtTime(t_i - system_delay);
                w_i = curve.getwAtTime(t_i - system_delay);
                %V_i_s = curve.getVAtDist(s(i));
                
                %V_i = max(V_i, V_i_s);
                                 
                [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i, w_i);
                [v_l_U , v_r_U] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);
                
                %5. SEND CONTROL TO ROBOT
                if((s(i+1) >= sf))
                    break;
                else
                    robot.sendVelocity(v_l_U, v_r_U);
                end

                %6. UPDATE GRAPHS 
                if(t_i >= system_delay)
                    set(signal7, 'xdata', [get(signal7,'xdata') x_g_ref(i+1)], 'ydata', [get(signal7,'ydata') y_g_ref(i+1)]);
                    set(signal8, 'xdata', [get(signal8,'xdata') x_g_act(i+1)], 'ydata', [get(signal8,'ydata') y_g_act(i+1)]);
                    set(signal9, 'xdata', [get(signal9,'xdata') x_g_lid(i+1)], 'ydata', [get(signal9,'ydata') y_g_lid(i+1)]);
                    S1 = sprintf('x_e_r_r=%f, y_e_r_r=%f, th_e_r_r=%f', err_x_g_ref(i+1), err_y_g_ref(i+1), err_th_g_ref(i+1));
                    S2 = sprintf('s_e_r_r%f, kurv_e_r_r=%f', dist_error, kurv_error);
                    str = {S1, S2}; 
                    set(obj.A, 'String', str);
                end
                
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;

                %8. DELAY MAC CLOCK
                pause(TrajectoryFollowerStateE.UpdatePause);
            end    
            robot.stop();
            state = [x; y; th;];
            
            i = actual_robot.i;
            p_f_ref = curve.getFinalPoseAsPose();
            
            tform_p_f_ref_s2g = p_f_ref.bToA();
            tform_p_f_ref_w2g = obj.tform_init_w2s*tform_p_f_ref_s2g;
            t_p_f_ref = Pose.matToPoseVecAsPose(tform_p_f_ref_w2g);  
            obj.init_pose = t_p_f_ref;
            obj.tform_init_w2s = obj.init_pose.bToA();
            obj.global_dist_error = dist_error;
            obj.global_kurv_error = kurv_error;
        end
    end

    methods(Access = private)

    end
    
    
end
