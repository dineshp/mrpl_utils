classdef TrajectoryFollowerC < handle
    
    properties(Constant)
        k_x_p = .01;
        k_y_p = .01;
        k_th_p = .01;
        UpdatePause = .01;
        t_delay = .2;
        fuseGain = .25;
    end
    
    properties(Access = public)
        robotState;
        curve;
        tform_init_w2s;
        init_pose;
        last_pose;
        global_error;
        global_dist_error;
        global_kurv_error;
        initialized = 0;
        A;
        localizer;
        bodyPts;
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)    
                
        function obj = TrajectoryFollowerC(init_pose, lml)
            obj.initialized = 0; 
            obj.init_pose = init_pose;
            obj.localizer = lml;
            obj.bodyPts = bodyGraph();
        end
        
        function [init_pose, success] = localize_nop(obj, num_iterations,bootstrap_pose)
            scan = getS;

            x_l = zeros(1,1);
            y_l = zeros(1,1);
            ref = zeros(360, 2);
            for n=1:1:360
                if(abs(scan(n)) <= 1.5 && abs(scan(n)) > .06)
                    x_l(n) = scan(n)*cos((n-1)*pi/180);
                    y_l(n) = scan(n)*sin((n-1)*pi/180);
                else
                   x_l(n) = 0;
                   y_l(n) = 0;
                end
                ref(n,1) = x_l(n);
                ref(n,2) = y_l(n);
            end  
            lidsx = (x_l ~= 0);
            lidsy = (y_l ~= 0);
            lids = lidsx .* lidsy;
            lids = logical(lids);
            x_l = x_l(lids);
            y_l = y_l(lids);
            %w_l = ones(1,length(x_l));
            %laserPtsAll = [x_l; y_l; w_l];
            %laserPtsPlotAll = obj.init_pose.bToA()*laserPtsAll;
            %scatter(laserPtsPlotAll(1,:), laserPtsPlotAll(2,:),'r');
            
            laserPtsHull = [x_l; y_l; ones(1,length(x_l));];
            ids_h = obj.localizer.throwOutliers(obj.init_pose,laserPtsHull);
            laserPtsHull(:,ids_h) = [];
            x_hull = transpose(laserPtsHull(1,:));
            y_hull = transpose(laserPtsHull(2,:));
            
            ref_hull = [x_hull y_hull];
            out=convhull(ref_hull);
            extremes=ref_hull(out,:);
            x_l_h = transpose(extremes(:,1));
            y_l_h = transpose(extremes(:,2));
            laserPtsHull = [x_l_h; y_l_h; ones(1,length(x_l_h));];
            %laserPtsHull = [x_l_h([1, floor(length(x_l_h)/2), length(x_l_h)-1]); y_l_h([1, floor(length(y_l_h)/2), length(y_l_h)-1]); ones(1,3);];
            
            w_l = ones(1,length(x_l));
            laserPts = [x_l; y_l; w_l];
            
            lml = obj.localizer;
            ids = lml.throwOutliers(obj.init_pose,laserPts);
            laserPts(:,ids) = [];

            [success,thePose] = lml.refinePose(bootstrap_pose,laserPtsHull, num_iterations);
            init_pose = bootstrap_pose;
            if(success)
                init_pose = thePose;
                obj.init_pose = thePose;
            end
        end
        

        
        function rcMode(obj, robot)
            obj.initialized = 1;

            bodyPts = obj.bodyPts;
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
            lml = obj.localizer;
            lines_p1 = obj.lml.lines_p1;
            lines_p2 = obj.lml.lines_p2;
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
        
        function [delta] = poseDiff(obj, fusedPose,lidPose)
            %static function
            x2 = lidPose.x;
            x1 = fusedPose.x;
            y2 = lidPose.y;
            y1 = fusedPose.y;
            th2 = lidPose.th;
            th1 = fusedPose.th;
            delta = [x2-x1; y2 - y1; atan2(sin(th2-th1),cos(th2-th1))];
        end

        function [fusedPose] = fusePose(obj, odoPose,lidPose)
            
            fusedPoseVec = odoPose.getPoseVec() + TrajectoryFollowerC.fuseGain*obj.poseDiff(odoPose,lidPose);
            fusedPose = Pose(fusedPoseVec(1), fusedPoseVec(2), fusedPoseVec(3));
        end
               
        function [state, i] = executeTrajectorySE(obj, robot, curve)
            %get reference to reference and actual state
            if(obj.initialized == 0)
                obj.tform_init_w2s = obj.init_pose.bToA();
                obj.global_dist_error = 0;
                obj.global_kurv_error = 0;
                obj.initialized = 1;
                dim = [.14 .6 .3 .3];
                lines_p1 = obj.localizer.lines_p1;
                lines_p2 = obj.localizer.lines_p2;
                hold on;
                plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
                hold on;
                plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
                hold on;
    
                S1 = sprintf('x_e_r_r=%f, y_e_r_r=%f, th_e_r_r=%f', 0, 0, 0);
                S2 = sprintf('s_e_r_r%f, kurv_e_r_r=%f', 0, 0);
                str = {S1, S2}; 
                %obj.A = annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on');
            end
                        
            t_f = curve.getTrajectoryDuration();
            final_pose = curve.getFinalPoseAsPose();
            if((isinf(t_f) || isnan(t_f)) || (final_pose.x == 0 && final_pose.y == 0 && final_pose.th == 0))
                state = [];
                i = 0;
                return;
            end
            n = floor(t_f/TrajectoryFollowerC.UpdatePause)+1;
            obj.robotState = RobotState(n, obj.init_pose);
            
            actual_robot = obj.robotState;
            t = actual_robot.t;
            w = actual_robot.w;
            V = actual_robot.V;
            s = actual_robot.s;
            x = actual_robot.x;
            y = actual_robot.y;
            th = actual_robot.th;
            xl = actual_robot.xl;
            yl = actual_robot.yl;
            thl = actual_robot.thl;
            
            xl_g = actual_robot.xl_g;
            yl_g = actual_robot.yl_g;
            thl_g = actual_robot.thl_g;
            x_g_act = actual_robot.x_g_act;
            y_g_act = actual_robot.y_g_act;
            th_g_act = actual_robot.th_g_act;
            x_g_ref = actual_robot.x_g_ref;
            y_g_ref = actual_robot.y_g_ref;
            th_g_ref = actual_robot.th_g_ref;
            err_x_g_ref = actual_robot.err_x_g_ref;
            err_y_g_ref = actual_robot.err_y_g_ref;
            err_th_g_ref = actual_robot.err_th_g_ref;
                                                        
            signal7 = plot(x_g_ref, y_g_ref, 'm-', 'Linewidth', 1);
            hold on;
            signal8 = plot(x_g_act, y_g_act, 'c-', 'Linewidth', 1);
            hold on;
            legend('map','map','ref', 'act');
            
            worldBodyPoints = obj.init_pose.bToA()*obj.bodyPts;
            signal9 = plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'c-', 'Linewidth', 1);
                  
            scan = getS;
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
            w_l = ones(1,length(x_l));
            laserPts = [x_l; y_l; w_l];
            laserPtsPlot = obj.init_pose.bToA()*laserPts;
            %signal10 = scatter(laserPtsPlot(1,:), laserPtsPlot(2,:), 'filled', 'r');
            hold on;
            
            worldBodyPoints = obj.init_pose.bToA()*obj.bodyPts;
            signal11 = plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'g-', 'Linewidth', 1);
            delete(signal11);
            i = actual_robot.i;
            set(signal7, 'xdata', [get(signal7,'xdata') x_g_ref(i)], 'ydata', [get(signal7,'ydata') y_g_ref(i)]);
            set(signal8, 'xdata', [get(signal8,'xdata') x_g_act(i)], 'ydata', [get(signal8,'ydata') y_g_act(i)]);
                
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
            
            pST = getST;
            cST = pST;
            while(t_i <= (t_f+TrajectoryFollowerC.t_delay+system_delay))
                i = actual_robot.i;
                
%                 cST = getST;
%                 
%                 if(cST ~= pST)
%                     pST = cST;
%                     scan = getS;
%                     x_l = zeros(1,1);
%                     y_l = zeros(1,1);
%                     ref = zeros(360, 2);
%                     for n=1:1:360
%                         if(abs(scan(n)) <= 1.5 && abs(scan(n)) > .06)
%                             x_l(n) = scan(n)*cos((n-1)*pi/180);
%                             y_l(n) = scan(n)*sin((n-1)*pi/180);
%                         else
%                            x_l(n) = 0;
%                            y_l(n) = 0;
%                         end
%                         ref(n,1) = x_l(n);
%                         ref(n,2) = y_l(n);
%                     end  
%                     lidsx = (x_l ~= 0);
%                     lidsy = (y_l ~= 0);
%                     lids = lidsx .* lidsy;
%                     lids = logical(lids);
%                     x_l = x_l(lids);
%                     y_l = y_l(lids);
%                     %w_l = ones(1,length(x_l));
%                     %laserPtsAll = [x_l; y_l; w_l];
%                     %laserPtsPlotAll = obj.init_pose.bToA()*laserPtsAll;
%                     %scatter(laserPtsPlotAll(1,:), laserPtsPlotAll(2,:),'r');
%                     fusedPose = Pose(xl_g(i), yl_g(i), thl_g(i));
%                     laserPtsHull = [x_l; y_l; ones(1,length(x_l));];
%                     ids_h = obj.localizer.throwOutliers(fusedPose,laserPtsHull);
%                     laserPtsHull(:,ids_h) = [];
%                     x_hull = transpose(laserPtsHull(1,:));
%                     y_hull = transpose(laserPtsHull(2,:));
% 
% 
%                       
%                     success = 0;
%                     if(length(x_hull) >= 3)
%                         
%                         ref_hull = [x_hull y_hull];
%                         out=convhull(ref_hull);
%                         extremes=ref_hull(out,:);
%                         x_l_h = transpose(extremes(:,1));
%                         y_l_h = transpose(extremes(:,2));
%                         laserPtsHull = [x_l_h; y_l_h; ones(1,length(x_l_h));];
%                         
%                         lml = LineMapLocalizer(obj.localizer.lines_p1,obj.localizer.lines_p2,.01,.005,.0005);
%                         [success,lidPose] = lml.refinePose(fusedPose,laserPtsHull, 1000);
%                                                          
%                         newFusedPose = obj.fusePose(fusedPose,lidPose);
%                         xl_g(i) = newFusedPose.x;
%                         yl_g(i) = newFusedPose.y;
%                         thl_g(i) = newFusedPose.th;
%                     end
%                     
% %                     if(success)
% %                         tpose_lidPose_world = lidPose.bToA();
% %                         tform_inv_robot = obj.init_pose.aToB();
% %                         tform_lidPose_robot = tform_inv_robot*tpose_lidPose_world;
% %                         lidPose_relative = Pose.matToPoseVecAsPose(tform_lidPose_robot);
% %                                       
% %                         newFusedPose = obj.fusePose(fusedPose,lidPose_relative);
% %                         xl(i) = newFusedPose.x;
% %                         yl(i) = newFusedPose.y;
% %                         thl(i) = newFusedPose.th;
% %                         
% %                         
% %                         
% %                     
% %                         delete(signal11);
% %                         worldBodyPoints = lidPose.bToA()*obj.bodyPts;
% %                         signal11 = plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'g-', 'Linewidth', 1);
% %                         hold on;
% %                     end
%                 end
                
                
                % 1. UPDATE TIME
                t_i = t_i + dt_i;

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
                tform_p_i_act_w2g = obj.init_pose.bToA()*tform_p_i_act_s2g;
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
                p_prev_l_world = Pose(xl_g(i), yl_g(i), thl_g(i));
                p_prev_l_tform = obj.init_pose.aToB()*p_prev_l_world.bToA();
                p_prev_l = Pose.matToPoseVecAsPose(p_prev_l_tform);
                
                p_i_act_l = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev_l);
                tform_p_i_act_l_s2g = p_i_act_l.bToA();
                tform_p_i_act_l_w2g = obj.init_pose.bToA()*tform_p_i_act_l_s2g;
                t_p_i_act_l = Pose.matToPoseVecAsPose(tform_p_i_act_l_w2g);
                
                xl_g(i+1) = t_p_i_act_l.x;
                yl_g(i+1) = t_p_i_act_l.y;
                thl_g(i+1) = t_p_i_act_l.th;
                
                xl(i+1) = p_i_act_l.x;
                yl(i+1) = p_i_act_l.y;
                thl(i+1) = p_i_act_l.th;
                
                p_i_ref = curve.getPoseAtTime(t_i - system_delay);
                tform_p_i_ref_s2g = p_i_ref.bToA();
                tform_p_i_ref_w2g = obj.init_pose.bToA()*tform_p_i_ref_s2g;
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
                robot.sendVelocity(v_l_U, v_r_U);

                %6. UPDATE GRAPHS 
                if(t_i >= system_delay)
                    set(signal7, 'xdata', [get(signal7,'xdata') x_g_ref(i+1)], 'ydata', [get(signal7,'ydata') y_g_ref(i+1)]);
                    set(signal8, 'xdata', [get(signal8,'xdata') x_g_act(i+1)], 'ydata', [get(signal8,'ydata') y_g_act(i+1)]);
                    worldBodyPoints = t_p_i_act.bToA()*obj.bodyPts;
                    delete(signal9);
                    signal9 = plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'c-', 'Linewidth', 1);
                    hold on;
                    %delete(signal11);
                    
                    %worldBodyPoints = t_p_i_act_l.bToA()*obj.bodyPts;
                    %signal11 = plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'g-', 'Linewidth', 1);
                    %hold on;
                    S1 = sprintf('x_e_r_r=%f, y_e_r_r=%f, th_e_r_r=%f', err_x_g_ref(i+1), err_y_g_ref(i+1), err_th_g_ref(i+1));
                    S2 = sprintf('s_e_r_r%f, kurv_e_r_r=%f', dist_error, kurv_error);
                    str = {S1, S2}; 
                    %set(obj.A, 'String', str);
                end
                
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;

                %8. DELAY MAC CLOCK
                pause(TrajectoryFollowerC.UpdatePause);
            end    
            robot.stop();
            state = [x; y; th;];
            
            i = actual_robot.i;
            p_f_ref = curve.getFinalPoseAsPose();
            
            tform_p_f_ref_s2g = p_f_ref.bToA();
            tform_p_f_ref_w2g = obj.init_pose.bToA()*tform_p_f_ref_s2g;
            t_p_f_ref = Pose.matToPoseVecAsPose(tform_p_f_ref_w2g);  
            obj.init_pose = t_p_f_ref;
            obj.tform_init_w2s = obj.init_pose.bToA();
            obj.global_dist_error = dist_error;
            obj.global_kurv_error = kurv_error; 
            obj.last_pose = Pose(x_g_act(i-1), y_g_act(i-1), th_g_act(i-1));
            delete(signal9);
        end
        
        function [state, i] = executeTrajectory(obj, robot, curve)
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
            n = floor(t_f/TrajectoryFollowerC.UpdatePause)+1;
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
                                            
            hold on;
            
            axis auto;
            title(['Reference (magenta line) & Actual (cyan line) Trajectory (x vs. y)']);
            signal7 = plot(x_g_ref, y_g_ref, 'm-', 'Linewidth', 3);
            hold on;
            signal8 = plot(x_g_act, y_g_act, 'c-', 'Linewidth', 3);
            hold on;
            xlabel('X (meters)');
            ylabel('Y (meters)');
            legend('ref', 'act');
                                             
            i = actual_robot.i;
            set(signal7, 'xdata', [get(signal7,'xdata') x_g_ref(i)], 'ydata', [get(signal7,'ydata') y_g_ref(i)]);
            set(signal8, 'xdata', [get(signal8,'xdata') x_g_act(i)], 'ydata', [get(signal8,'ydata') y_g_act(i)]);
                
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
            while(t_i <= (t_f+TrajectoryFollowerC.t_delay+system_delay))
                % 1. UPDATE TIME
                t_i = t_i + dt_i;
                i = actual_robot.i;

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
                tform_p_i_act_w2g = obj.init_pose.bToA()*tform_p_i_act_s2g;
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
                tform_p_i_ref_w2g = obj.init_pose.bToA()*tform_p_i_ref_s2g;
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
                    S1 = sprintf('x_e_r_r=%f, y_e_r_r=%f, th_e_r_r=%f', err_x_g_ref(i+1), err_y_g_ref(i+1), err_th_g_ref(i+1));
                    S2 = sprintf('s_e_r_r%f, kurv_e_r_r=%f', dist_error, kurv_error);
                    str = {S1, S2}; 
                    set(obj.A, 'String', str);
                end
                
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;

                %8. DELAY MAC CLOCK
                pause(TrajectoryFollowerC.UpdatePause);
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
