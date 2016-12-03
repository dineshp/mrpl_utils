classdef TrajectoryFollower < handle
    
    properties(Constant)
%         k_x_p = 0; %robot 7
%         k_y_p = 0;
%         k_th_p = 0;
        k_x_p = 0;
        k_y_p = 1.5;
        k_th_p = .6;
        UpdatePause = .01;
        t_delay = 0;
        %t_delay = .2;
        system_delay = 1.5;
        gain = 0.01;
        errThresh = 0.001;
        gradThresh = 0.0005;
    end
    
    properties(Access = public)
        robotState;
        last_pose;
        localized = 0;
        initialized = 0;
        localizer;
        A;
        bodyPts;
        lines_p1 = [];
        lines_p2 = [];
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)    
                
        function obj = TrajectoryFollower()
            obj.initialized = 0;
        end

        function [u_V, u_w] = feedback(obj,p_i_act,p_i_ref)
             r_r_p = p_i_act.aToB()*(p_i_ref.getPoseVec() - p_i_act.getPoseVec());
             u_p = [obj.k_x_p 0 0; 0 obj.k_y_p obj.k_th_p]*r_r_p;
             u_V = u_p(1);
             u_w = u_p(2);             
        end
                
        function goal_pose_robot = world_to_robot(obj, goal_pose)
            tform_goal_pose_robot = obj.last_pose.aToB()*goal_pose.bToA();
            goal_pose_robot = Pose.matToPoseVecAsPose(tform_goal_pose_robot);
        end
            
        function curr_pose_world = robot_to_world(obj, curr_pose_robot)
            tform_curr_pose_world = obj.last_pose.bToA()*curr_pose_robot.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
        end
        
        function fig = init(obj)
            fig = figure(1);
            obj.initialized = 0;
            obj.localized = 0;
        end
        
        function [rc, thePose] = localizeRT(obj, scan, thePose)

        p1 = [0 ; 0];
        p2 = [ 48*.0254 ; 0];
        p3 = [0 ; 48*.0254 ];
        p4 = [48*.0254; 48*.0254];
        lines_p1 = [p2 p1 p2];
        lines_p2 = [p1 p3 p4];
            lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);

            x_l = zeros(1,1);
            y_l = zeros(1,1);
            for n=1:1:360
                if(abs(scan(n)) <= 1.3 && abs(scan(n)) > .06)
                    x_l(n) = scan(n)*cos((n-1)*pi/180);
                    y_l(n) = scan(n)*sin((n-1)*pi/180);
                else
                   x_l(n) = 0;
                   y_l(n) = 0;
                end
            end
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
            x_l = laserPts(1,:);
            y_l = laserPts(2,:);


            if(length(laserPts(1,:)) < 3)
                rc = 0;
                
                disp('pose estimate is way off, all points were thrown as outliers');
                return;
            end
            start = tic;
            try
%                 x_hull = transpose(laserPts(1,:));
%                 y_hull = transpose(laserPts(2,:));
%                 ref_hull = [x_hull y_hull];
%                 out=convhull(ref_hull);
%                 %disp(out2);
%                 extremes=ref_hull(out,:);
%                 x_l_h = transpose(extremes(:,1));
%                 y_l_h = transpose(extremes(:,2));
%                 laserPts = [x_l_h; y_l_h; ones(1,length(x_l_h));];
                x_l = x_l(1:1:end);
                y_l = y_l(1:1:end);
                w_l = ones(1,length(x_l));
                laserPts = [x_l; y_l; w_l];
                [rc,outPose] = lml.refinePose(thePose,laserPts, 100000);
            catch
                disp('localizer failed, trying again');
                x_l = x_l(1:1:end);
                y_l = y_l(1:1:end);
                w_l = ones(1,length(x_l));
                laserPts = [x_l; y_l; w_l];
                [rc,outPose] = lml.refinePose(thePose,laserPts, 100000);
            end


            %disp(toc(start));
            if(rc)
                thePose = outPose;
                %plotRobotAnotate(thePose);
            else
                disp('localization not successful');
                return;
            end
        
        end
        
        function [rc, thePose] = localize(obj, scan, thePose)

            p1 = [0 ; 0];
            p2 = [ 48*.0254 ; 0];
            p3 = [0 ; 48*.0254 ];
            p4 = [48*.0254; 48*.0254];
            lines_p1 = [p2 p1 p2];
            lines_p2 = [p1 p3 p4];
            lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);

            x_l = zeros(1,1);
            y_l = zeros(1,1);
            for n=1:1:360
                if(abs(scan(n)) <= 1.3 && abs(scan(n)) > .06)
                    x_l(n) = scan(n)*cos((n-1)*pi/180);
                    y_l(n) = scan(n)*sin((n-1)*pi/180);
                else
                   x_l(n) = 0;
                   y_l(n) = 0;
                end
            end
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
            x_l = laserPts(1,:);
            y_l = laserPts(2,:);


            if(length(laserPts(1,:)) < 3)
                rc = 0;
                
                disp('pose estimate is way off, all points were thrown as outliers');
                return;
            end
            start = tic;
            try
%                 x_hull = transpose(laserPts(1,:));
%                 y_hull = transpose(laserPts(2,:));
%                 ref_hull = [x_hull y_hull];
%                 out=convhull(ref_hull);
%                 %disp(out2);
%                 extremes=ref_hull(out,:);
%                 x_l_h = transpose(extremes(:,1));
%                 y_l_h = transpose(extremes(:,2));
%                 laserPts = [x_l_h; y_l_h; ones(1,length(x_l_h));];
                x_l = x_l(1:1:end);
                y_l = y_l(1:1:end);
                w_l = ones(1,length(x_l));
                laserPts = [x_l; y_l; w_l];
                [rc,outPose] = lml.refinePose(thePose,laserPts, 100000);
            catch
                disp('localizer failed, trying again');
                x_l = x_l(1:1:end);
                y_l = y_l(1:1:end);
                w_l = ones(1,length(x_l));
                laserPts = [x_l; y_l; w_l];
                [rc,outPose] = lml.refinePose(thePose,laserPts, 100000);
            end


            disp(toc(start));
            if(rc)
                obj.last_pose = outPose;
                obj.localized = 1;
                thePose = outPose;
                plotRobotAnotate(thePose);
            else
                disp('localization not successful');
                return;
            end
        
        end

        function turnNL(obj,robot)

            turnRelAngle(robot,pi());

        end
        
        function turn(obj,robot)

            turnRelAngle(robot,pi());
            fwd = Pose(0,0,pi());
            
            tform_curr_pose_world = obj.last_pose.bToA()*fwd.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
            %scatter(curr_pose_world.x, curr_pose_world.y, 'filled', 'k')
            obj.localized = 0;
           
            %plotRobotAnotate(curr_pose_world);
            
            start_pose = curr_pose_world;
            if(obj.localized == 0)
                rc = 0;
                    while(rc == 0)
                        scan = getS;
                        [rc, thePose] = obj.localize(scan, start_pose);
                        if(rc == 1)
                            obj.last_pose = thePose;
                        end
                        pause(.01)
                    end
            end
        end
        
        function backUpNL(obj,robot)
            s_act = moveRelDistBack(robot,.15);
        end

        function backUp(obj,robot)
            s_act = moveRelDistBack(robot,.15);
            fwd = Pose(s_act,0,0);
            
            tform_curr_pose_world = obj.last_pose.bToA()*fwd.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
            scatter(curr_pose_world.x, curr_pose_world.y, 'filled', 'k')
            obj.localized = 0;
           
 

            start_pose = curr_pose_world;
            if(obj.localized == 0)
                rc = 0;
                    while(rc == 0)
                        scan = getS;
                        [rc, thePose] = obj.localize(scan, start_pose);
                        pause(.05)
                    end
            end
        end
        
        
        function ramSailNL(obj,robot)
            s_act = moveRelDistForward(robot,.15);
            

        end
        
        function ramSail(obj,robot)
            s_act = moveRelDistForward(robot,.15);
            fwd = Pose(s_act,0,0);
            
            tform_curr_pose_world = obj.last_pose.bToA()*fwd.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
            scatter(curr_pose_world.x, curr_pose_world.y, 'filled', 'k')
            obj.localized = 0;
           
 

            start_pose = curr_pose_world;
            if(obj.localized == 0)
                rc = 0;
                    while(rc == 0)
                        scan = getS;
                        [rc, thePose] = obj.localize(scan, start_pose);
                        pause(.05)
                    end
            end
        end
        
        function localize_and_plot(obj, robot)
            foundSail = 0;
            p1 = [0 ; 0];
            p2 = [ 48*.0254 ; 0];
            p3 = [0 ; 48*.0254 ];
            p4 = [48*.0254; 48*.0254];
            lines_p1 = [p2 p1 p2];
            lines_p2 = [p1 p3 p4];

            plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
            hold on;
            plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
            hold on;
            lml = LineMapLocalizer(lines_p1,lines_p2,.01,.005,.0005);

            hold on;

            start_pose = obj.last_pose;
            if(obj.localized == 0)
                rc = 0;
                    while(rc == 0)
                        scan = getS;
                        [rc, thePose] = obj.localize(scan, start_pose);
                        if(rc == 1)
                            obj.last_pose = thePose;
                        end
                        pause(.05)
                    end
            end
            init_pose = obj.last_pose;

                        x = zeros(1,1);
            y = zeros(1,1);

            scan = getS;
            for n=1:1:360
                if(abs(scan(n)) <= 1.3 && abs(scan(n)) > .1)
                    x(n) = scan(n)*cos((n-1)*pi/180);
                    y(n) = scan(n)*sin((n-1)*pi/180);
                else
                  x(n) = 0;
                  y(n) = 0;
                end
            end
            x_full = x;
            y_full = y;


            w_full = ones(1,length(x_full));
            laserPts = [x_full; y_full; w_full];
            laserPts_full = laserPts;
            world_laserPts_full = init_pose.bToA()*laserPts_full;
            scatter(world_laserPts_full(1,:), world_laserPts_full(2,:), 'r');

            ids = lml.throwInliers(init_pose,laserPts);
            laserPts(:,ids) = 0;
            x = laserPts(1,:);
            y = laserPts(2,:);
            world_laserPts = init_pose.bToA()*laserPts;
            scatter(world_laserPts(1,:), world_laserPts(2,:), 'g');
            hold on;


        
 
            
            
        end

        function foundSail = go_to_sail(obj, robot)
            foundSail = 0;
            p1 = [0 ; 0];
            p2 = [ 48*.0254 ; 0];
            p3 = [0 ; 48*.0254 ];
            p4 = [48*.0254; 48*.0254];
            lines_p1 = [p2 p1 p2];
            lines_p2 = [p1 p3 p4];

            plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
            hold on;
            plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
            hold on;
            lml = LineMapLocalizer(lines_p1,lines_p2,.01,.005,.0005);

            hold on;

            start_pose = obj.last_pose;
            if(obj.localized == 0)
                rc = 0;
                    while(rc == 0)
                        scan = getS;
                        [rc, thePose] = obj.localize(scan, start_pose);
                        if(rc == 1)
                            obj.last_pose = thePose;
                        end
                        pause(.05)
                    end
            end
            init_pose = obj.last_pose;

                        x = zeros(1,1);
            y = zeros(1,1);

            scan = getS;
            for n=1:1:360
                th = (n-1)*pi/180;
                th = th - atan2(.024, .28);
                if((abs(scan(n)) <= 1.3 && abs(scan(n)) > .1))
                    x(n) = scan(n)*cos(th);
                    y(n) = scan(n)*sin(th);

                else
                  x(n) = 0;
                  y(n) = 0;
                end
            end
            x_full = x;
            y_full = y;


            w_full = ones(1,length(x_full));
            laserPts = [x_full; y_full; w_full];
            laserPts_full = laserPts;
            world_laserPts_full = init_pose.bToA()*laserPts_full;
            scatter(world_laserPts_full(1,:), world_laserPts_full(2,:), 'r');

            ids = lml.throwInliers(init_pose,laserPts);
            laserPts(:,ids) = 0;
            x = laserPts(1,:);
            y = laserPts(2,:);
            world_laserPts = init_pose.bToA()*laserPts;
            scatter(world_laserPts(1,:), world_laserPts(2,:), 'g');
            hold on;


        
            [plotG, found, pose, laserPts] = findLineCandidate(laserPts);
            if(found == 0)

            end
            pose = finalPose(pose);
            %disp(found);
            tform_curr_pose_world = obj.last_pose.bToA()*pose.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
            pose = curr_pose_world;

            start_pose = init_pose;
            goal_pose = pose;
            theta = goal_pose.th;
            goal_pose = Pose(goal_pose.x, goal_pose.y, theta);
            if(found && plotG)
                [start_pose, goal_pose, state] = obj.executeTrajectory(robot, start_pose, goal_pose, .2);
                foundSail = 1;
            else
                disp('sail not found');
            end
            
        end
        
        
        function foundSail = go_to_sailNL(obj, robot)
            foundSail = 0;
                                    x = zeros(1,1);
            y = zeros(1,1);
            scan = getS;
            for n=1:1:360
                if(abs(scan(n)) <= 1.3 && abs(scan(n)) > .1)
                    x(n) = scan(n)*cos((n-1)*pi/180);
                    y(n) = scan(n)*sin((n-1)*pi/180);
                else
                  x(n) = 0;
                  y(n) = 0;
                end
            end
            x_full = x;
            y_full = y;

            init_pose = Pose(0,0,0);
            w_full = ones(1,length(x_full));
            laserPts = [x_full; y_full; w_full];
            laserPts_full = laserPts;
            world_laserPts_full = init_pose.bToA()*laserPts_full;
            scatter(world_laserPts_full(1,:), world_laserPts_full(2,:), 'r');
            hold on;
            found = 1;
            plotG = 0;
            sailPose = Pose(0,0,0);
            while(found == 1 && plotG == 0)
                [plotG, found, sailPose, laserPts] = findLineCandidate(laserPts);
            end

            %disp(found);

            if(found == 1 && plotG == 1)
                            pose = finalPose(sailPose);
                plotRobotAnotate(pose);
                curve = CubicSpiralTrajectory.planTrajectory(pose.x, pose.y, pose.th, 1);
                curve.planVelocities(.15);
                obj.executeTrajectoryRel(robot, curve, .2);
                foundSail = 1;
            else
                disp('sail not found');
            end
            
        end
        
        function localizeSim(obj, start_pose)
            obj.localized = 1;
            obj.last_pose = start_pose;
        end
                  
        
        
        function [state, i] = executeTrajectoryRel(obj, robot, curve, t_delay)
            %get reference to reference and actual state
            init_pose = Pose(0, 0, 0);
            tform_init_w2s = init_pose.bToA();
            t_f = curve.getTrajectoryDuration();
            n = floor(t_f/TrajectoryFollowerC.UpdatePause)+1;
            obj.robotState = RobotState(n, init_pose);
            
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
            while(t_i <= (t_f+t_delay+system_delay))
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
                tform_p_i_act_w2g = tform_init_w2s*tform_p_i_act_s2g;
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
                tform_p_i_ref_w2g = tform_init_w2s*tform_p_i_ref_s2g;
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
            
        end
            
        function [start_pose, goal_pose, state] = executeTrajectory(obj, robot, start_pose, goal_pose, velocity)  
            disp('begin to move')
            if(obj.localized == 0)
                
                 rc = 0;
                 while(rc == 0)
                    scan = getS;
                    [rc, thePose] = obj.localize(scan, start_pose);
                    obj.last_pose = thePose;
                    if(rc == 1)
                    
                    end
                    pause(.01)
                end
            end
                              
            goal_pose_robot = obj.world_to_robot(goal_pose);
            
            curve = CubicSpiralTrajectory.planTrajectory(goal_pose_robot.x, goal_pose_robot.y, goal_pose_robot.th, 1);
            
            
            plotArray1 = curve.poseArray(1,:);
            plotArray2 = curve.poseArray(2,:);
            w_full = ones(1,length(plotArray1));
            curvePts = [plotArray1; plotArray2; w_full];
            world_curvePts = obj.last_pose.bToA()*curvePts;
            
            plot(world_curvePts(1,:),world_curvePts(2,:),'r');
            
            curve.planVelocities(velocity);
            obj.localized = 0;
                        
            t_f = curve.getTrajectoryDuration();
            n = floor(t_f/TrajectoryFollower.UpdatePause)+1;
            obj.robotState = RobotState(n, obj.last_pose);
            
            actual_robot = obj.robotState;
            t = actual_robot.t;
            w = actual_robot.w;
            V = actual_robot.V;
            s = actual_robot.s;
            x = actual_robot.x;
            y = actual_robot.y;
            th = actual_robot.th;
            
            x_world = actual_robot.x_g_act;
            y_world = actual_robot.y_g_act;
            th_world = actual_robot.th_g_act;
                                           
            t_i = 0;
            dt_i = 0;
            prevX = getX;
            prevY = getY;
            pT = getT;
            cT = pT;
            curX = prevX;
            curY = prevY;
            
            pST = getST;
            cST = pST;
            while(t_i <= (t_f+TrajectoryFollower.t_delay+TrajectoryFollower.system_delay))
                % 1. UPDATE TIME
                i = actual_robot.i;
                t_i = t_i + dt_i;
                
                rc = 0;
                localPose = Pose(0,0,0);
                thePose = Pose(x_world(i), y_world(i), th_world(i));
                
                cST = getST;
                if(cST ~= pST)
                    pST = cST;
                    scan = getS;
                    [rc, localPose] = obj.localizeRT(scan, thePose);
                    if(rc)
                        %plotRobotAnotate(thePose);
                    end
                    
                end
                


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
                p_prev = Pose(x(i), y(i), th(i));
                p_i_act = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev);

                curr_pose_world = obj.robot_to_world(p_i_act);
                
                x_world(i+1) = curr_pose_world.x;
                y_world(i+1) = curr_pose_world.y;
                th_world(i+1) = curr_pose_world.th;
                t(i+1) = t_i;
                V(i+1) = V_i;
                w(i+1) = w_i;
                s(i+1) = s(i) + (V_i*dt_i);
                x(i+1) = p_i_act.x;
                y(i+1) = p_i_act.y;
                th(i+1) = p_i_act.th;
                
              
                % 4. UPDATE CONTROL             
                %get velocity from open loop 
                V_i = curve.getVAtTime(t_i - TrajectoryFollower.system_delay);
                w_i = curve.getwAtTime(t_i - TrajectoryFollower.system_delay);
%                 V_i_s = curve.getVAtDist(s(i));
%                 w_i_s = curve.getwAtTime(s(i));
% 
%                 V_i = max(V_i_t, V_i_s);
%                 w_i = max(w_i_t, w_i_s);


                p_i_ref = curve.getPoseAtTime(t_i - TrajectoryFollower.system_delay - .2);
                %get velocity from open loop 
                u_p_V = 0;
                u_p_w = 0;
                if(rc)
                    p_i_ref = obj.robot_to_world(p_i_ref);
                    [u_p_V, u_p_w] = obj.feedback(localPose,p_i_ref);
                else
                    p_i_ref = obj.robot_to_world(p_i_ref);
                    p_i_act = obj.robot_to_world(p_i_act);
                    [u_p_V, u_p_w] = obj.feedback(p_i_act,p_i_ref);
                end
                    
                V_i_U = V_i + (u_p_V);
                w_i_U = w_i + (u_p_w);

                [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i_U, w_i_U);
                [v_l_U , v_r_U] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);


                
                %5. SEND CONTROL TO ROBOT
                robot.sendVelocity(v_l_U, v_r_U);

                %6. UPDATE GRAPHS 
                if(rc)
                    plotRobotRealtime(localPose);
                else
                    plotRobotRealtime(curr_pose_world);
                end
                %plotRobotRealtime(t_p_i_ref);
                
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;

                %8. DELAY MAC CLOCK
                pause(.01);
            end   
            robot.stop();
            state = [x; y; th;];

                    last_ref = curve.getFinalPoseAsPose();
                    last_ref = obj.robot_to_world(last_ref);
                    last_act =  Pose(x_world(i), y_world(i), th_world(i));
                    
                p_prev = last_act;
                     
                    pST = cST;
                     rc = 0;
                     count = 0;
                     while(rc == 0)
                        scan = getS;
                        if(mod(count, 2) == 0)
                            p_prev = last_act;
                        else
                            p_prev = last_ref;
                        end
            
                        [rc, pose] = obj.localize(scan, p_prev);
                        count = count + 1;
                        pause(.05);
                     end
        end                     
    end

    methods(Access = private)

    end
    
    
end