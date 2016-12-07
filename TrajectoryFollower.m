classdef TrajectoryFollower < handle
    
    properties(Constant)
        k_x_p = 0;
        k_y_p = 1.5;
        k_th_p = .6;
        k_fuse = .25;
        UpdatePause = .01;
        t_delay = .2;
        system_delay = 1.5;
    end
    
    properties(Access = public)
        robotState;
        last_pose;
        localized = 0;
        initialized = 0;
        A;
        bodyPts;
        lines_p1;
        lines_p2;
        localizer;
        maxVSail;
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)    
        
        function [delta] = poseDiff(obj,poseVec1,poseVec2)
            %static function
            x2 = poseVec2(1);
            x1 = poseVec1(1);
            y2 = poseVec2(2);
            y1 = poseVec1(2);
            th2 = poseVec2(3);
            th1 = poseVec1(3);
            delta = [x2-x1; y2 - y1; atan2(sin(th2-th1),cos(th2-th1))];
        end

        function fusedPose = fusePose(obj, odoPose,lidPose)    
            fusedPoseVec = odoPose.getPoseVec() + TrajectoryFollower.k_fuse*obj.poseDiff(odoPose.getPoseVec(),lidPose.getPoseVec());
            fusedPose = Pose(fusedPoseVec(1), fusedPoseVec(2), fusedPoseVec(3));
        end
               
        function obj = TrajectoryFollower(errorT, maxVSail)
            obj.initialized = 0;
            obj.maxVSail = maxVSail;
            off = .0254*6;
            p1 = [0 ; 0];
            p2 = [ 12*8*.0254 ; 0];
            p3 = [0 ; 12*12*.0254 ];
            p4 = [12* 8*.0254; 12*12*.0254];
            obj.lines_p1 = [p2-[off;0] p1+[0;off] p2+[0;off] p3+[off;0]];
            obj.lines_p2 = [p1+[off;0] p3-[0;off] p4-[0;off] p4-[off;0]];
            obj.localizer = LineMapLocalizer(obj.lines_p1,obj.lines_p2,.01,errorT,.0005);
        end
        
        function lml = getLocalizer(obj, error_thres)
            lml = LineMapLocalizer(obj.lines_p1,obj.lines_p2,.01,.001*error_thres,.0005);
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
        
        function [rc, thePose] = localizeRT(obj, scan, thePose, localizeCount)
            lml = obj.getLocalizer(localizeCount);
            x_l = [];
            y_l = [];
            for n=1:1:360
                if(abs(scan(n)) <= 2.5 && abs(scan(n)) > .06)
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
        
        function [rc, thePose] = localize(obj, thePose)

            x_l = [];
            y_l = [];
            pST = getST;
            cST = pST;
            rc = 0;
            localizeCount = 1;
            while(rc == 0)
                lml = obj.getLocalizer(localizeCount);
                
                % WAIT FOR SCAN TO CHANGE
                scanCount = 0;
                cST = getST;
                scan = getS;
                while(eq(cST, pST))     
                    if(scanCount > 100000)
                        S = sprintf('Scan not changing');
                        errorStruct.message = S;
                        errorStruct.identifier = 'TRJFC:LASER_ISSUE';
                        error(errorStruct);
                        robot = raspbot;
                        robot.startLaser();
                        robot.laser.NewMessageFcn=@laserEventListener;
                        robot.encoders.NewMessageFcn=@encoderEventListener;
                        pause(5);
                        obj.localize(thePose);
                    end
                    scanCount = scanCount + 1;
                    
                    cST = getST;
                    scan = getS;

                    pause(.001);
                end
                pST = cST;
                
                for n=1:1:360
                    if(abs(scan(n)) <= 3 && abs(scan(n)) > .06)
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
                    %error('pose estimate is way off, all points were thrown as outliers');
                    robot = raspbot;
                    robot.startLaser();
                    robot.laser.NewMessageFcn=@laserEventListener;
                    robot.encoders.NewMessageFcn=@encoderEventListener;
                    pause(5);
                    obj.localize(thePose);
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
                
                if(rc)
                    obj.last_pose = outPose;
                    obj.localized = 1;
                    thePose = outPose;
                    plotRobotAnotate(thePose, laserPts);
                else
                    localizeCount = localizeCount + 1;
                    disp('localization not successful');
                end
                
                
            end
        end
        
        function turn(obj,robot,angle)

            turnRelAngle(robot,rad2deg(angle));
            fwd = Pose(0,0,angle);
            
            tform_curr_pose_world = obj.last_pose.bToA()*fwd.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
            %scatter(curr_pose_world.x, curr_pose_world.y, 'filled', 'k')
            obj.localized = 0;
           
            %plotRobotAnotate(curr_pose_world);
            
            start_pose = curr_pose_world;
            if(obj.localized == 0)
                [rc, thePose] = obj.localize(start_pose);
            end
        end
        
        function backUp(obj,robot, dist)
            s_act = moveRelDistBack(robot,dist);
            fwd = Pose(s_act,0,0);
            
            tform_curr_pose_world = obj.last_pose.bToA()*fwd.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
            obj.localized = 0;
           
 

            start_pose = curr_pose_world;
            if(obj.localized == 0)
                [rc, thePose] = obj.localize(start_pose);
            end
        end
             
        function ramSail(obj,robot)
            s_act = moveRelDistForward(robot,.15);
            fwd = Pose(s_act,0,0);
            
            tform_curr_pose_world = obj.last_pose.bToA()*fwd.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
            obj.localized = 0;
           
 

            start_pose = curr_pose_world;
            if(obj.localized == 0)
                [rc, thePose] = obj.localize(start_pose);
            end
        end
        
        function picked_up = is_sail_on_fork(obj)
            picked_up = 1;
            for i=1:5
                x = [];
                y = [];
                scan = getS;
                for n=1:1:360
                    pts = 0;
                    th = (n-1)*pi/180;
                    wedge_width_left = pi()/8;
                    wedge_width_right = pi()/8;
                    if((abs(scan(n)) <= 3 && abs(scan(n)) > .1) && ((th <= wedge_width_left ) || (th >= 2*pi() - wedge_width_right)) )
                        x(n) = scan(n)*cos(th);
                        y(n) = scan(n)*sin(th);
                        pts = pts + 1;
                    else
                      x(n) = 0;
                      y(n) = 0;
                    end

                    if((abs(scan(n)) <= 1 && abs(scan(n)) > .1))
                        x_full_nowedge(n) = scan(n)*cos(th);
                        y_full_nowedge(n) = scan(n)*sin(th);

                    else
                      x_full_nowedge(n) = 0;
                      y_full_nowedge(n) = 0;
                    end

                end
                x_full = x;
                y_full = y;
                
                if(pts > 5)
                    picked_up = 0;
                    disp('sail not detected on forks!!!');
                end
                
                pause(.05);
            end
        end
        
        function [foundSail, sail_to_left] = go_to_sail(obj, robot, sail_index)
            foundSail = 0;
            sails = [];
            lml = obj.localizer;

            hold on;

            start_pose = obj.last_pose;
            if(obj.localized == 0)
                [rc, thePose] = obj.localize(start_pose);
            end
            init_pose = obj.last_pose;
            stale_pose = init_pose;
            x = [];
            y = [];
            %offset is .25 for 7 (maybe more or less), .75 for 8, .3 for 6
            %current is 7
            offset = .25;
                                                       
            if(sail_index == 10 || sail_index == 8)
                 offset = .5;
            end
            
            scan = getS;
            scan_bk = scan;
            for n=1:1:360
                th = (n-1)*pi/180;
                th = th - offset*atan2(.024, .28);
                wedge_width_left = pi()/7.5;
                wedge_width_right = pi()/4;
                if((abs(scan(n)) <= 1.5 && abs(scan(n)) > .1) && ((th <= wedge_width_left ) || (th >= 2*pi() - wedge_width_right)) )
                    x(n) = scan(n)*cos(th);
                    y(n) = scan(n)*sin(th);

                else
                  x(n) = 0;
                  y(n) = 0;
                end
                
                if((abs(scan(n)) <= 1 && abs(scan(n)) > .1))
                    x_full_nowedge(n) = scan(n)*cos(th);
                    y_full_nowedge(n) = scan(n)*sin(th);

                else
                  x_full_nowedge(n) = 0;
                  y_full_nowedge(n) = 0;
                end
                
            end
            x_full = x;
            y_full = y;
            
            w_nowedge = ones(1,length(x_full_nowedge));
            laserPts_nowedge = [x_full_nowedge; y_full_nowedge; w_nowedge];
            ids = lml.throwInliers(init_pose,laserPts_nowedge);
            laserPts_nowedge(:,ids) = 0;
            world_laserPts_nowedge = init_pose.bToA()*laserPts_nowedge;
            scatter(world_laserPts_nowedge(1,:), world_laserPts_nowedge(2,:), 'r');
            hold on;


            w_full = ones(1,length(x_full));
            laserPts = [x_full; y_full; w_full];
            ids = lml.throwInliers(init_pose,laserPts);
            laserPts(:,ids) = 0;
            
            %sail map

            %8 9 10 line points
            margin = 8; %inches
            p1 = [(7*12)*.0254 ; 12*.0254];
            p2 = [ 7*12*.0254 ; 4.5*12*.0254 ];

            %midfield
            p3 = [(0+margin)*0.0254 ; 6*12*.0254 ];
            p4 = [(8*12-margin)*.0254; 6*12*.0254];
            sail_lines_p1 = [p2 p3];
            sail_lines_p2 = [p1 p4];
            sail_lml = LineMapLocalizerRelaxed(sail_lines_p1,sail_lines_p2,.01,.001,.0005);
            sail_ids = sail_lml.throwOutliers(init_pose,laserPts);
            laserPts(:,sail_ids) = 0;
            
            
            x = laserPts(1,:);
            y = laserPts(2,:);
            world_laserPts = init_pose.bToA()*laserPts;
            scatter(world_laserPts(1,:), world_laserPts(2,:), 'g');
            hold on;
            
            is_laserPts_left = 1;
            poses = [];
            while(is_laserPts_left == 1)
                [sailFound, is_laserPts_left, sail, laserPts] = findLineCandidate(laserPts, init_pose);
 
                if(sailFound)
                    sail = finalPose(sail);
                    tform_curr_pose_world = init_pose.bToA()*sail.bToA();
                    sail_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);       
                    poses = [sail_pose_world, poses];
                    hold on;
                end

                %disp(nnz(x));
            end

            if(~isempty(poses))
                sorted = sortClosest(init_pose, poses); 
                for i=1:length(sorted)
                    txt = sprintf(' %d ', i);
                    txt = strcat('\leftarrow ', txt); 
                    g_pose = sorted(i);
                    t = text(g_pose.x,g_pose.y,txt,'FontSize',10);
                    t.Color = 'k';
                    hold on;
                end
                goal_pose = sorted(1);
                turn_angle = atan2((goal_pose.y - obj.last_pose.y), (goal_pose.x - obj.last_pose.x)) - obj.last_pose.th;
                foundSail = 1;
                if(abs(turn_angle) > deg2rad(20))
                    disp('sail found to the right but angle is too large');
                    turn_angle = atan2((goal_pose.y - obj.last_pose.y), (goal_pose.x - obj.last_pose.x)) - obj.last_pose.th;
                    obj.turn(robot, turn_angle);
                    obj.backUp(robot, .15);
                    
                    obj.executeTrajectory(robot, obj.last_pose, goal_pose, obj.maxVSail);

                else
                    obj.turn(robot, turn_angle);
                    obj.executeTrajectory(robot, obj.last_pose, goal_pose, obj.maxVSail);

                end
%                 if(abs(turn_angle) > deg2rad(20))
%                     obj.backUp(robot, .45);
%                     aquisition_goal_pose = aquisitionPose(goal_pose);
%                     turn_angle = atan2((aquisition_goal_pose.y - obj.last_pose.y), (aquisition_goal_pose.x - obj.last_pose.x)) - obj.last_pose.th;
%                     obj.turn(robot, turn_angle);
%                     obj.executeTrajectory(robot, obj.last_pose, aquisition_goal_pose, .5);
%                     obj.go_to_sail(robot);
%                     foundSail = 1;
                
              
                
               
                
                
            else    
                disp('sail not found');
            end
                
                x = [];
                y = [];
                scan = scan_bk;
                %return only sails to the left
                for n=1:1:360
                    th = (n-1)*pi/180;
                    if((abs(scan(n)) <= 1.5 && abs(scan(n)) > .1) && ((th >= pi/8) && (th <= pi/4) ))
                        x(n) = scan(n)*cos(th);
                        y(n) = scan(n)*sin(th);

                    else
                      x(n) = 0;
                      y(n) = 0;
                    end

                    if((abs(scan(n)) <= 1 && abs(scan(n)) > .1))
                        x_full_nowedge(n) = scan(n)*cos(th);
                        y_full_nowedge(n) = scan(n)*sin(th);

                    else
                      x_full_nowedge(n) = 0;
                      y_full_nowedge(n) = 0;
                    end

                end
                x_full = x;
                y_full = y;

                w_nowedge = ones(1,length(x_full_nowedge));
                laserPts_nowedge = [x_full_nowedge; y_full_nowedge; w_nowedge];
                ids = lml.throwInliers(stale_pose,laserPts_nowedge);
                laserPts_nowedge(:,ids) = 0;
                world_laserPts_nowedge = stale_pose.bToA()*laserPts_nowedge;


                w_full = ones(1,length(x_full));
                laserPts = [x_full; y_full; w_full];
                ids = lml.throwInliers(stale_pose,laserPts);
                laserPts(:,ids) = 0;

                %sail map

                %8 9 10 line points
                margin = 8; %inches
                p1 = [(7*12)*.0254 ; 12*.0254];
                p2 = [ 7*12*.0254 ; 4.5*12*.0254 ];

                %midfield
                p3 = [(0+margin)*0.0254 ; 6*12*.0254 ];
                p4 = [(8*12-margin)*.0254; 6*12*.0254];
                sail_lines_p1 = [p2 p3];
                sail_lines_p2 = [p1 p4];
                sail_lml = LineMapLocalizerRelaxed(sail_lines_p1,sail_lines_p2,.01,.001,.0005);
                sail_ids = sail_lml.throwOutliers(stale_pose,laserPts);
                laserPts(:,sail_ids) = 0;


                x = laserPts(1,:);
                y = laserPts(2,:);
                world_laserPts = stale_pose.bToA()*laserPts;

                is_laserPts_left = 1;
                poses = [];
                while(is_laserPts_left == 1)
                    [sailFound, is_laserPts_left, sail, laserPts] = findLineCandidateRelaxed(laserPts, stale_pose);

                    if(sailFound)
                        sail = finalPose(sail);
                        tform_curr_pose_world = stale_pose.bToA()*sail.bToA();
                        sail_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);       
                        poses = [sail_pose_world, poses];
                        hold on;
                    end

                    %disp(nnz(x));
                end
                sail_to_left = 0;
                if(~isempty(poses))
                    sail_to_left = 1;
                end
                
            
        end        
               
        function init_pose = localizeSim(obj, start_pose, scan)
            x_l = [];
            y_l = [];

                lml = obj.localizer;
                               
                for n=1:1:360
                    if(abs(scan(n)) <= 3 && abs(scan(n)) > .06)
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
                ids = lml.throwOutliers(start_pose,laserPts);
                laserPts(:,ids) = [];
                x_l = laserPts(1,:);
                y_l = laserPts(2,:);


                if(length(laserPts(1,:)) < 3)
                    error('pose estimate is way off, all points were thrown as outliers');
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
                    [rc,outPose] = lml.refinePose(start_pose,laserPts, 100000);
                catch
                    disp('localizer failed, trying again');
                    x_l = x_l(1:1:end);
                    y_l = y_l(1:1:end);
                    w_l = ones(1,length(x_l));
                    laserPts = [x_l; y_l; w_l];
                    [rc,outPose] = lml.refinePose(start_pose,laserPts, 100000);
                end
                
                if(rc)
                    disp(toc(start));
                    init_pose = outPose;
                    plotRobotAnotate(init_pose, laserPts);
                end
                
                

        end
                      
        function [start_pose, goal_pose, state] = executeTrajectory(obj, robot, start_pose, goal_pose, velocity)  
            if(obj.localized == 0)
                [rc, thePose] = obj.localize(start_pose);
            end
                              
            goal_pose_robot = obj.world_to_robot(goal_pose);
            
            curve = CubicSpiralTrajectory.planTrajectory(goal_pose_robot.x, goal_pose_robot.y, goal_pose_robot.th, 1);
            
            
            plotArray1 = curve.poseArray(1,:);
            plotArray2 = curve.poseArray(2,:);
            w_full = ones(1,length(plotArray1));
            curvePts = [plotArray1; plotArray2; w_full];
            world_curvePts = obj.last_pose.bToA()*curvePts;
            
            plot(world_curvePts(1,:),world_curvePts(2,:),'m', 'Linewidth', 3);
            
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
            x_world(1) = obj.last_pose.x;
            y_world(1) = obj.last_pose.y;
            
            signal1 = plot(x_world, y_world, 'c-', 'Linewidth', 3);
            hold on;
            set(signal1, 'xdata', [get(signal1,'xdata') x_world(1)], 'ydata', [get(signal1,'ydata') y_world(1)]);
            hold on;                              
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
            localizeCount = 1;
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
                    [rc, localPose] = obj.localizeRT(scan, thePose, localizeCount);
                    if(rc)
                        %plotRobotAnotate(thePose);
                    else
                        localizeCount = localizeCount +1;
                    end
                    
                end
                


                % 2. WAIT FOR ENCODER CHANGE
                count = 0;
                while(eq(cT, pT))     
                    if(count > 100000)
                        S = sprintf('Encoders not changing at cT=%f, pT=%f, curX%f, curY%f, t_i=%f', cT, pT, curX, curY, t_i);
                        errorStruct.message = S;
                        errorStruct.identifier = 'TRJFC:ENCODER_ISSUE';
                        %error(errorStruct);
                        robot = raspbot;
                        robot.startLaser();
                        robot.laser.NewMessageFcn=@laserEventListener;
                        robot.encoders.NewMessageFcn=@encoderEventListener;
                        pause(5);
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


                p_i_ref = curve.getPoseAtTime(t_i - TrajectoryFollower.system_delay - TrajectoryFollower.t_delay);
                %get velocity from open loop 
                u_p_V = 0;
                u_p_w = 0;
                if(rc)
                    x_world(i+1) = localPose.x;
                    y_world(i+1) = localPose.y;
                    th_world(i+1) = localPose.th;
                    robot_local_pose = obj.world_to_robot(localPose);
                    fusedPose = obj.fusePose(p_i_act, robot_local_pose);
                    x(i+1) = fusedPose.x;
                    y(i+1) = fusedPose.y;
                    th(i+1) = fusedPose.th;
                    [u_p_V, u_p_w] = obj.feedback(fusedPose,p_i_ref);
                else
                    curr_pose_world = obj.robot_to_world(p_i_act);          
                    x_world(i+1) = curr_pose_world.x;
                    y_world(i+1) = curr_pose_world.y;
                    th_world(i+1) = curr_pose_world.th;
                    
                    %p_i_ref = obj.robot_to_world(p_i_ref);
                    %p_i_act = obj.robot_to_world(p_i_act);
                    [u_p_V, u_p_w] = obj.feedback(p_i_act,p_i_ref);
                end
                    
                V_i_U = V_i + (u_p_V);
                w_i_U = w_i + (u_p_w);

                [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i_U, w_i_U);
                [v_l_U , v_r_U] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);


                
                %5. SEND CONTROL TO ROBOT
                robot.sendVelocity(v_l_U, v_r_U);

                %6. UPDATE GRAPHS
                set(signal1, 'xdata', [get(signal1,'xdata') x_world(i)], 'ydata', [get(signal1,'ydata') y_world(i)]);
                hold on;
                if(rc)
                    plotRobotRealtime(localPose);
                else
                    plotRobotRealtime(curr_pose_world);
                end
                
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;

                %8. DELAY MAC CLOCK
                pause(TrajectoryFollower.UpdatePause);
            end   
            robot.stop();
            state = [x; y; th;];

            last_ref = curve.getFinalPoseAsPose();
            last_ref = obj.robot_to_world(last_ref);
            last_act =  Pose(x_world(i), y_world(i), th_world(i));

            p_prev = last_act;

            rc = 0;
            count = 0;
            while(rc == 0)
                if(mod(count, 2) == 0)
                    p_prev = last_act;
                else
                    p_prev = last_ref;
                end

                [rc, pose] = obj.localize(p_prev);
                count = count + 1;
            end
        end                     
    end

    methods(Access = private)

    end
    
    
end