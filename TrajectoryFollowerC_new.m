classdef TrajectoryFollowerC < handle
    
    properties(Constant)
        k_x_p = .01;
        k_y_p = .01;
        k_th_p = .01;
        UpdatePause = .01;
        t_delay = .2;
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
                
        function obj = TrajectoryFollowerC()
            obj.initialized = 0;
        end
        
        function goal_pose_robot = world_to_robot(obj, goal_pose)
            tform_goal_pose_robot = obj.last_pose.aToB()*goal_pose.bToA();
            goal_pose_robot = Pose.matToPoseVecAsPose(tform_goal_pose_robot);
        end
            
        function curr_pose_world = robot_to_world(obj, curr_pose_robot)
            tform_curr_pose_world = obj.last_pose.bToA()*curr_pose_robot.bToA();
            curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
        end
        
        function rc = localize(obj)
            rc = false;
%                 init_pose = Pose(0.6096, 0.6096, pi()/2.0);
%     %init_pose = finalPose(init_pose);
%     p1 = [0 ; 0];
%     p2 = [ 48*.0254 ; 0];
%     p3 = [0 ; 48*.0254 ];
%     lines_p1 = [p2 p1];
%     lines_p2 = [p1 p3];
%     lml = LineMapLocalizer(lines_p1,lines_p2,.01,.005,.0005);
%     trajFollower = TrajectoryFollowerC(init_pose, lml);
%     figure;
%     title(['Bootstrapping']);
%     xlabel('X (meters)');
%     ylabel('Y (meters)');
%     hold on;
%     axis([-.25 2 -.25 2]);
%     hold on;
%     plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
%     hold on;
%     plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
%     hold on;
%     success = 0;
%     bootstrap_pose = init_pose;
%     while(success == 0)
%         [bootstrap_pose, success] = trajFollower.localize(1000, bootstrap_pose);
%         pause(.01);
%     end
%                     function [init_pose, success] = localize(obj, num_iterations,bootstrap_pose)                
%             scan = getS;
% 
%             x_l = zeros(1,1);
%             y_l = zeros(1,1);
%             ref = zeros(360, 2);
%             for n=1:1:360
%                 if(abs(scan(n)) <= 1.5 && abs(scan(n)) > .06)
%                     x_l(n) = scan(n)*cos((n-1)*pi/180);
%                     y_l(n) = scan(n)*sin((n-1)*pi/180);
%                 else
%                    x_l(n) = 0;
%                    y_l(n) = 0;
%                 end
%                 ref(n,1) = x_l(n);
%                 ref(n,2) = y_l(n);
%             end  
%             lidsx = (x_l ~= 0);
%             lidsy = (y_l ~= 0);
%             lids = lidsx .* lidsy;
%             lids = logical(lids);
%             x_l = x_l(lids);
%             y_l = y_l(lids);
%             w_l = ones(1,length(x_l));
%             laserPtsAll = [x_l; y_l; w_l];
%             laserPtsPlotAll = obj.init_pose.bToA()*laserPtsAll;
%             scatter(laserPtsPlotAll(1,:), laserPtsPlotAll(2,:),'r');
%             
%             laserPtsHull = [x_l; y_l; ones(1,length(x_l));];
%             ids_h = obj.localizer.throwOutliers(obj.init_pose,laserPtsHull);
%             laserPtsHull(:,ids_h) = [];
%             x_hull = transpose(laserPtsHull(1,:));
%             y_hull = transpose(laserPtsHull(2,:));
%             
%             ref_hull = [x_hull y_hull];
%             out=convhull(ref_hull);
%             extremes=ref_hull(out,:);
%             x_l_h = transpose(extremes(:,1));
%             y_l_h = transpose(extremes(:,2));
%             laserPtsHull = [x_l_h; y_l_h; ones(1,length(x_l_h));];
%             laserPtsHull = [x_l_h(1:2:end); y_l_h(1:2:end); ones(1,length(x_l_h(1:2:end)));];
%             
%             w_l = ones(1,length(x_l));
%             laserPts = [x_l; y_l; w_l];
%             
%             lml = obj.localizer;
%             ids = lml.throwOutliers(obj.init_pose,laserPts);
%             laserPts(:,ids) = [];
% 
%             [success,thePose] = lml.refinePose(obj.init_pose,laserPtsHull, num_iterations);
%             init_pose = bootstrap_pose;
%             if(success)
%                 worldBodyPoints = thePose.bToA()*obj.bodyPts;
%                 plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'g-', 'Linewidth', 1);
%                 hold on;
%                 init_pose = thePose;
%                 obj.init_pose = thePose;
%             end
%             
%             laserPtsPlot = thePose.bToA()*laserPts;
%             laserPtsHullPlot = thePose.bToA()*laserPtsHull;
%             scatter(laserPtsPlot(1,:), laserPtsPlot(2,:), 'r');
%             scatter(laserPtsHullPlot(1,:), laserPtsHullPlot(2,:), 'filled', 'r');
%             hold on;
%             pause(.01);
%         end
        
        end

        function [sail_pose] = find_sail(obj, scan)
        
%             close all;
% clc;
% figure(1); clf;
% hold on;
% axis([-1.1 1.1 -1.1 1.1]);
% for i=1:1:15
%     x = zeros(1,1);
%     y = zeros(1,1);
%     scan = transpose(rangeImages(i,:));
%     ref = zeros(360, 2);
%     for n=1:1:360
%         if(abs(scan(n)) <= 1 && abs(scan(n)) > .06)
%             x(n) = scan(n)*cos((n-1)*pi/180);
%             y(n) = scan(n)*sin((n-1)*pi/180);
%         else
%           x(n) = 0;
%           y(n) = 0;
%         end
%     end
%     x_full = x;
%     y_full = y;
%     
%     lidsx = (x ~= 0);
%     lidsy = (y ~= 0);
%     lids = lidsx .* lidsy;
%     lids = logical(lids);
%     x = x(lids);
%     y = y(lids);
%     
%     first_sail_pt = [x(1) y(1)];
%     last_sail_pt = [x(end) y(end)];
%     
%     
%     x = x(2:length(x)-1);
%     y = y(2:length(y)-1);
%     
%     
%     hull_ref = [transpose(x) transpose(y)];
%     
%     hull_indices = convhull(hull_ref);
%     hull_pts=hull_ref(hull_indices,:);
%     
%     x_hull_pts = transpose(hull_pts(:,1));
%     y_hull_pts = transpose(hull_pts(:,2));
%     
%     end_pt1 = hull_pts(1,:);
%     compute Euclidean distances:
%     distances = sqrt(sum(bsxfun(@minus, hull_pts, end_pt1).^2,2));
%     end_pt2 = hull_pts(find(distances==max(distances)),:); 
%        
%     x1 = end_pt1(1);
%     y1 = end_pt1(2);
%     x2 = end_pt2(1);
%     y2 = end_pt2(2);   
%     fitvars = polyfit(x_hull_pts, y_hull_pts, 1);
%     best_fit_slope = fitvars(1);
%     
%     m_sail_start_slope = (first_sail_pt(2) - y1)/(first_sail_pt(1) - x1);
%     d_error = pdist([end_pt1; first_sail_pt],'euclidean');
%     slope_error = abs(m_sail_start_slope - best_fit_slope);
%     if(d_error <= .0175 && slope_error <= 3.25)
%         x1 = first_sail_pt(1);
%         y1 = first_sail_pt(2);
%     else
%         scatter(first_sail_pt(1), first_sail_pt(2), 'filled', 'MarkerFaceColor','r');
%     end
%     
%     
%     m_sail_end_slope = (last_sail_pt(2) - y2)/(last_sail_pt(1) - x2);
%     d_error = pdist([end_pt2; last_sail_pt],'euclidean');
%     slope_error = abs(m_sail_end_slope - best_fit_slope);
%     if(d_error <= .0175 && slope_error <= 3.25)
%         x2 = last_sail_pt(1);
%         y2 = last_sail_pt(2);
%     else
%         scatter(last_sail_pt(1), last_sail_pt(2), 'filled', 'MarkerFaceColor','r');
%     end
%     
%     
%     th = atan2(-(x2-x1),(y2-y1));
%     best_fit_mid = midPoint([[x1 y1] [x2 y2]]);
%     pose = Pose(best_fit_mid(1), best_fit_mid(2), th);
%     pose = finalPose(pose);
%     bodyPts = bodyGraph();
%     worldBodyPoints = pose.bToA()*bodyPts;
%     
%     
%     scatter( x_hull_pts,y_hull_pts, 'filled', 'MarkerFaceColor', 'g');
%     scatter( [x1 x2], [y1 y2], 'filled', 'MarkerFaceColor', 'b');  
%     scatter(x_full, y_full, 'g');
%     scatter(best_fit_mid(1), best_fit_mid(2), 'filled', 'MarkerFaceColor', 'k');
%     
%     
%     plot(worldBodyPoints(1,:), worldBodyPoints(2,:), 'c-', 'Linewidth', 3);
%     txt = sprintf('[%0.2f, %0.2f, %0.2f]', pose.x, pose.y, rad2deg(pose.th));
%     txt = strcat('\leftarrow', txt); 
%     t = text(worldBodyPoints(1,1),worldBodyPoints(2,1),txt,'FontSize',5);
%     t.Color = 'k';
%     hold on;
%     
%     
% 
%     pause(.01);
% end
        
        
        
        end
            
        function [start_pose, goal_pose, state] = executeTrajectorySim(obj, robot, start_pose, goal_pose, velocity)            
            
            
            
            obj.last_pose = start_pose;
            
            goal_pose_robot = world_to_robot(goal_pose);
            
            curve = CubicSpiralTrajectory.planTrajectory(goal_pose_robot.x, goal_pose_robot.y, goal_pose_robot.th, 1);
            curve.planVelocities(velocity);
                        
            t_f = curve.getTrajectoryDuration();
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
            
            x_world = actual_robot.x_g_act;
            y_world = actual_robot.y_g_act;
            th_world = actual_robot.th_g_act;

                                            
         
            i = actual_robot.i;
                            
            t_i = 0;
            dt_i = TrajectoryFollowerC.UpdatePause;
            while(actual_robot.i < n)
                % 1. UPDATE TIME
                i = actual_robot.i;
                t_i = i*dt_i;

                V_i = curve.getVAtTime((i-1)*dt_i);
                w_i = curve.getwAtTime((i-1)*dt_i);
                
                % 3. UPDATE STATE  
                [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i, w_i);
                [vl_i , vr_i] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);

                [V_i , w_i] = RobotModelAdv.vlvrToVw(vl_i, vr_i);
                p_prev = Pose(x(i), y(i), th(i));
                p_i_act = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev);

                %p_i_act=curr_pose_robot
                curr_pose_world = robot_to_world(p_i_act);
                
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
                
                %5. SEND CONTROL TO ROBOT
                %robot.sendVelocity(v_l_U, v_r_U);

                %6. UPDATE GRAPHS 
                plotRobotRealtime(curr_pose_world);
                %plotRobotRealtime(t_p_i_ref);
                
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;

                %8. DELAY MAC CLOCK
                pause(TrajectoryFollowerC.UpdatePause);
            end    
            robot.stop();
            state = [x; y; th;];         
        end
                       
        function [start_pose, end_pose, state] = executeTrajectory(obj, robot, start_pose, end_pose, velocity)            
            
            if(obj.localized == 0)
                rc = obj.localize(start_pose);
                %localize sets last pose to localized pose and flips the
                %flag and prints localization success and plots an annoted
                %robot
                if(rc == 0)
                    obj.last_pose = start_pose;
                    disp('Localization failed!!');
                end
            end
            
            tform_end_pose = end_pose.bToA();
            tform_inv_robot = obj.last_pose.aToB();
            tform_end_pose_robot = tform_inv_robot*tform_end_pose;
            end_pose_robot = Pose.matToPoseVecAsPose(tform_end_pose_robot);
            
            
            
            curve = CubicSpiralTrajectory.planTrajectory(end_pose_robot.x, end_pose_robot.y, end_pose_robot.th, 1);
            curve.planVelocities(velocity);
                        
            t_f = curve.getTrajectoryDuration();
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
                                            
         
            i = actual_robot.i;
                            
            t_i = 0;
            dt_i = 0;
            prevX = getX;
            prevY = getY;
            pT = getT;
            cT = pT;
            curX = prevX;
            curY = prevY;
            sf = curve.distArray(end);
            while(t_i <= (t_f+TrajectoryFollowerC.t_delay+TrajectoryFollowerC.system_delay))
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
                tform_p_i_act_w2g = obj.last_pose.bToA()*tform_p_i_act_s2g;
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
                
                p_i_ref = curve.getPoseAtTime(t_i - TrajectoryFollowerC.system_delay);
                tform_p_i_ref_s2g = p_i_ref.bToA();
                tform_p_i_ref_w2g = obj.last_pose.bToA()*tform_p_i_ref_s2g;
                t_p_i_ref = Pose.matToPoseVecAsPose(tform_p_i_ref_w2g);
                x_g_ref(i+1) = t_p_i_ref.x;
                y_g_ref(i+1) = t_p_i_ref.y;
                th_g_ref(i+1) = t_p_i_ref.th;           
                
                %compute error
                r_r_p = t_p_i_act.aToB()*(t_p_i_ref.getPoseVec() - t_p_i_act.getPoseVec());
                err_x_g_ref(i+1) = r_r_p(1);
                err_y_g_ref(i+1) = r_r_p(2);
                err_th_g_ref(i+1) = r_r_p(3);
              
                % 4. UPDATE CONTROL             
                %get velocity from open loop 
                V_i = curve.getVAtTime(t_i - TrajectoryFollowerC.system_delay);
                w_i = curve.getwAtTime(t_i - TrajectoryFollowerC.system_delay);
                %V_i_s = curve.getVAtDist(s(i));
                
                %V_i = max(V_i, V_i_s);
                                 
                [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i, w_i);
                [v_l_U , v_r_U] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);
                
                %5. SEND CONTROL TO ROBOT
                robot.sendVelocity(v_l_U, v_r_U);

                %6. UPDATE GRAPHS 
                plotRobotRealtime(t_p_i_act);
                %plotRobotRealtime(t_p_i_ref);
                
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;

                %8. DELAY MAC CLOCK
                pause(TrajectoryFollowerC.UpdatePause);
            end    
            robot.stop();
            state = [x; y; th;];         
        end
    end

    methods(Access = private)

    end
    
    
end