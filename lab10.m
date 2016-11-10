%% driver (test)
clc;
clearvars -except robot;
close all;
format long g;
if(~exist('robot', 'var'))
    robot = raspbot();
    robot.encoders.NewMessageFcn=@encoderEventListener;
    robot.laser.NewMessageFcn=@laserEventListener;
    robot.startLaser();
    pause(3);
end
pause(1);


try
    init_pose = Pose(0.6096, 0.6096, pi()/2.0);
    %init_pose = finalPose(init_pose);
    p1 = [0 ; 0];
    p2 = [ 48*.0254 ; 0];
    p3 = [0 ; 48*.0254 ];
    lines_p1 = [p2 p1];
    lines_p2 = [p1 p3];
    lml = LineMapLocalizer(lines_p1,lines_p2,.01,.005,.0005);
    trajFollower = TrajectoryFollowerC(init_pose, lml);
    figure;
    title(['Bootstrapping']);
    xlabel('X (meters)');
    ylabel('Y (meters)');
    hold on;
    axis([-.25 2 -.25 2]);
    hold on;
    plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    success = 0;
    bootstrap_pose = init_pose;
    while(success == 0)
        [bootstrap_pose, success] = trajFollower.localize(1000, bootstrap_pose);
        pause(.01);
    end
    if(success)
        title('Bootstrapping success! Press enter to continue');
        fprintf('Robot state [x y th]: [%f %f %f]\n', bootstrap_pose.x, bootstrap_pose.y, bootstrap_pose.th);
        str = sprintf('Robot state [x y th]: [%f %f %f]', bootstrap_pose.x, bootstrap_pose.y, bootstrap_pose.th);
        dim = [.14 .6 .3 .3];
        annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on');
        set(gcf,'KeyPressFcn',@keyboardEventListener);
        while(1)
            key = pollKeyboard();
            if(strcmp(key,'return'))
                break;
            end
            pause(.05);
        end
        set(gcf,'KeyPressFcn',[]);
        close all;
    end
    lml = LineMapLocalizer(lines_p1,lines_p2,.05,.005,.0005);
    trajFollower.localizer = lml;
    %calculate robot relative poses
    
    %1. xf = 0.3048; yf = 0.9144; thf = pi()/2.0;
    final_pose1_world = Pose(0.3048, 0.9144, pi()/2.0);
    %final_pose1_world = finalPose(final_pose1_world);
    tform_goal_world = final_pose1_world.bToA();
    tform_inv_robot = trajFollower.init_pose.aToB();
    tform_goal_robot = tform_inv_robot*tform_goal_world;
    final_pose1_relative = Pose.matToPoseVecAsPose(tform_goal_robot);
    
    curve = CubicSpiralTrajectory.planTrajectory(final_pose1_relative.x, final_pose1_relative.y, final_pose1_relative.th, 1);
    curve.planVelocities(.15);
    
    hold on;
    title(['Reference (magenta) & Sensed (cyan) State (x vs. y)']);
    hold on;
    axis([-.25 1.5 -.25 1.5]);
    hold on;
    xlabel('X (meters)');
    hold on;
    ylabel('Y (meters)');
    hold on;
    
    [r, r_size] = trajFollower.executeTrajectorySE(robot, curve);
    hold on;
    
    
    success = 0;
    bootstrap_pose = final_pose1_world;
    while(success == 0)
        [bootstrap_pose, success] = trajFollower.localize(100000, bootstrap_pose);
        pause(.01);
    end
    pause(15);
    
    %2. xf = 0.9144; yf = 0.3048; thf = 0.0;
    final_pose2_world = Pose(0.9144, 0.3048, 0.0);
    %final_pose2_world = finalPose(final_pose2_world);
    tform_goal_world = final_pose2_world.bToA();
    tform_inv_robot = trajFollower.init_pose.aToB();
    tform_goal_robot = tform_inv_robot*tform_goal_world;
    final_pose2_relative = Pose.matToPoseVecAsPose(tform_goal_robot);

    curve2 = CubicSpiralTrajectory.planTrajectory(final_pose2_relative.x, final_pose2_relative.y, final_pose2_relative.th, 1);
    curve2.planVelocities(.15);
    
    [r2, r2_size] = trajFollower.executeTrajectorySE(robot, curve2);
    hold on;
    
    success = 0;
    bootstrap_pose = final_pose2_world ;
    while(success == 0)
        [bootstrap_pose, success] = trajFollower.localize(100000, bootstrap_pose);
        pause(.01);
    end
    pause(15);
    
    %3. xf = 0.6096; yf = 0.6096; thf = pi()/2.0;
    final_pose3_world = Pose(0.6096, 0.6096, pi()/2.0);
    %final_pose3_world = finalPose(final_pose3_world);
    tform_goal_world = final_pose3_world.bToA();
    tform_inv_robot = trajFollower.init_pose.aToB();
    tform_goal_robot = tform_inv_robot*tform_goal_world;
    final_pose3_relative = Pose.matToPoseVecAsPose(tform_goal_robot);
    
    curve3 = CubicSpiralTrajectory.planTrajectory(final_pose3_relative.x, final_pose3_relative.y, final_pose3_relative.th, 1);
    curve3.planVelocities(.15);

    [r3, r3_size] = trajFollower.executeTrajectorySE(robot, curve3);
    hold on;
    
    bootstrap_pose = final_pose3_world;
    success = 0;
    while(success == 0)
        [bootstrap_pose, success] = trajFollower.localize(10000, bootstrap_pose);
        pause(.01);
    end

    %scatter([final_pose1_world.x final_pose2_world.x final_pose3_world.x], [final_pose1_world.y final_pose2_world.y final_pose3_world.y], 'filled', 'MarkerFaceColor', 'k');
    hold on;
catch ME
    robot.encoders.NewMessageFcn=[];
    robot.encoders.NewMessageFcn=@encoderEventListener;
    if (strcmp(ME.identifier,'TRJFC:ENCODER_ISSUE'))
        msg = [ME.identifier];
        causeException = MException('MAIN:TRJFC:ENCODER_ISSUE',msg);
        ME = addCause(ME,causeException);
    end
   rethrow(ME);
end 














%% merge odometry and pose fixing
clc;
clearvars -except robot;
close all;
format long g;
if(~exist('robot', 'var'))
    robot = raspbot();
    robot.encoders.NewMessageFcn=@encoderEventListener;
    robot.startLaser();
    pause(3);
end
pause(1);

try
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
    plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    plot(0, 0, 'g-', 'Linewidth', 1, 'DisplayName', 'robot');
    hold on;
    %set(signal8, 'xdata', [get(signal8,'xdata') x_g_act(i)], 'ydata', [get(signal8,'ydata') y_g_act(i)]);
    scatter(0,0, 'r', 'filled','DisplayName', 'laser');
    hold on;
    legend('show');


    thePose = Pose(15*.0254,9*.0254,pi()/2);
    lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);


    n = 10000;
    init_pose = Pose(thePose.x,thePose.y,thePose.th);
    actual_robot = RobotState(n, init_pose);
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
    i = actual_robot.i;

    t_i = 0;
    dt_i = 0;
    prevX = getX;
    prevY = getY;
    pT = getT;
    cT = pT;
    curX = prevX;
    curY = prevY;
    while(1)
        r = RobotKeypressDriver.drive(robot,1);
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
        % Odometry stuff        
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
            x_g_act(i+1) = p_i_act.x;
            y_g_act(i+1) = p_i_act.y;
            th_g_act(i+1) = p_i_act.th;
            t(i+1) = t_i;
            V(i+1) = V_i;
            w(i+1) = w_i;
            s(i+1) = s(i) + (V_i*dt_i);
            x(i+1) = p_i_act.x;
            y(i+1) = p_i_act.y;
            th(i+1) = p_i_act.th;

            actual_robot.iPlusPlus;
            
        % Odometry stuff end
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
catch ME
    robot.encoders.NewMessageFcn=[];
    robot.encoders.NewMessageFcn=@encoderEventListener;
    if (strcmp(ME.identifier,'TRJFC:ENCODER_ISSUE'))
        msg = [ME.identifier];
        causeException = MException('MAIN:TRJFC:ENCODER_ISSUE',msg);
        ME = addCause(ME,causeException);
    end
   rethrow(ME);
end 
