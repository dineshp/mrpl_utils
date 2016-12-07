%% generate range image data
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
    %number of range images
    n = 15;
    rangeImages = zeros(n, 360);
    
    for i=1:1:n
        scan = robot.laser.LatestMessage;
        rangeImg = transpose(scan.Ranges);
        rangeImages(i,:) = rangeImg;
        load gong.mat;
        %soundsc(y);
        pause(.5);
    end
    
    load gong.mat;
    soundsc(y);
    load gong.mat;
    soundsc(y);
    load gong.mat;
    soundsc(y);
    save('rangeImages_final_4ft_3ft_pi-over-2', 'rangeImages');
    
    
    %[m, n] = size(rangeImages);
    %fprintf('%f %f', m, n); 
    %rangeBrightness = scan.Intensities;
    %showdetails(scan);
    %[m, n] = size(rangeImg);
    %fprintf('%f %f', m, n);
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

%%
%plot range scan
close all;
clc;
figure(1); clf;
hold on;
axis([-1.1 1.1 -1.1 1.1]);
for i=1:1:14
    x = zeros(1,1);
    y = zeros(1,1);
    scan = transpose(rangeImages(i,:));
    ref = zeros(360, 2);
    for n=1:1:360
        if(abs(scan(n)) <= 1 && abs(scan(n)) > .06)
            x(n) = scan(n)*cos((n-1)*pi/180);
            y(n) = scan(n)*sin((n-1)*pi/180);
        else
          x(n) = 0;
          y(n) = 0;
        end
    end
    x_full = x;
    y_full = y;
    
    figure;
    p1 = [0 ; 0];
    p2 = [ 48*.0254 ; 0];
    p3 = [0 ; 48*.0254 ];
    lines_p1 = [p2 p1];
    lines_p2 = [p1 p3];
    
    plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    lml = LineMapLocalizer(lines_p1,lines_p2,.01,.005,.0005);
    
    hold on;
    init_pose = Pose(0.6096, 0.6096, pi()/2.0);
    
    w_full = ones(1,length(x_full));
    laserPts = [x_full; y_full; w_full];

    ids = lml.throwInliers(init_pose,laserPts);
    laserPts(:,ids) = [];
    
    signal = scatter(laserPts(1,:), laserPts(2,:), 'g');
    signal2 = scatter(x_full, y_full, 'r');
    pause(2);
    delete(signal);
    pause(1);
end




%% find sail
close all;
clc;
figure(1); clf;
hold on;
axis([-1.1 1.1 -1.1 1.1]);
for i=1:1:14
    x = zeros(1,1);
    y = zeros(1,1);
    scan = transpose(rangeImages(i,:));
    ref = zeros(360, 2);
    for n=1:1:360
        if(abs(scan(n)) <= 1 && abs(scan(n)) > .06)
            x(n) = scan(n)*cos((n-1)*pi/180);
            y(n) = scan(n)*sin((n-1)*pi/180);
        else
          x(n) = 0;
          y(n) = 0;
        end
    end
    x_full = x;
    y_full = y;
    
    lidsx = (x ~= 0);
    lidsy = (y ~= 0);
    lids = lidsx .* lidsy;
    lids = logical(lids);
    x = x(lids);
    y = y(lids);
    
    first_sail_pt = [x(1) y(1)];
    last_sail_pt = [x(end) y(end)];
    
    
    x = x(2:length(x)-1);
    y = y(2:length(y)-1);
    
    
    hull_ref = [transpose(x) transpose(y)];
    
    hull_indices = convhull(hull_ref);
    hull_pts=hull_ref(hull_indices,:);
    
    x_hull_pts = transpose(hull_pts(:,1));
    y_hull_pts = transpose(hull_pts(:,2));
    
    end_pt1 = hull_pts(1,:);
    %compute Euclidean distances:
    distances = sqrt(sum(bsxfun(@minus, hull_pts, end_pt1).^2,2));
    end_pt2 = hull_pts(find(distances==max(distances)),:); 
       
    x1 = end_pt1(1);
    y1 = end_pt1(2);
    x2 = end_pt2(1);
    y2 = end_pt2(2);   
    fitvars = polyfit(x_hull_pts, y_hull_pts, 1);
    best_fit_slope = fitvars(1);
    
    m_sail_start_slope = (first_sail_pt(2) - y1)/(first_sail_pt(1) - x1);
    d_error = pdist([end_pt1; first_sail_pt],'euclidean');
    slope_error = abs(m_sail_start_slope - best_fit_slope);
    if(d_error <= .0175 && slope_error <= 3.25)
        x1 = first_sail_pt(1);
        y1 = first_sail_pt(2);
    end
    
    
    m_sail_end_slope = (last_sail_pt(2) - y2)/(last_sail_pt(1) - x2);
    d_error = pdist([end_pt2; last_sail_pt],'euclidean');
    slope_error = abs(m_sail_end_slope - best_fit_slope);
    if(d_error <= .0175 && slope_error <= 3.25)
        x2 = last_sail_pt(1);
        y2 = last_sail_pt(2);
    end
    
    
    th = atan2(-(x2-x1),(y2-y1));
    best_fit_mid = midPoint([[x1 y1] [x2 y2]]);
    pose = Pose(best_fit_mid(1), best_fit_mid(2), th);
    pose = finalPose(pose);
    
    scatter(x_full, y_full, 'g');
    scatter(best_fit_mid(1), best_fit_mid(2), 'filled', 'MarkerFaceColor', 'm');
    plot([best_fit_mid(1) pose.x], [best_fit_mid(2) pose.y],'m');
    
    
    plotRobotAnotate(pose);
    
    
    

    pause(.01);
end




%% Drive to object
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
    
    figure;
    hold on;
    trajFollower = TrajectoryFollowerC();
    
    %number of range images
    n = 15;
    rangeImages = zeros(n, 360);
    
    for i=1:1:n
        scan = robot.laser.LatestMessage;
        rangeImg = transpose(scan.Ranges);
        rangeImagesDrive(i,:) = rangeImg;
        p_pose = findLineCandidate(scan.Ranges);
        if(p_pose.x == 0 && p_pose.y == 0 && p_pose.th == 0)
            continue;
        end
        pose = finalPose(p_pose);
        curve = CubicSpiralTrajectory.planTrajectory(pose.x, pose.y, pose.th, 1);
        curve.planVelocities(.2);
        [r, r_size] = trajFollower.executeTrajectory(robot, curve, .2);
        hold on;
        load gong.mat;
        soundsc(y);
        pause(10);
        
    end
    
    pause(1);
    load gong.mat;
    soundsc(y);
    pause(1);
    load gong.mat;
    soundsc(y);
    pause(1);
    load gong.mat;
    soundsc(y);
    
    
    %[m, n] = size(rangeImages);
    %fprintf('%f %f', m, n); 
    %rangeBrightness = scan.Intensities;
    %showdetails(scan);
    %[m, n] = size(rangeImg);
    %fprintf('%f %f', m, n);
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
