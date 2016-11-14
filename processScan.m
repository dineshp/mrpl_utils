%%
%plot range scan
close all;
clc;
figure(1); clf;
hold on;
axis([-1.1 1.1 -1.1 1.1]);
while(1)
    x = zeros(1,1);
    y = zeros(1,1);
    scan = getS;
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
    signal = scatter(x_full, y_full, 'g');
    pause(2);
    delete(signal);
    pause(1);
end


%%
%throw out map points
close all;
clc;
figure(1); clf;
hold on;
axis([-4*.0254 48*.0254 -4*.0254 48*.0254]);
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
    disp(mat2str(x_full));
    
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
    laserPts_full = laserPts;

    ids = lml.throwInliers(init_pose,laserPts);
    laserPts(:,ids) = [];
    world_laserPts = init_pose.bToA()*laserPts;
    
    world_laserPts_full = init_pose.bToA()*laserPts_full;
    
    signal2 = scatter(world_laserPts_full(1,:), world_laserPts_full(2,:), 'r', 'Linewidth', 2);
    signal = scatter(world_laserPts(1,:), world_laserPts(2,:), 'filled', 'g');
    pause(2);
    delete(signal);
    delete(signal2);
    pause(1);
end


%% find sail
close all;
%clear all;
clc;
figure(1); clf;
hold on;
%axis([-4*.0254 48*.0254 -4*.0254 48*.0254]);
%load('rangeImages_map_and_multi_sail_6096_6096_pi-over-2.mat');



x = zeros(1,1);
y = zeros(1,1);

    scan = getS;
    ref = zeros(360, 2);
    for n=1:1:360
        if(abs(scan(n)) <= 1 && abs(scan(n)) > .1)
            x(n) = scan(n)*cos((n-1)*pi/180);
            y(n) = scan(n)*sin((n-1)*pi/180);
        else
          x(n) = 0;
          y(n) = 0;
        end
    end
    x_full = x;
    y_full = y;

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
    laserPts_full = laserPts;
    world_laserPts_full = init_pose.bToA()*laserPts_full;
    %scatter(world_laserPts_full(1,:), world_laserPts_full(2,:), 'r');

    ids = lml.throwInliers(init_pose,laserPts);
    laserPts(:,ids) = 0;
    x = laserPts(1,:);
    y = laserPts(2,:);
    world_laserPts = init_pose.bToA()*laserPts;
    %scatter(world_laserPts(1,:), world_laserPts(2,:), 'g');
    hold on;
    found = 1;
    trajFollower = TrajectoryFollower();

    while(1)
        [plotG, found, pose, laserPts] = findLineCandidate(laserPts);
        if(found == 0)

        end
        pose = finalPose(pose);
        %disp(found);
        tform_curr_pose_world = init_pose.bToA()*pose.bToA();
        curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
        pose = curr_pose_world;
        
        
        
        
        
        start_pose = init_pose;
        goal_pose = pose;

        if(found && plotG)
            [start_pose, goal_pose, state] = trajFollower.executeTrajectory(robot, start_pose, goal_pose, .2);
        end
        
        if(plotG)
            init_pose = trajFollower.last_pose;
            scatter([pose.x], [pose.y],'m');
        
        
        
            hold on;
            %plotRobotAnotate(pose);
        end
        %disp(nnz(x));
        pause(10);
    end


%% localize
x = zeros(1,1);
y = zeros(1,1);
close all;
clear all;
figure;
load('rangeImages_map_and_single_sail_6096_6096_pi-over-2.mat');
for(m=1:1:14)
    thePose = Pose(0.6096, .6096, pi()/2);
    p1 = [0 ; 0];
    p2 = [ 48*.0254 ; 0];
    p3 = [0 ; 48*.0254 ];
    lines_p1 = [p2 p1];
    lines_p2 = [p1 p3];
    plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);

    hold on;
    
    
    scan = transpose(rangeImages(m,:));
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
        error('pose estimate is way off, all points were thrown as outliers');
    end
    
    try
%         x_hull = transpose(laserPts(1,:));
%         y_hull = transpose(laserPts(2,:));
%         ref_hull = [x_hull y_hull];
%         out=convhull(ref_hull);
%         %disp(out2);
%         extremes=ref_hull(out,:);
%         x_l_h = transpose(extremes(:,1));
%         y_l_h = transpose(extremes(:,2));
%         laserPts = [x_l_h; y_l_h; ones(1,length(x_l_h));];
        x_l = x_l(1:10:end);
        y_l = y_l(1:10:end);
        w_l = ones(1,length(x_l));
        laserPts = [x_l; y_l; w_l];
    catch
        disp('conv hull failed');
        x_l = x_l(1:10:end);
        y_l = y_l(1:10:end);
        w_l = ones(1,length(x_l));
        laserPts = [x_l; y_l; w_l];
    end
    
    
    
    start = tic;
    [success,outPose] = lml.refinePose(thePose,laserPts, 10000);
    disp(toc(start));
    thePose = outPose;
    plotRobotAnotate(thePose);

    laserPtsPlot = thePose.bToA()*laserPts;
    signal7 = scatter(laserPtsPlot(1,:), laserPtsPlot(2,:), 'filled', 'r');
    hold on;
    %disp(thePose.getPoseVec);
    pause(1);
    clf;
end


%%
%localize robot
x = zeros(1,1);
y = zeros(1,1);
close all;
clearvars -except robot;
figure;
while(1)
    thePose = Pose(0.6096/2, .6096/2, pi()/2);
    p1 = [0 ; 0];
    p2 = [ 48*.0254 ; 0];
    p3 = [0 ; 48*.0254 ];
    lines_p1 = [p2 p1];
    lines_p2 = [p1 p3];
    plot(lines_p1(1,:), lines_p1(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    plot(lines_p2(1,:), lines_p2(2,:), 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    hold on;
    lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);

    hold on;
    
    
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
        error('pose estimate is way off, all points were thrown as outliers');
    end
        start = tic;
    try
%         x_hull = transpose(laserPts(1,:));
%         y_hull = transpose(laserPts(2,:));
%         ref_hull = [x_hull y_hull];
%         out=convhull(ref_hull);
%         %disp(out2);
%         extremes=ref_hull(out,:);
%         x_l_h = transpose(extremes(:,1));
%         y_l_h = transpose(extremes(:,2));
%         laserPts = [x_l_h; y_l_h; ones(1,length(x_l_h));];
        x_l = x_l(1:10:end);
        y_l = y_l(1:10:end);
        w_l = ones(1,length(x_l));
        laserPts = [x_l; y_l; w_l];
        [success,outPose] = lml.refinePose(thePose,laserPts, 10000);
    catch
        disp('conv hull failed');
        x_l = x_l(1:10:end);
        y_l = y_l(1:10:end);
        w_l = ones(1,length(x_l));
        laserPts = [x_l; y_l; w_l];
        [success,outPose] = lml.refinePose(thePose,laserPts, 10000);
    end
    
    
    disp(toc(start));
    disp(outPose.getPoseVec());
    thePose = outPose;
    plotRobotAnotate(thePose);

    laserPtsPlot = thePose.bToA()*laserPts;
    signal7 = scatter(laserPtsPlot(1,:), laserPtsPlot(2,:), 'filled', 'r');
    hold on;
    %disp(thePose.getPoseVec);
    pause(1);
    clf;
end




%%
clc;
close all;
figure;
clf;
clearvars -except robot;
trajFollower = TrajectoryFollower();
trajFollower.last_pose = Pose(0.6096, 0.6096, pi()/2.0);
clc;
close all;
figure;
clf;
while(1);
    foundSail = trajFollower.go_to_sail(robot);   
    if(foundSail)
        pause(5);
        trajFollower.backUp(robot);
        pause(5);
        trajFollower.turn(robot);
    end
    
    pause(10);
    clf;
end


%%
clc;
close all;

clearvars -except robot;
while(1)
    turnRelAngle(robot,pi()/2.0);
    pause(2);
end