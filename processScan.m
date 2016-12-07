%%
%plot range scan

clc;
close all;
clearvars -except robot;
figure;
% axis([-.25 4 -.25 4]);

off = .0254*6;
p1 = [0 ; 0];
p2 = [ 12*8*.0254 ; 0];
p3 = [0 ; 12*12*.0254 ];
p4 = [12* 8*.0254; 12*12*.0254];
lines_p1 = [p2-[off;0] p1+[0;off] p2+[0;off] p3+[off;0]];
lines_p2 = [p1+[off;0] p3-[0;off] p4-[0;off] p4-[off;0]];
% for i = 1:length(lines_p1(1,:))
%     hold on;
%     
%     plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
% end
robot = raspbot;
robot.laser.NewMessageFcn=@laserEventListener;
robot.encoders.NewMessageFcn=@encoderEventListener;
pause(1);
signal = scatter(0, 0, 'g');
while(1)
    delete(signal);
    x = zeros(1,1);
    y = zeros(1,1);
    scan = getS;
    ref = zeros(360, 2);
    for n=1:1:360
        th = (n-1)*pi/180;
        wedge_width = pi()/5;
        if((abs(scan(n)) <= 1.3 && abs(scan(n)) > .1) && ((th <= wedge_width) || (th >= 2*pi() - wedge_width)) )
            x(n) = scan(n)*cos(th);
            y(n) = scan(n)*sin(th);
        else
          x(n) = 0;
          y(n) = 0;
        end
    end
    x_full = x;
    y_full = y;
    signal = scatter(x_full, y_full, 'r');
    pause(.1);
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
axis([-4*.0254 48*.0254 -4*.0254 48*.0254]);
load('rangeImages_map_and_multi_sail_6096_6096_pi-over-2.mat');



for i=1:1:14
    x = zeros(1,1);
    y = zeros(1,1);
    scan = transpose(rangeImages(i,:));
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
    
    init_pose = Pose(3*0.3048, 3*0.3048, pi()/2.0);
    plotRobotAnotateC(init_pose);

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
    found = 1;
    plotG = 0;
    trajFollower = TrajectoryFollower();
    poses = [];
    while(found == 1)
        [plotG, found, pose, laserPts] = findLineCandidate(laserPts, init_pose);
        pose = finalPose(pose);
        %disp(found);
        tform_curr_pose_world = init_pose.bToA()*pose.bToA();
        curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
        pose = curr_pose_world;
         
        goal_pose = pose;
        if(plotG)
            plotRobotAnotateM(pose);           
            poses = [pose, poses];
            hold on;
        end
        
        %disp(nnz(x));
    end
    sorted = sortClosest(init_pose, poses); 
    for i=1:length(sorted)
        txt = sprintf(' %d ', i);
        txt = strcat('\leftarrow ', txt); 
        pose = sorted(i);
        

        
        t = text(pose.x,pose.y,txt,'FontSize',10);
        t.Color = 'k';
        hold on;
    end
    pause(5);
    clf;
end

%% find sail ROBOT
close all;
%clear all;
clc;
figure(1); clf;
hold on;
axis([-4*.0254 48*.0254 -4*.0254 48*.0254]);
load('rangeImages_map_and_multi_sail_6096_6096_pi-over-2.mat');



while(1)
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
    
    init_pose = Pose(3*0.3048, 3*0.3048, pi()/2.0);
    plotRobotAnotateC(init_pose);

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
    found = 1;
    plotG = 0;
    trajFollower = TrajectoryFollower();
    poses = [];
    while(found == 1)
        [plotG, found, pose, laserPts] = findLineCandidate(laserPts, init_pose);
        pose = finalPose(pose);
        %disp(found);
        tform_curr_pose_world = init_pose.bToA()*pose.bToA();
        curr_pose_world = Pose.matToPoseVecAsPose(tform_curr_pose_world);
        pose = curr_pose_world;
         
        goal_pose = pose;
        if(plotG)
            plotRobotAnotateM(pose);           
            poses = [pose, poses];
            hold on;
        end
        
        %disp(nnz(x));
    end
    sorted = sortClosest(init_pose, poses); 
    for i=1:length(sorted)
        txt = sprintf(' %d ', i);
        txt = strcat('\leftarrow ', txt); 
        pose = sorted(i);
        

        
        t = text(pose.x,pose.y,txt,'FontSize',10);
        t.Color = 'k';
        hold on;
    end
    pause(5);
    clf;
end
%% localize
x = zeros(1,1);
y = zeros(1,1);
clc;
close all;
clearvars -except robot;
figure;
axis([-.25 4 -.25 4]);
thePose = Pose(.3048, 6*.3048, -pi()/2);
off = 0;%.0254*6;
p1 = [0 ; 0];
p2 = [ 12*8*.0254 ; 0];
p3 = [0 ; 12*12*.0254 ];
p4 = [12* 8*.0254; 12*12*.0254];
lines_p1 = [p2-[off;0] p1+[0;off] p2+[0;off] p3+[off;0]];
lines_p2 = [p1+[off;0] p3-[0;off] p4-[0;off] p4-[off;0]];
lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);
while(1)


    for i = 1:length(lines_p1(1,:))
        hold on;

        plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    end
    
    
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




%% Challenge Task
clc;
close all;
figure;
axis([-.25 1.3 -.25 1.3]);
clf;
clearvars -except robot;
trajFollower = TrajectoryFollower();
trajFollower.last_pose = Pose(.3048, .3048, -pi()/2.0);

clc;
close all;
figure;
clf;
while(1);
    robot.forksDown();
    pause(0.5);
    foundSail = trajFollower.go_to_sail(robot); 
    pause(2);
    if(foundSail)
        
        trajFollower.ramSail(robot);
        pause(2);
        robot.forksUp();
        pause(2);
        robot.forksDown();
        pause(2);
        trajFollower.backUp(robot);
        pause(2);
        trajFollower.turn(robot);
    end
    
    pause(10);
    clf;
end



%% Challenge Task 2
clc;
close all;
figure;
clf;
clearvars -except robot;
trajFollower = TrajectoryFollower();
%trajFollower.last_pose = Pose(0.5, 0.5, pi()/2.0);

clc;
close all;
figure;
clf;
while(1);
    robot.forksDown();
    pause(0.5);
    foundSail = trajFollower.go_to_sailNL(robot); 
    pause(2);
    if(foundSail)
        trajFollower.ramSailNL(robot);
        pause(2);
        robot.forksUp();
        pause(2);
        robot.forksDown();
        pause(2);
        trajFollower.backUpNL(robot);
        pause(2);
        trajFollower.turnNL(robot);
    end
    
    pause(10);
    clf;
end