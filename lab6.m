%% section1

close all;
clearvars;
traj = CubicSpiralTrajectory([-3.0 1.55 3.0],1001);
traj.plot();
pose = traj.getFinalPose();
fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));

%% section2
close all;
CubicSpiralTrajectory.makeLookupTable(100);


% curve = CubicSpiralTrajectory([-3.0 1.55 3.0],10001);
% pose = curve.getFinalPose();
% plot(pose(1),pose(2));
% fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));

% curve = CubicSpiralTrajectory([-3.0 1.55 3.0],100001);
% pose = curve.getFinalPose();
% plot(pose(1),pose(2));
% fprintf('x:%f y:%f t:%f\n',pose(1),pose(2),pose(3));

%% challenge task curves
curve = CubicSpiralTrajectory.planTrajectory(.25, .25, 0.0, 1);
curve.plot();

curve2 = CubicSpiralTrajectory.planTrajectory(.25, .25, 0.0, 1);
curve2.plot();
curve.planVelocities(.25);


%% testing just plan and plotting trajectory
clearvars -except robot;
clc;
close all;

hold on;
curve = CubicSpiralTrajectory.planTrajectory(.25, .25, 0.0, 1);
curve.plot();
hold on;
curve2 = CubicSpiralTrajectory.planTrajectory(-.5, -.5, -pi/2.0, 1);
%curve2.plot();
%hold on;
curve3 = CubicSpiralTrajectory.planTrajectory(-.25, .25, pi/2.0, 1);
%curve3.plot();
%hold on;

xlim([-.5 .5]);
ylim([-.5 .5]);

finalPose1World = Pose(curve.poseArray(1,curve.numSamples), curve.poseArray(2,curve.numSamples), curve.poseArray(3,curve.numSamples));
tform = finalPose1World.bToA();
curve2_x_aff_ref = zeros(1, curve.numSamples);
curve2_y_aff_ref = zeros(1, curve.numSamples);
curve2_th_aff_ref =   zeros(1, curve.numSamples);
for i=1:1:curve.numSamples
    t = tform*[curve2.poseArray(1,i); curve2.poseArray(2,i); 1];
    curve2_x_aff_ref(i) = t(1);
    curve2_y_aff_ref(i) = t(2);
end
plot(curve2_x_aff_ref, curve2_y_aff_ref, 'g');

finalPose2World = Pose(curve2_x_aff_ref(end), curve2_y_aff_ref(end), curve2.poseArray(3,curve.numSamples));
tform2 = finalPose2World.bToA();
curve3_x_aff_ref = zeros(1, curve.numSamples);
curve3_y_aff_ref = zeros(1, curve.numSamples);
for i=1:1:curve.numSamples
    t = tform2*[curve3.poseArray(1,i); curve3.poseArray(2,i); 1];
    curve3_x_aff_ref(i) = t(1);
    curve3_y_aff_ref(i) = t(2);
end

plot(curve3_x_aff_ref, curve3_y_aff_ref, 'b');

 
scatter([curve.poseArray(1,curve.numSamples) curve2_x_aff_ref(end) curve3_x_aff_ref(end)], [curve.poseArray(2,curve.numSamples) curve2_y_aff_ref(end) curve3_y_aff_ref(end)], 'filled', 'MarkerFaceColor', 'k');

%% Challenge Task

%v.8.11 removed feedback, added global kurv and global dist error (new local variables to help), added exception for freezing in encoder update loop, removed feedback, changed annotation control, changed order of updating actual state and reference state

% TEST EVERYTHING WORKS

% to do: create MrplSystem which takes sequence of end state vectors and:
% creates curves, executes trajectories -- graphs actual and reference in
% real time and updates error in real time (keeps track of global error and
% odemtry is transformed), prints end state of actual and reference after execution of each curve, 
% graphs highly dense trajectories & odometries on
% seperate figure 


clc;
clearvars -except robot;
close all;
format long g;
if(~exist('robot', 'var'))
    robot = raspbot();
    robot.encoders.NewMessageFcn=@encoderEventListener;
    pause(3);
end
pause(1);


try
    worldPose = Pose(0, 0, 0);

    curve = CubicSpiralTrajectory.planTrajectory(.25, .25, 0.0, 1);
    curve.planVelocities(.2);
    curve2 = CubicSpiralTrajectory.planTrajectory(-.5, -.5, -pi/2.0, 1);
    curve2.planVelocities(.2);
    curve3 = CubicSpiralTrajectory.planTrajectory(-.25, .25, pi/2.0, 1);
    curve3.planVelocities(.2);

    figure;
    hold on;
    trajFollower = TrajectoryFollowerC();
    [r, r_size] = trajFollower.executeTrajectory(robot, curve, .2);
    hold on;
    pause(2);

    [r2, r2_size] = trajFollower.executeTrajectory(robot, curve2, .2);
    hold on;
    pause(2);

    [r3, r3_size] = trajFollower.executeTrajectory(robot, curve3, .2);
    hold on;
    
    %reference figure
    title(['Transformed Reference Trajectory']);
    xlim([-0.6 0.6]);
    ylim([-0.6 0.6]);
    hold on;

    plot(curve.poseArray(1,:), curve.poseArray(2, :), 'k', 'LineWidth', .5);
    hold on;

    finalPose1World = Pose(curve.poseArray(1,curve.numSamples), curve.poseArray(2,curve.numSamples), curve.poseArray(3,curve.numSamples));
    tform = finalPose1World.bToA();
    curve2_x_aff_ref = zeros(1, curve.numSamples);
    curve2_y_aff_ref = zeros(1, curve.numSamples);
    for i=1:1:curve.numSamples
        t = tform*[curve2.poseArray(1,i); curve2.poseArray(2,i); 1];
        curve2_x_aff_ref(i) = t(1);
        curve2_y_aff_ref(i) = t(2);
    end
    plot(curve2_x_aff_ref, curve2_y_aff_ref, 'k', 'LineWidth', .5);
    hold on;

    finalPose2World = Pose(curve2_x_aff_ref(end), curve2_y_aff_ref(end), curve2.poseArray(3,curve.numSamples));
    tform2 = finalPose2World.bToA();
    curve3_x_aff_ref = zeros(1, curve.numSamples);
    curve3_y_aff_ref = zeros(1, curve.numSamples);
    for i=1:1:curve.numSamples
        t = tform2*[curve3.poseArray(1,i); curve3.poseArray(2,i); 1];
        curve3_x_aff_ref(i) = t(1);
        curve3_y_aff_ref(i) = t(2);
    end
    plot(curve3_x_aff_ref, curve3_y_aff_ref, 'k', 'LineWidth', .5);
    hold on;

    scatter([curve.poseArray(1,curve.numSamples) curve2_x_aff_ref(end) curve3_x_aff_ref(end)], [curve.poseArray(2,curve.numSamples) curve2_y_aff_ref(end) curve3_y_aff_ref(end)], 'filled', 'MarkerFaceColor', 'k');
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
