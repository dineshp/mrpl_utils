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
    %init_pose = Pose(.6096,.6090,pi()/2);
    init_pose = Pose(0,0,0);
    p1 = [0 ; 0];
    p2 = [ 48*.0254 ; 0];
    p3 = [0 ; 48*.0254 ];
    lines_p1 = [p2 p1];
    lines_p2 = [p1 p3];
    lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);
    trajFollower = TrajectoryFollowerC(init_pose, lml);
    
    
    curve = CubicSpiralTrajectory.planTrajectory(.25, .25, 0.0, 1);
    curve.planVelocities(.2);
    curve2 = CubicSpiralTrajectory.planTrajectory(-.5, -.5, -pi/2.0, 1);
    curve2.planVelocities(.2);
    curve3 = CubicSpiralTrajectory.planTrajectory(-.25, .25, pi/2.0, 1);
    curve3.planVelocities(.2);
    %calculate robot relative poses
    %
    %1. xf = 0.3048; yf = 0.3048; thf = pi()/2.0;
    %final_pose1_world = Pose(0.3048, 0.3048, pi()/2.0);
    %tform_goal_world = final_pose1_world.bToA();
    %tform_inv_robot = init_pose.aToB();
    %tform_goal_robot = tform_inv_robot*tform_goal_world;
    %final_pose1_relative = Pose.matToPoseVecAsPose(tform_goal_robot);
    %
    %2. xf = 0.9144; yf = 0.3048; thf = 0.0;
    %final_pose2_world = Pose(0.9144, 0.3048, 0.0;
    %tform_goal_world = final_pose2_world.bToA();
    %tform_inv_robot = final_pose1_world.aToB();
    %tform_goal_robot = tform_inv_robot*tform_goal_world;
    %final_pose2_relative = Pose.matToPoseVecAsPose(tform_goal_robot);
    %
    %3. xf = 0.6096; yf = 0.6096; thf = pi()/2.0;
    %final_pose3_world = Pose(0.6096, 0.6096, pi()/2.0);
    %tform_goal_world = final_pose3_world.bToA();
    %tform_inv_robot = final_pose2_world.aToB();
    %tform_goal_robot = tform_inv_robot*tform_goal_world;
    %final_pose3_relative = Pose.matToPoseVecAsPose(tform_goal_robot);
    %
    %curve = CubicSpiralTrajectory.planTrajectory(final_pose1_relative.x, final_pose1_relative.y, final_pose1_relative.th, 1);
    %curve.planVelocities(.2);
    %curve2 = CubicSpiralTrajectory.planTrajectory(final_pose2_relative.x, final_pose2_relative.y, final_pose2_relative.th, 1);
    %curve2.planVelocities(.2);
    %curve3 = CubicSpiralTrajectory.planTrajectory(final_pose3_relative.x, final_pose3_relative.y, final_pose3_relative.th, 1);
    %curve3.planVelocities(.2);
    
    figure;
    hold on;
    [r, r_size] = trajFollower.executeTrajectory(robot, curve);
    hold on;
    pause(2);

    [r2, r2_size] = trajFollower.executeTrajectory(robot, curve2);
    hold on;
    pause(2);

    [r3, r3_size] = trajFollower.executeTrajectory(robot, curve3);
    hold on;
    
    %reference figure
%     title(['Transformed Reference Trajectory']);
%     xlim([-0.6 0.6]);
%     ylim([-0.6 0.6]);
%     hold on;

%     plot(curve.poseArray(1,:), curve.poseArray(2, :), 'k', 'LineWidth', .5);
%     hold on;

    finalPose1World = Pose(curve.poseArray(1,curve.numSamples), curve.poseArray(2,curve.numSamples), curve.poseArray(3,curve.numSamples));
    tform = finalPose1World.bToA();
    curve2_x_aff_ref = zeros(1, curve.numSamples);
    curve2_y_aff_ref = zeros(1, curve.numSamples);
    for i=1:1:curve.numSamples
        t = tform*[curve2.poseArray(1,i); curve2.poseArray(2,i); 1];
        curve2_x_aff_ref(i) = t(1);
        curve2_y_aff_ref(i) = t(2);
    end
%     plot(curve2_x_aff_ref, curve2_y_aff_ref, 'k', 'LineWidth', .5);
%     hold on;

    finalPose2World = Pose(curve2_x_aff_ref(end), curve2_y_aff_ref(end), curve2.poseArray(3,curve.numSamples));
    tform2 = finalPose2World.bToA();
    curve3_x_aff_ref = zeros(1, curve.numSamples);
    curve3_y_aff_ref = zeros(1, curve.numSamples);
    for i=1:1:curve.numSamples
        t = tform2*[curve3.poseArray(1,i); curve3.poseArray(2,i); 1];
        curve3_x_aff_ref(i) = t(1);
        curve3_y_aff_ref(i) = t(2);
    end
%     plot(curve3_x_aff_ref, curve3_y_aff_ref, 'k', 'LineWidth', .5);
%     hold on;

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
