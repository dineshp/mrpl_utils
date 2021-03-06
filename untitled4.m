%%
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


start_pose = Pose(0.6096, 0.6096, pi()/2.0);
goal_pose = Pose(0.6096/2, 0.6096/2, -pi);
trajFollower = TrajectoryFollower();





[start_pose, goal_pose, state] = trajFollower.executeTrajectory(robot, start_pose, goal_pose, .2);
%start_pose = goal_pose;
%goal_pose =  Pose(0.6096, 0.6096, 0);
%[start_pose, goal_pose, state] = trajFollower.executeTrajectorySim(start_pose, goal_pose, .2);


