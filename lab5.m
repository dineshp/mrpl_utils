%% lab5 early steps
clearvars; 
fig8 = Figure8ReferenceControl(3, 1, .5);
traj = RobotTrajectory(fig8);
%traj.plotIntDensityTraj;
traj.plotInterpTraj(100);


%% lab5 simulator
clearvars -except robot;
close all;
fig8_del = Figure8ReferenceControl(3, 1, 0);
traj_model = RobotTrajectory(fig8_del);
fig8_ref = Figure8ReferenceControl(3, 1, .5);
trajFollowSim = TrajectoryFollowerSimulator(traj_model, fig8_ref);

%% lab5 challenge task
clearvars -except robot traj_model;
close all;
format long g;
robot.encoders.NewMessageFcn=@encoderEventListener;
clearvars -except robot traj_model;
pause(1);
if(~exist('traj_model', 'var'))
fig8_del = Figure8ReferenceControl(3, 1, .2);
fig8_ref = Figure8ReferenceControl(3, 1, 0);
traj_model = RobotTrajectory(fig8_del, fig8_ref);
else 
r = TrajectoryFollower(robot, traj_model);
end
robot.encoders.NewMessageFcn=[];
clearvars -except robot traj_model;