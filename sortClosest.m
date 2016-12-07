




function sorted = sortClosest(init_pose,PoseArray)
%sorts the sails in order of distance from robot
rVec = init_pose.getPoseVec;
x = rVec(1);
y = rVec(2);
vecArray = zeros(4,size(PoseArray,2));
%disp(size(PoseArray));
for i = 1:size(PoseArray,2)
    goal_pose = PoseArray(i);
    plotRobotAnotateM(goal_pose);    
    vecArray(1:3,i) = PoseArray(i).getPoseVec();
    tform_goal_pose_robot = init_pose.aToB()*goal_pose.bToA();
    goal_pose_robot = Pose.matToPoseVecAsPose(tform_goal_pose_robot);
    curve = CubicSpiralTrajectory.planTrajectory(goal_pose_robot.x, goal_pose_robot.y, goal_pose_robot.th, 1);
    goal_pose.relLength = curve.getTrajectoryDistance();

    plotArray1 = curve.poseArray(1,:);
    plotArray2 = curve.poseArray(2,:);
    w_full = ones(1,length(plotArray1));
    curvePts = [plotArray1; plotArray2; w_full];
    world_curvePts = init_pose.bToA()*curvePts;   
    plot(world_curvePts(1,:),world_curvePts(2,:),'m');
    hold on;
    
    vecArray(4,i) = 1/goal_pose.relLength;
end
sorted = [];
vecArray = sortrows(vecArray',4)';
for i = 1:size(PoseArray,2)
    sorted =  [Pose(vecArray(1:3,i)), sorted];
end


end