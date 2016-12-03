function [sorted] = sortClosest(robotPose,PoseArray)
%sorts the sails in order of distance from robot
rVec = robotPose.getPoseVec;
x = rVec(1);
y = rVec(2);
vecArray = zeros(4,size(PoseArray,2));
for i = 1:length(PoseArray)
    vecArray(1:3,i) = PoseArray(i).getPoseVec();
    vecArray(4,i) = (vecArray(1,i)-x)^2 + (vecArray(2,i)-y)^2;
end

vecArray = sortrows(vecArray',4)'
for i = 1:length(PoseArray)
    sorted(i) =  Pose(vecArray(1:3,i));
end
end