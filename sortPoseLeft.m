function [sorted] = sortPoseLeft(PoseArray)
%sorts the poses in order of left to right wrt map coordinates
vecArray = zeros(3,size(PoseArray,2));
for i = 1:length(PoseArray)
    vecArray(:,i) = PoseArray(i).getPoseVec();
end
vecArray = sortrows(vecArray',-1)'
for i = 1:length(PoseArray)
    sorted(i) =  Pose(vecArray(:,i));
end
end

