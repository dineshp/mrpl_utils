classdef LineMapLocalizer < handle
    %mapLocalizer A class to match a range scan against a map in
    % order to find the true location of the range scan relative to
    % the map.
    properties(Constant)
        maxErr = 0.05; % 5 cm
        minPts = 5; % min # of points that must match
    end
    properties(Access = private)
    end
    properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.01;
        errThresh = 0.001;
        gradThresh = 0.0005;
    end
    
    methods(Static = true)
        function [rad2, po] = closestPointOnLineSegment(pi,p1,p2)
            % Given set of points and a line segment, returns the
            % closest point and square of distance to segment for
            % each point. If the closest point is an endpoint, returns
            % infinity for rad2 because such points are bad for
            % lidar matching localization.
            %
            % [rad2, po] = CLOSESTPOINTONLINESEGMENT(pi,p1,p2)
            %
            % pi - Array of points of size 2 x n.
            % p1 - Column of size 2, endpoint of segment.
            % p2 - Column of size 2, endpoint of segment.
            %
            % rad2 - Squared distance to closest point on segment.
            % po - Closest points on segment. Same size as pi.
            v1 = bsxfun(@minus,pi,p1);
            v2 = p2-p1;
            v3 = bsxfun(@minus,pi,p2);
            v1dotv2 = bsxfun(@times,v1,v2);
            v1dotv2 = sum(v1dotv2,1);
            v2dotv2 = sum(v2.*v2);
            v3dotv2 = bsxfun(@times,v3,v2);
            v3dotv2 = sum(v3dotv2,1);
            nPoints = size(pi,2);
            rad2 = zeros(1,nPoints);
            po = zeros(2,nPoints);
            % Closest is on segment
            flag1 = v1dotv2 > 0.0 & v3dotv2 < 0.0;
            if any(flag1)
                scale = v1dotv2/v2dotv2; temp = bsxfun(@plus,v2*scale,[p1(1) ; p1(2)]);
                po(:,flag1) = temp(:,flag1);
                dx = pi(1,flag1)-po(1,flag1);
                dy = pi(2,flag1)-po(2,flag1);
                rad2(flag1) = dx.*dx+dy.*dy;
            end
            % Closest is first endpoint
            flag2 = v1dotv2 <= 0.0;
            if any(flag2)
                temp = bsxfun(@times,ones(2,sum(flag2)),[p1(1); p1(2)]);
                po(:,flag2) = temp;
                rad2(flag2) = inf;
                
            end
            % Closest is second endpoint
            flag3 = ~flag1 & ~flag2;
            if any(flag3)
                temp = bsxfun(@times,ones(2,sum(flag3)),[p2(1); p2(2)]);
                po(:,flag3) = temp;
                rad2(flag3) = inf;
            end
        end
        
        
    end
    
    methods(Access = public)
        function obj = LineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
            % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
        end
        
        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = obj.closestPointOnLineSegment(pi,...
                    obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(sqrt(r2) > obj.maxErr);
        end
        
       function ids = throwInliers(obj,pose,scan_homogeneous)
            % Find ids of outliers in a scan.
            worldPts = pose.bToA()*scan_homogeneous;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(sqrt(r2) < .05);
        end
        
        function avgErr = fitError(obj,pose,ptsInModelFrame)
            % Find the standard deviation of perpendicular distances of
            % all points to all lines
            % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err = sum(r2);
            num = length(r2);
            if(num >= LineMapLocalizer.minPts)
                avgErr = sqrt(err)/num;
            else
                % not enough points to make a guess
                avgErr = inf;
            end
        end
        
        function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            errPlus0 = fitError(obj,poseIn,modelPts);
            eps = 0.001;
            J = zeros(1,3);
            for i =1:3
                dp = zeros(3,1);
                dp(i) = eps;
                
                newPose = Pose(poseIn.getPoseVec+dp);
                errPlusI = fitError(obj,newPose,modelPts);
                J(i) = (errPlusI-errPlus0) ./eps;
            end
        end
        
        %copy paste into localization class
function [success,outPose] = refinePose(obj,inPose,ptsInModelFrame, maxIters)
            %figure(1)
            %Throw out outliers
            options = optimset( 'algorithm', {'levenberg-marquardt',.1}, ...
                    'DerivativeCheck', 'off', ...
                    'TolX', obj.errThresh, ...
                    'Display', 'off', ...
                    'MaxIter', maxIters );
            ids = obj.throwOutliers(inPose,ptsInModelFrame);
            inPoseVec = inPose.getPoseVec;
            ptsInModelFrame(:,ids) = [];
            %determine if worked
           % iterations = 1;
            % gain = .01;
            % errThresh = .001;
            % gradThresh = .001;
            
            %outPose = inPose;
            %J = Inf(1,3);
            obj.fitErrorVec(inPoseVec,ptsInModelFrame);
            outPoseVec= lsqnonlin( @(poseVec) obj.fitErrorVec(poseVec,ptsInModelFrame), inPoseVec, [], [], options );

%             while ((iterations < maxIters) && (norm(J) > obj.gradThresh) )
%                 [errPlus0,J] = obj.getJacobian(outPose,ptsInModelFrame);
%                   scatter(iterations,errPlus0);
%                   hold on;
%                 if (errPlus0 < obj.errThresh)
%                     success = 1;
%                     break
%                 end
%                                   
%                 outVec = outPose.getPoseVec - obj.gain*J';
%                 outPose = Pose(outVec);
%                 figure(iterations)
%                              hold on;
%                               plot(obj.lines_p1(1,:),obj.lines_p1(2,:));
%                             plot(obj.lines_p2(1,:),obj.lines_p2(2,:));
%                               worldLidarPts = outPose.bToA()*ptsInModelFrame;
%                             scatter(worldLidarPts(1,:),worldLidarPts(2,:));
%                             waitforbuttonpress;
%                 iterations = iterations + 1;
%             end
outPose = Pose(outPoseVec);
errPlus0 = fitError(obj,outPose,ptsInModelFrame);
success = (errPlus0 < obj.errThresh);
          
end
        
function avgErr = fitErrorVec(obj,poseVec,ptsInModelFrame)
            % Find the standard deviation of perpendicular distances of
            % all points to all lines
            % transform the points
            pose= Pose(poseVec);
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            %err = sum(r2);
           
            num = length(r2);
            avgErr = zeros(1,360);
            if(num >= obj.minPts)
                 avgErr(1:num) = sqrt(r2);
               % avgErr = sqrt(err)/num;
            else
                % not enough points to make a guess
                avgErr = Inf(1, 360);
            end
end
    end
    
    methods(Access = private)
        
        
    end
end