function th = findLineYaw(rangeImg)
    th = 0;
    pointThreshold = 0;
    sailThreshold = Inf;
    errorThreshold = Inf;
    sailLength = .125;
% remove all points with bad range
    goodOnes = rangeImg > 0.06 & rangeImg <= .75;
    rangeImg = rangeImg(goodOnes);
    indices = linspace(1,length(goodOnes),length(goodOnes))';
    indices = indices(goodOnes);
% Compute the angles of surviving points
   % th = (indices-1)*(pi/180);
    pointCloud = [indices rangeImg];
    pointCloudXY = RangeImage.irToXy(pointCloud);
    centroid = Pose(0,0,0);
    
    for i = 1:1:length(pointCloudXY(:,1))
        poi = pointCloudXY(i,:);
        sailIndices = ((pointCloudXY(:,1)-poi(1)).^2 + (pointCloudXY(:,2)-poi(2)).^2) < (sailLength/2)^2;
       % sailPoints = pointCloudXY([sailIndices sailIndices])
        x = pointCloudXY(:,1);
        y = pointCloudXY(:,2);
        x = x(sailIndices);
        y = y(sailIndices);
        x_c = x;
        y_c = y;
        x_mean = mean(x);
        y_mean = mean(y);
        x = x-x_mean;
        y = y-y_mean;
        if length(x) < pointThreshold
            continue
        end

        %moment of inertia stuff
         Ixx = x' * x;
        Iyy = y' * y;
        Ixy = - x' * y;
        Inertia = [Ixx Ixy;Ixy Iyy] / length(x); 
        % normalized
        lambda = eig(Inertia);
        lambda = sqrt(lambda)*1000.0;
        sort(lambda);
        %Do checks here 
        if lambda(1) > errorThreshold
            continue
        end    
        if abs(lambda(2) - sailLength) > sailThreshold
            continue
        end
        th = (atan2(2*Ixy,Iyy-Ixx)/2.0);
        th_vect = [cos(th); sin(th)];
        pos_vect = [x_mean; y_mean];
        if dot(th_vect,pos_vect) < 0
            th = mod((th+2*pi),2*pi)-pi;
        end
        
        break

    end

end
