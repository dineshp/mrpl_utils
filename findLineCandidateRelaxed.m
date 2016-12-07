function [plotG, found, centroid, laserPts] = findLineCandidateRelaxed(laserPts, pose)
    plotG = 0;
    found = 0;
    x_full = laserPts(1,:);
    y_full = laserPts(2,:);
    x = x_full;
    y = y_full;
    sailLength = .125;
    init_pose = pose;
    centroid = Pose(0,0,0);
    
    
    %disp(length(laserPts));
    
    
    
    
    for j = 1:1:length(x)
        
        if((x(j) ==0) && (y(j) ==0))
            continue;
        end
        poi = [x(j) y(j)];
        sailIndices = ((x-poi(1)).^2 + (y-poi(2)).^2) < (sailLength/2)^2;
          
        
        rad = sailLength/2;
        step = pi/20;
        cir = 0: step: 2.0*pi;


        bx = [rad*cos(cir)];
        by = [rad*sin(cir)];
        bw = ones(1,length(bx));
        circle = [bx; by; bw];
        poiPose = Pose(poi(1), poi(2), 0);
        point_circle = poiPose.bToA()*circle;
        world_circle = init_pose.bToA()*point_circle;
        plot(world_circle(1,:),world_circle(2,:),'y');
        
        
        x_I = x(sailIndices);
        y_I = y(sailIndices);
        if(length(sailIndices) < 4) %9
            continue;
        end    

        x_mean = mean(x_I);
        y_mean = mean(y_I);
        x_I = x_I-x_mean;
        y_I = y_I-y_mean;

        Ixx = dot(x_I ,x_I);
        Iyy = dot(y_I, y_I);
        Ixy = - dot(x_I, y_I);
        Inertia = [Ixx Ixy;Ixy Iyy]; 
        lambda = eig(Inertia);
        lambda = sqrt(lambda)*100;
        lambda = sort(lambda);
        if lambda(1) >= 1.3
            continue;
        end

        if abs(lambda(2) - 12.5) >= 5.0 %1.5
            continue;
        end
        
        
             
      
        %disp(lambda(2));
        
        potentialSail = find(sailIndices);
        
        missingRight = 1;
        rightMost = max(potentialSail);
        while(missingRight)
            if(rightMost == 360)
                rightMost = 1;
            else
                rightMost = rightMost+1;
            end
            
            if(x_full(rightMost) ~= 0 || y_full(rightMost) ~= 0)
                potentialSail = [potentialSail rightMost];
                missingRight = 1;
            else
                missingRight = 0;
            end               
        end
        
        missingLeft = 1;
        leftMost = min(potentialSail);
        while(missingLeft)
            if(leftMost == 1)
                leftMost = 360;
            else
                leftMost = leftMost-1;
            end
            
            if(x_full(leftMost) ~= 0 || y_full(leftMost) ~= 0)
                potentialSail = [leftMost potentialSail];
                missingLeft = 1;
            else
                missingLeft = 0;
            end               
        end
        
        
        sailIndices = zeros(1, length(x_full));
        for(q=1:1:length(potentialSail))
            sailIndices(potentialSail(q)) = 1;
        end 
        
        
             
        
        x_I = x(logical(sailIndices));
        y_I = y(logical(sailIndices));
        x_mean = mean(x_I);
        y_mean = mean(y_I);
        x_I = x_I-x_mean;
        y_I = y_I-y_mean;

        Ixx = dot(x_I ,x_I);
        Iyy = dot(y_I, y_I);
        Ixy = - dot(x_I, y_I);
        Inertia = [Ixx Ixy;Ixy Iyy]; 
        lambda = eig(Inertia);
        lambda = sqrt(lambda)*100;
        lambda = sort(lambda);
        %disp(lambda);
        th = (atan2(2*Ixy,Iyy-Ixx)/2.0);
        th_vect = [cos(th); sin(th)];
        pos_vect = [x_mean; y_mean];
        if dot(th_vect,pos_vect) < 0
            th = mod((th+2*pi),2*pi)-pi;
        end
        %disp(th);
        
        
        plotPoints = laserPts(:,logical(sailIndices));
        plotPoints_w = ones(1,length(plotPoints(1,:)));
        world_plotPoints = init_pose.bToA()*[plotPoints(1,:); plotPoints(2,:); plotPoints_w];

        scatter(world_plotPoints(1,:), world_plotPoints(2,:),'b', 'filled');
        
        
        centroidPts = [x_mean; y_mean; 1];
        world_centroidPts = init_pose.bToA()*centroidPts;
        scatter(world_centroidPts(1,:),world_centroidPts(2,:),'k', 'filled');
                 
        centroid = Pose(x_mean, y_mean, th);
                  
        
        laserPts(:,logical(sailIndices)) = 0;

        plot(world_circle(1,:),world_circle(2,:),'k');
        found = 1;
        plotG = 1;
        break;

    end
    
    
end
