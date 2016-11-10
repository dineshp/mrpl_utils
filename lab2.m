%% Display Lidar Data
close all;
figure(1); clf;
hold on;
axis([-1.1 1.1 -1.1 1.1]);
while(1)
    x = zeros(1,1);
    y = zeros(1,1);
    scan = robot.laser.LatestMessage.Ranges;
    for n=1:1:360
        if(abs(scan(n)) <= 1 && abs(scan(n)) > .06)
            x(n) = scan(n)*cos((n-1)*pi/180);
            y(n) = scan(n)*sin((n-1)*pi/180);
        else
           x(n) = 0;
           y(n) = 0;
        end
    end
    % plot (-y,x) instead of (x,y) to rotate 90 degrees counterclockwise
    %plot(robot.laser.LatestMessage.Ranges);
    scatter(-y, x);
    pause(1);
end

%% Lab 2 Task 1 Read and Display Lidar Data
figure(1); clf;
xA = zeros(1,1);
yA = zeros(1,1);
while(1)
    scan = robot.laser.LatestMessage.Ranges;
    for n=1:1:360
       if(abs(scan(n)) < 1)
           [x, y, th] = RangeImage.irToXy(n, scan(n));
           xA = cat(1, xA, x);
           yA = cat(1, yA, y);
       end
    end
    disp(xA);
    scatter(-yA, xA, 'x');
    pause(.2);
end

%% Lab 2 Task 2 Plot The Position of The Nearest Object 
figure(1); clf;
%xA = zeros(1,1);
%yA = zeros(1,1);
while(1)
    maxDist = 1.5;
    closestN = 0;
    xA = zeros(1,1);
    yA = zeros(1,1);
    scan = robot.laser.LatestMessage.Ranges;
    for n=1:1:360
       if(abs(scan(n)) > .06 && abs(scan(n)) < 1.5)
           [x, y, th] = RangeImage.irToXy(n, scan(n));
           if(abs(th) < pi/2)
               if (scan(n) < maxDist)
                   maxDist = scan(n);
                   closestN = n;
               end
               %xA = cat(1, xA, x);
               %yA = cat(1, yA, y);
           end
       end
    end
    
    
    objectIndex = zeros(1,1);
    objectIndex(1) = mod(closestN-2, 360)+1;
    [x, y, th] = RangeImage.irToXy(objectIndex(1), scan(objectIndex(1)));
    xA = cat(1, xA, x);
    yA = cat(1, yA, y);
    objectIndex(2) = closestN; 
    [x, y, th] = RangeImage.irToXy(objectIndex(2), scan(objectIndex(2)));
    xA = cat(1, xA, x);
    yA = cat(1, yA, y);
    objectIndex(3) = mod(closestN+1, 360);
    [x, y, th] = RangeImage.irToXy(objectIndex(3), scan(objectIndex(3)));
    xA = cat(1, xA, x);
    yA = cat(1, yA, y);
    
    
    
    

%     i = 1; j = -1;
%     while(closestN+i<360 && (scan(closestN+i) ~=0))
%        objectIndex(i+1) = closestN+i;
%        i = i+1 
%     end
%     
%     while(closestN+j>1 && scan(closestN+j) ~= 0)
%        objectIndex(i+1) = closestN+j;
%        j = j-1;
%        i = i+1;
%     end 
%        
%      for i=1:length(objectIndex)
%          [x, y, th] = RangeImage.irToXy(objectIndex(i), scan(objectIndex(i)));
%          xA = cat(1, xA, x);
%          yA = cat(1, yA, y);
%      end

   
%    disp(closestN);
     scatter(-yA, xA, 'x');
     axis([-1 1 -1 1]);
    pause(.2);
end

%% Lab 2 Task 3 Distance Serve
k = .2;
maxV = .15;
idealDist = 1;
while(1)
    maxDist = 1.5;
    closestN = 0;
    xA = zeros(1,1);
    yA = zeros(1,1);
    scan = robot.laser.LatestMessage.Ranges;
    for n=1:1:360
       if(abs(scan(n)) > .06 && abs(scan(n)) < 3)
           [x, y, th] = RangeImage.irToXy(n, scan(n));
           if(abs(th) < pi/2)
               if (scan(n) < maxDist)
                   maxDist = scan(n);
                   closestN = n;
               end
           end
       end
    end

    objectIndex = zeros(1,1);
    objectIndex(1) = mod(closestN-2, 360)+1;
    objectIndex(2) = closestN; 
    objectIndex(3) = mod(closestN, 360)+1;
    disp(objectIndex);
    dist = mean([scan(objectIndex(1)), scan(objectIndex(2)), scan(objectIndex(3))], 'double');
    disp(dist);
    
    v = k*(dist - idealDist);
    
    if(v > maxV)
        v = maxV;
    end
    robot.sendVelocity(v, v);
    
%     [x, y, th] = RangeImage.irToXy(objectIndex(1), scan(objectIndex(1)));
%     xA = cat(1, xA, x);
%     yA = cat(1, yA, y);
%     [x, y, th] = RangeImage.irToXy(objectIndex(2), scan(objectIndex(2)));
%     xA = cat(1, xA, x);
%     yA = cat(1, yA, y);
%     [x, y, th] = RangeImage.irToXy(objectIndex(3), scan(objectIndex(3)));
%     xA = cat(1, xA, x);
%     yA = cat(1, yA, y);
%   
%      scatter(-yA, xA, 'x');
%      axis([-1 1 -1 1]);
    pause(.1);
end

%% Lab 2 Challenge Task w Plotting
k = .5;
maxV = .15;
idealDist = 1;
loopCount = 0;
while(1)
    maxDist = 1.5;
    closestN = 0;
    xA = zeros(1,1);
    yA = zeros(1,1);
    if(mod(loopCount, 5) == 0)
        scan = robot.laser.LatestMessage.Ranges;
    end
    for n=1:1:360
       if(abs(scan(n)) > .06 && abs(scan(n)) < 1.5)
           [x, y, th] = RangeImage.irToXy(n, scan(n));
           if(abs(th) < pi/2)
               if (scan(n) < maxDist)
                   maxDist = scan(n);
                   closestN = n;
               end
           end
       end
    end

    %objectIndex = zeros(1,1);
    %objectIndex(1) = mod(closestN-2, 360)+1;
    %objectIndex(2) = closestN; 
    %objectIndex(3) = mod(closestN, 360)+1;
    %disp(objectIndex);
    %dist = mean([scan(objectIndex(1)), scan(objectIndex(2)), scan(objectIndex(3))], 'double');
    if(closestN == 0)
        loopCount = loopCount + 1;
        continue;
    end
    
    dist = scan(closestN);
    %disp(dist);
    
    [x, y, th] = RangeImage.irToXy(closestN, scan(closestN));
    
    Kurv = th/abs(dist - idealDist); 
    v = k*(dist - idealDist);
    
    if(v > maxV)
        v = maxV;
    end
    if(v < (-1)*maxV)
        v = (-1)*maxV;
    end
    w = Kurv*v;
    wheelbase = .082;
    if(v < 0)
        vl = v;
        vr = v;
    else
        vl = v - (wheelbase/2)*w;
        vr = v + (wheelbase/2)*w;
    end
     
    robot.sendVelocity(vl, vr);
    %disp(vl);
    %disp(vr);
    
%     [x, y, th] = RangeImage.irToXy(objectIndex(1), scan(objectIndex(1)));
%     xA = cat(1, xA, x);
%     yA = cat(1, yA, y);
%     [x, y, th] = RangeImage.irToXy(objectIndex(2), scan(objectIndex(2)));
%     xA = cat(1, xA, x);
%     yA = cat(1, yA, y);
%     [x, y, th] = RangeImage.irToXy(objectIndex(3), scan(objectIndex(3)));
%     xA = cat(1, xA, x);
%     yA = cat(1, yA, y);
    
    scatter(-y, x, 'x');
    axis([-2 2 -2 2]);
    pause(.1);
    
    loopCount = loopCount + 1;
end