%% Lab 1 Task 1 Move the Robot
clear all;
robot = raspbot();
tStart = tic;
while(toc(tStart) <= 4)
    robot.sendVelocity(.05, .05)
    pause(.005)
end
tStart = tic;
pause(2)
while(toc(tStart) <= 4)
    robot.sendVelocity(-.05, -.05)
    pause(.005)
end

%% Lab 1 Task 2 Basic Simulation
clear all;
% constants
velocity = .05; % in m
leftStart = 6934;
rightStart = 4396;
% sim variables
global signedDistance;
signedDistance = 0; % in m
global leftEncoder;
leftEncoder = leftStart;
global rightEncoder;
rightEncoder = rightStart;
tStart = tic;
while(signedDistance < .2)
    i = toc(tStart)*velocity;
    tStart = tic;
    leftEncoder = leftEncoder + i;
    rightEncoder = rightEncoder + i;
    signedDistance = ((leftEncoder - leftStart) + (rightEncoder - rightStart))/2;
    disp(signedDistance);
    pause(.005)
end
disp(signedDistance);

velocity = -.05; % in m
signedDistance = 0; % in m
leftEncoder = leftStart;
rightEncoder = rightStart;
tStart = tic;
while(signedDistance > -.2)
    i = toc(tStart)* velocity;
    leftEncoder = leftEncoder + i;
    rightEncoder = rightEncoder + i;
    signedDistance = ((leftEncoder - leftStart) + (rightEncoder - rightStart))/2;
    pause(.005)
end
disp(signedDistance);

%% Lab 1 Task 5 Basic Plotting and Real Time Plotting
clear all;
global timeArray;
timeArray = zeros(1,1);
global leftArray;
leftArray = zeros(1,1);
global rightArray;
rightArray = zeros(1,1);

% constants
velocity = .05; % in m
leftStart = .6934;
rightStart = .4396;
% sim variables
global signedDistance;
signedDistance = 0; % in m
global leftEncoder;
leftEncoder = leftStart;
global rightEncoder;
rightEncoder = rightStart;
global loop;
loop = 1;
global tStart;
tStart = tic;
while(signedDistance <= .2)
    loop = loop + 1;
    
    time = toc(tStart);
    tStart = tic;
    timeArray(loop) = timeArray(loop-1) + time;
    
    i = time*velocity;

    leftEncoder = leftEncoder + i;
    leftDist = leftEncoder - leftStart;
    leftArray(loop) = leftDist;
        
    rightEncoder = rightEncoder + i;
    rightDist = rightEncoder - rightStart;
    rightArray(loop) = rightDist;
    
    signedDistance = (leftDist + rightDist)/2;
    
    plot(timeArray,leftArray,timeArray,rightArray);
    pause(.005);
end
plot(timeArray,leftArray,timeArray,rightArray);

pause(2);

% constants
velocity = -.05; % in m
% sim variables
signedDistance = 0; % in m
loop = 1;
tStart = tic;
while(signedDistance >= -.3048)
    loop = loop + 1;
    
    time = toc(tStart);
    tStart = tic;
    timeArray(loop) = timeArray(loop-1) + time;
    
    i = time*velocity;
    
    leftEncoder = leftEncoder + i;
    leftDist = leftEncoder - leftStart;
    leftArray(loop) = leftDist;
        
    rightEncoder = rightEncoder + i;
    rightDist = rightEncoder - rightStart;
    rightArray(loop) = rightDist;
    
    signedDistance = (leftDist + rightDist)/2;
    
    plot(timeArray,leftArray,timeArray,rightArray);
    pause(.005);
end
plot(timeArray,leftArray,timeArray,rightArray);




%% Lab 1 Challenge Task
% constants
global velocity;
velocity = .05; % in m
global leftStart;
leftStart = double(robot.encoders.LatestMessage.Data(1))/1000;
global rightStart;
rightStart = double(robot.encoders.LatestMessage.Data(2))/1000;

% variables
global timeArray;
timeArray = zeros(1,1);
global leftArray;
leftArray = zeros(1,1);
global rightArray;
rightArray = zeros(1,1);

global signedDistance;
signedDistance = 0; % in m
global loop;
loop = 1;
global tStart;
tStart = tic;

robot.sendVelocity(velocity, velocity);
while(signedDistance < .2)
    loop = loop + 1;
    
    time = toc(tStart);
    tStart = tic;
    timeArray(loop) = timeArray(loop-1) + time;
    
    leftEncoder = double(robot.encoders.LatestMessage.Data(1))/1000;
    leftDist = leftEncoder - leftStart;
    leftArray(loop) = leftDist;
    
    rightEncoder = double(robot.encoders.LatestMessage.Data(2))/1000;
    rightDist = rightEncoder - rightStart;
    rightArray(loop) = rightDist;
    
    signedDistance = (leftDist + rightDist)/2;
    
    plot(timeArray,leftArray,timeArray,rightArray);
    robot.sendVelocity(velocity, velocity);
    pause(.005);
end
robot.sendVelocity(0, 0);
plot(timeArray,leftArray,timeArray,rightArray);

pause(2);

leftStart = double(robot.encoders.LatestMessage.Data(1))/1000;
rightStart = double(robot.encoders.LatestMessage.Data(2))/1000;
velocity = -.05; % in m
signedDistance = 0; % in m
loop = 1;
tStart = tic;

robot.sendVelocity(velocity, velocity);
while(signedDistance > -0.3048)
    loop = loop + 1;
    
    time = toc(tStart);
    tStart = tic;
    timeArray(loop) = timeArray(loop-1) + time;
    
    leftEncoder = double(robot.encoders.LatestMessage.Data(1))/1000;
    leftDist = leftEncoder - leftStart
    leftArray(loop) = leftDist;
    
    rightEncoder = double(robot.encoders.LatestMessage.Data(2))/1000;
    rightDist = rightEncoder - rightStart;
    rightArray(loop) = rightDist;
    
    signedDistance = (leftDist + rightDist)/2;
    
    plot(timeArray,leftArray,timeArray,rightArray);
    robot.sendVelocity(velocity, velocity);
    pause(.005);
end
robot.sendVelocity(0, 0);
plot(timeArray,leftArray,timeArray,rightArray);