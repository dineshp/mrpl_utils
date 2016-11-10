%% Lab 3 Warm Up Exercise 1: The Robot (Whisperer) Listener

robot.encoders.NewMessageFcn=@encoderEventListener;
r = getT;
disp(r);
%robot.laser.NewMessageFcn=@laserEventListener;
robot.sendVelocity(.15, .15);
pause(1);
r = getT;
disp(size(r));
robot.encoders.NewMessageFcn=[];
%robot.laser.NewMessageFcn=[];

%% Lab 3 Warm Up Exercise 2, Plot Velocity v Time: Case 1 
% Read the encoder as fast as you can. Use your laptop clock to compute dt.
clearvars -except robot
format long g

global velocity;
velocity = .15; % in m
global pLeftEncoder;
pLeftEncoder = robot.encoders.LatestMessage.Vector.X;
global pRightEncoder;
pRightEncoder = robot.encoders.LatestMessage.Vector.Y;

% variables
global timeArray;
timeArray = zeros(1,1000);
global velArray;
velArray = zeros(1,1000);

global loop;
loop = 1;
global tStart;
tStart = tic;

robot.sendVelocity(velocity, velocity);
while(timeArray(loop) < 2)
    loop = loop + 1;
    
    dt = toc(tStart);
    tStart = tic;
    timeArray(loop) = timeArray(loop-1) + dt;
    
    cLeftEncoder = robot.encoders.LatestMessage.Vector.X;
    cRightEncoder = robot.encoders.LatestMessage.Vector.Y;
    ds = mean([(cLeftEncoder - pLeftEncoder), (cRightEncoder - pRightEncoder)], 'double');

    velArray(loop) = ds/dt;
    
    pLeftEncoder = cLeftEncoder;
    pRightEncoder = cRightEncoder;
    
    plot(timeArray(1:loop),velArray(1:loop));
    robot.sendVelocity(velocity, velocity);
    pause(.05);
end
robot.stop();
plot(timeArray(1:loop),velArray(1:loop));

%% Lab 3 Warm Up Exercise 2, Plot Velocity v Time: Case 2
% Respond quickly to Encoder Events. Use the event time tags to compute dt.
clearvars -except robot
format long g
robot.encoders.NewMessageFcn=@encoderEventListener;

global velocity;
velocity = .15; % in m
global pLeftEncoder;
pLeftEncoder = robot.encoders.LatestMessage.Vector.X;
global pRightEncoder;
pRightEncoder = robot.encoders.LatestMessage.Vector.Y;

% variables
global timeArray;
timeArray = zeros(1,1000);
global velArray;
velArray = zeros(1,1000);

global loop;
loop = 1;
global pT;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);

robot.sendVelocity(velocity, velocity);
while(timeArray(loop) < 2)
    loop = loop + 1;
    
    cT = getT;
    while(eq(cT, pT))     
        cT = getT;
        pause(.001);
    end
    dt = cT - pT;
    pT = cT;
    timeArray(loop) = timeArray(loop-1) + dt;
    
    cLeftEncoder = getX;
    cRightEncoder = getY;   
    ds = mean([(cLeftEncoder - pLeftEncoder), (cRightEncoder - pRightEncoder)], 'double');

    velArray(loop) = ds/dt;
    
    pLeftEncoder = cLeftEncoder;
    pRightEncoder = cRightEncoder;
    
    plot(timeArray(1:loop),velArray(1:loop));
    robot.sendVelocity(velocity, velocity);
    pause(.05);
end
robot.stop();
plot(timeArray(1:loop),velArray(1:loop));

robot.encoders.NewMessageFcn=[];


%% Lab 3 Warm Up Exercise 2, Plot Velocity v Time: Case 3
%  Respond quickly to Encoder Events. Use your laptop clock to compute dt.
clearvars -except robot
format long g
robot.encoders.NewMessageFcn=@encoderEventListener;

global velocity;
velocity = .15; % in m
global pLeftEncoder;
pLeftEncoder = robot.encoders.LatestMessage.Vector.X;
global pRightEncoder;
pRightEncoder = robot.encoders.LatestMessage.Vector.Y;

% variables
global timeArray;
timeArray = zeros(1,1000);
global velArray;
velArray = zeros(1,1000);

global loop;
loop = 1;
global tStart;
tStart = tic;
global pT;
pT = getT;

robot.sendVelocity(velocity, velocity);
while(timeArray(loop) < 2)
    loop = loop + 1;
    
    cT = getT;
    while(eq(cT, pT))     
        cT = getT;
        pause(.001);
    end
    dt = toc(tStart);
    tStart = tic;
    timeArray(loop) = timeArray(loop-1) + dt;
    
    cLeftEncoder = getX;
    cRightEncoder = getY;
    ds = mean([(cLeftEncoder - pLeftEncoder), (cRightEncoder - pRightEncoder)], 'double');

    velArray(loop) = ds/dt;
    
    pLeftEncoder = cLeftEncoder;
    pRightEncoder = cRightEncoder;
    
    plot(timeArray(1:loop),velArray(1:loop));
    robot.sendVelocity(velocity, velocity);
    pause(.05);
end
robot.stop();
plot(timeArray(1:loop),velArray(1:loop));

robot.encoders.NewMessageFcn=[];

%% Lab 3 Warm up Excercise 3, Estimate Cornu Spiral
V = .1;
k = .125;
tf = sqrt(32*pi);
dt = .001;
[vl, vr] = RobotModel.getCorSpiralTrajectory(V, k, tf, dt);
[ x, y, th] = RobotModel.modelDiffSteerRobot(vl, vr, 0, tf,dt);
plot(x, y);

%% Lab 3 Warm up Excercise 3, Estimate Figure 8
clearvars -except robot

V = .2;
k_s = 3;
s_f = 1*k_s;
tf = s_f/V;
dt = .001;
[vl, vr] = RobotModel.getFigureEightTrajectory(V, tf, dt);
[ x, y, th] = RobotModel.modelDiffSteerRobot(vl, vr, 0, tf,dt);
plot(x, y);

%% Lab 3 Drive Spiral
clearvars -except robot
format long g
robot.encoders.NewMessageFcn=@encoderEventListener;
V = .1;
k = .125;
tf = sqrt(32*pi);
dt = .06;
[vl, vr] = RobotModel.getCorSpiralTrajectory(V, k, tf, dt);
[ x, y, th] = RobotDriver.driveTrajectory(robot, vl, vr, dt);
%plot(x, y);
axis([0 .4 0 .4])
robot.encoders.NewMessageFcn=[];

figure(2);
[ x_e, y_e, th_e] = RobotModel.modelDiffSteerRobot(vl, vr, 0, tf,dt);
plot(x_e, y_e);
axis([0 .4 0 .4])

%% Lab 3 Drive Figure 8 ###ROBOT 8
clearvars -except robot;
close all;
format long g;

robot.encoders.NewMessageFcn=@encoderEventListener;

V = .2;
k_s = 3;
s_f = 1*k_s;
tf = s_f/V;
lat = 0.00572100749584717;
pause_t = .05;
dt_traj = pause_t + lat;
[vl, vr] = RobotModel.getFigureEightTrajectory(V, tf, dt_traj);

total = 0;
n = 1;
for i=1:1:n
    r = RobotDriver.driveTrajectory(robot, vl, vr, pause_t, dt_traj);
    disp(r);
    total = total + abs(r);
end
if(n > 1)
disp('avg = ');
disp(total/n);
end

robot.encoders.NewMessageFcn=[];


%%
clearvars -except robot;
close all;
format long g;
robot.encoders.NewMessageFcn=@encoderEventListener;
clearvars -except robot;
pause(1);

V = .2;
k_s = 3;
s_f = 1*k_s;
tf = s_f/V;
[vl, vr] = RobotModel.getFigureEightTrajectory(V, tf);
RobotDriver.driveTrajectory2(robot, vl, vr, tf);
robot.encoders.NewMessageFcn=[];
clearvars -except robot;