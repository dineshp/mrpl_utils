function turnRelAngle(robot,angle)
% make sure the velocity is such that the distance will take at
% least a second
% Bunch of hard code, finished a 180 degree turn in under 2 secs
% Pretty accurate to 180 degrees.
% No FeedBack
angle = angle/2.0*pi/180;

velMax = min(abs(angle)*180/pi*2, 180);

velReducer = 6 - (2/85*abs(velMax)-4/17);

if abs(angle) < 10*pi/180
    velReducer = 6;
end

omegaMax = 1.57/velReducer;
alphaMax = 3*1.57/velReducer;

turn = -1;
if angle < 0
    angle = abs(angle);
    turn = 1;
end
theta = angle;
t = 0;
t_f = (angle + (omegaMax^2)/alphaMax)/omegaMax;
dt = TrajectoryFollower.UpdatePause;

vel = trapezoidalTurnProfile( t , alphaMax, omegaMax, theta);
vl = turn * vel;
vr = -turn * vel;
robot.sendVelocity(vl, vr);
tStart = tic;   
while(t <= t_f)
    dt_act = toc(tStart); %for robot, use time stamp
    t = t + dt_act;
   
    vel = trapezoidalTurnProfile(t , alphaMax, omegaMax, theta);
    vl = turn * vel;
    vr = -turn * vel;
    robot.sendVelocity(vl, vr);
    
    tStart = tic;   
    pause(dt);
end
robot.stop();

end