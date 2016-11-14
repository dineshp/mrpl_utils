%% Lab 4: PD Simulation, no robot
clearvars -except robot;
close all;
format long g;
figure;
hold on;
axis([0 6 -10 100])

s_f = .1;
maxU = .3;
k_p = 25;
k_d = 0;
k_i = 0;

s_act = 0;
t = 0;
err = s_f - s_act;
U = maxU; %k_p*err;
dt_cmd = .05;

%robot.sendVelocity(V, V); 
tStart = tic;
scatter(t, err*1000);
pause(dt_cmd);
while(abs(err) > .0001 && t < 6)
    dt_act = toc(tStart);
    ds = U*dt_act;
    t = t + dt_act;
    s_act = s_act + ds;
    err = s_f - s_act;
    U = k_p*err + k_d*err + k_i*err;
    if(U > .3)
        U = .3;
    elseif(U < -.3)
        U = -.3;
    end
    %robot.sendVelocity(V, V); 
    tStart = tic;
    scatter(t, err*1000);
    pause(dt_cmd);
end

%% PD control for the Robot
clearvars -except robot;
close all;
format long g;

robot.encoders.NewMessageFcn=@encoderEventListener;

figure;
hold on;
axis([0 6 -30 100])
s_f = .1;
maxU = .3;
k_p = 8;
k_d = .6;
k_i = 0;

s_act = 0;
t = 0;
err = s_f - s_act;
p_err = 0;
errInt = 0;
U = maxU; %k_p*err;
dt_cmd = .05;


prevX = robot.encoders.LatestMessage.Vector.X;
prevY = robot.encoders.LatestMessage.Vector.Y;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);
cT = getT;
curX = getX;
curY = getY;

robot.sendVelocity(U, U); 
scatter(t, err*1000);
pause(dt_cmd);
while(abs(err) > .0001 && t < 6) 
    cT = getT;
    curX = getX;
    curY = getY;
    while(eq(cT, pT))     
        cT = getT;
        curX = getX;
        curY = getY;
        pause(.001);
    end

    ds_act = mean([(curX - prevX), (curY - prevY)], 'double');
    dt_act = cT - pT;

    prevX = curX;
    prevY = curY;
    pT = cT;

    s_act = s_act + ds_act; 
    t = t + dt_act;
    
    err = s_f - s_act;
    errDer = (err - p_err)/dt_act;
    disp(errDer);
    p_err = err;
    
    U = k_p*err + k_d*errDer + k_i*errInt;
    if(U > .3)
        U = .3;
    elseif(U < -.3)
        U = -.3;
    end
    robot.sendVelocity(U, U); 
    scatter(t, err*1000);
    pause(dt_cmd);
end

robot.encoders.NewMessageFcn=[];

%% Plot Reference Velocity and its Integral in Simulation
clearvars -except robot;
close all;
format long g;
figure;
hold on;
axis([0 6 0 1000])

v_max = .25;
a_max = 3*.25;

s_f = 1;

t_f = (s_f + ((v_max)^2/a_max))/v_max;
t = 0;

dt_cmd = .05;


s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);   
u_ref = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);

%robot.sendVelocity(U, U);
tStart = tic;   
scatter(t, s_ref*1000);
scatter(t, u_ref*1000);
pause(dt_cmd);
while(t <= t_f)
    dt_act = toc(tStart); %for robot, use time stamp
    t = t + dt_act;
   
    s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);   
    u_ref = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);
    
    %robot.sendVelocity(U, U);
    tStart = tic;   
    scatter(t, s_ref*1000);
    scatter(t, u_ref*1000);
    pause(dt_cmd);
end

%% Plot Reference Velocity and Actual Distance on the Robot
clearvars -except robot;
close all;
format long g;
figure;
hold on;
axis([0 6 0 1000])

robot.encoders.NewMessageFcn=@encoderEventListener;

v_max = .25;
a_max = 3*.25;

s_f = 1;

t_f = (s_f + ((v_max)^2/a_max))/v_max;
t = 0;
s_act = 0;

dt_cmd = .05;


s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);   
u_ref = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);

prevX = robot.encoders.LatestMessage.Vector.X;
prevY = robot.encoders.LatestMessage.Vector.Y;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);
cT = getT;
curX = getX;
curY = getY;

robot.sendVelocity(u_ref, u_ref); 
scatter(t, s_ref*1000);
scatter(t, s_act*1000);
pause(dt_cmd);
while(t <= t_f)
    cT = getT;
    curX = getX;
    curY = getY;
    while(eq(cT, pT))     
        cT = getT;
        curX = getX;
        curY = getY;
        pause(.001);
    end

    ds_act = mean([(curX - prevX), (curY - prevY)], 'double');
    dt_act = cT - pT;

    prevX = curX;
    prevY = curY;
    pT = cT;

    s_act = s_act + ds_act; 
    t = t + dt_act;
   
    s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);   
    u_ref = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);
    
    robot.sendVelocity(u_ref, u_ref);
    scatter(t, s_ref*1000);
    scatter(t, s_act*1000);
    pause(dt_cmd);
end

robot.encoders.NewMessageFcn=[];

%% Identify the delay between Velocity Profile and what the Robot is actually doing
clearvars -except robot;
close all;
format long g;
figure(1);
hold on;
axis([0 6 0 1000]);
figure(2);
hold on;

robot.encoders.NewMessageFcn=@encoderEventListener;

v_max = .25;
a_max = 3*.25;

s_f = 1;
t_del = .16;
t_f = (s_f + ((v_max)^2/a_max))/v_max;
t_ter = (t_f+t_del+1);
t = 0;
s_act = 0;

dt_cmd = .05;


s_del = 0; 
u_ref = 0;
prevX = robot.encoders.LatestMessage.Vector.X;
prevY = robot.encoders.LatestMessage.Vector.Y;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);
cT = getT;
curX = getX;
curY = getY;

while(t <= t_ter)
    cT = getT;
    curX = getX;
    curY = getY;
    count = 0;
    while(eq(cT, pT))     
        count = count + 1;
        cT = getT;
        curX = getX;
        curY = getY;
        pause(.001);
    end

    ds_act = mean([(curX - prevX), (curY - prevY)], 'double');
    dt_act = cT - pT;

    prevX = curX;
    prevY = curY;
    pT = cT;

    s_act = s_act + ds_act; 
    t = t + dt_act;
   
    s_del = trapezoidalVelocityProfileIntegrator(t-t_del, a_max, v_max, s_f, 1);   
    u_ref = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);
    
    robot.sendVelocity(u_ref, u_ref);
    figure(1);
    scatter(t, s_del*1000);
    scatter(t, s_act*1000);
    figure(2);
    scatter(t, (s_del-s_act)*1000);
    pauseC = (dt_cmd-(.001*count));
    pauseC = max(0, pauseC);
    pause(pauseC);
end

robot.encoders.NewMessageFcn=[];

%% Challenge task
clearvars -except robot;
close all;
format long g;
robot.encoders.NewMessageFcn=@encoderEventListener;
clearvars -except robot;
pause(1);
figure(1);
hold on;
axis([0 6 0 1100]);
figure(2);
axis([0 6 -10 10]);
hold on;

k_p = 3;
k_d = .01;
k_i = 0;
v_max = .25;
a_max = 3*.25;

s_f = .15;
t_del = .3506;
t_f = (s_f + ((v_max)^2/a_max))/v_max;
t_ter = (t_f+t_del+1);
t = 0;
s_act = 0;

dt_cmd = .05;

err = 0;
errInt = 0;
p_err = 0;

s_del = 0; 
u_ref = 0;
prevX = getX;
prevY = getY;
pT = getT;
cT = pT;
curX = prevX;
curY = prevY;
feedback = 0;
while(t <= t_ter)
    count = 0;
    while(eq(cT, pT))     
        count = count + 1;
        cT = getT;
        curX = getX;
        curY = getY;
        pause(.001);
    end

    ds_act = mean([(curX - prevX), (curY - prevY)], 'double');
    dt_act = cT - pT;
    prevX = curX;
    prevY = curY;
    pT = cT;

    s_act = s_act + ds_act; 
    t = t + dt_act;
        
    s_del = trapezoidalVelocityProfileIntegrator(t-t_del, a_max, v_max, s_f, 1);   
    u_ref = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);
    
    err = s_del - s_act;
    errDer = (err - p_err)/dt_act;
    p_err = err;
    u_pid = k_p*err + k_d*errDer + k_i*errInt;
    
    U = u_ref + feedback*(u_pid);
    if(U > .25)
        U = .25;
    elseif(U < -.25)
        U = -.25;
    end
    robot.sendVelocity(U, U);
    figure(1);
    scatter(t, s_del*1000, 'k');
    scatter(t, s_act*1000);
    figure(2);
    scatter(t, (s_del-s_act)*1000);
    pauseC = (dt_cmd-(.001*count));
    pauseC = max(0, pauseC);
    pause(pauseC);
end
robot.stop();
robot.encoders.NewMessageFcn=[];
clearvars -except robot;