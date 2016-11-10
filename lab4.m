%% Test
format long g
robot.encoders.NewMessageFcn=@encoderEventListener;
robot.sendVelocity(.15, .15);
pause(1);
t = getT;
x = getX;
y = getY;
disp(t);
disp(x);
disp(y);
robot.encoders.NewMessageFcn=[];


%% Lab 4: Simulation, no robot
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

goalDist = 1;
maxV = .3;
k_p = 8;
k_d = .08;

prevX = robot.encoders.LatestMessage.Vector.X;
prevY = robot.encoders.LatestMessage.Vector.Y;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);
dist = 0;
totalTime = 0;
err = goalDist - dist;
V = maxV; %k_p*err;
p_err = err;
dt_cmd = .05;
tStart = tic;
while(abs(err) > .0001 && totalTime < 6)
    if(totalTime == 0)
        robot.sendVelocity(V, V); 
        scatter(totalTime, err*1000);
        pause(dt_cmd);
        dt_act = toc(tStart);
        totalTime = totalTime + dt_act;
    else       
        dt_act = toc(tStart); 
        totalTime = totalTime + dt_act;
        
        count = 0;
        cT = getT;
        while(eq(cT, pT))     
            cT = getT;
            count = count + 1;
            pause(.001);
        end
        curX = getX;
        curY = getY;

        ds_act = mean([(curX - prevX), (curY - prevY)], 'double');
        
        pT = cT;
        prevX = curX;
        prevY = curY;
       
        dist = dist + ds_act; 

        err = goalDist - dist;
        errDer = (err - p_err)/dt_act;
        p_err = err;
        
        V = k_p*err + k_d*errDer;
       
        scatter(totalTime, err*1000);

        if(V > .3)
            V = .3;
        elseif(V < -.3)
            V = -.3;
        end

        robot.sendVelocity(V, V);
        tStart = tic;
        
        pauseC = (dt_cmd-(.001*count));
        pauseC = max(0, pauseC);
        pause(pauseC);
    end
end

robot.encoders.NewMessageFcn=[];

%% Open loop sim
clearvars -except robot;
close all;
format long g;
figure;
hold on;
axis([0 6 -500 1000])

v_max = .25;
a_max = 3*.25;

s_f = 1;

t_f = (s_f + ((v_max)^2/a_max))/v_max;
disp(t_f);
t = 0;
U = 0; %a_max*t;

dt_cmd = .05;
tStart = tic;
while(t < (t_f+1))
    dt_act = toc(tStart); %for robot, use time stamp
    t = t + dt_act;
   
    s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);
    disp(s_ref);
    scatter(t, s_ref*1000);
    
    
    U = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);

    scatter(t, U*1000);
    %robot.sendVelocity(V, V);
    tStart = tic;   
    pause(dt_cmd);
end

%% Open loop on robot
clearvars -except robot;
close all;
format long g;
figure;
hold on;
axis([0 6 0 1500])

robot.encoders.NewMessageFcn=@encoderEventListener;

v_max = .25;
a_max = 3*.25;

s_f = 1;
t_f = (s_f + ((v_max)^2/a_max))/v_max;

s_act = 0;
t = 0;
U = 0; %a_max*t;


prevX = robot.encoders.LatestMessage.Vector.X;
prevY = robot.encoders.LatestMessage.Vector.Y;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);

dt_cmd = .05;
tStart = tic;
while(t <= t_f)
    dt_act = toc(tStart); 
    tStart = tic;
    U = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);
    robot.sendVelocity(U, U);
    s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);
    scatter(t, s_ref*1000);
    
    cT = getT;
    while(eq(cT, pT))     
        cT = getT;
        count = count + 1;
        pause(.001);
    end
    curX = getX;
    curY = getY;
    
    s_act = s_act + mean([(curX - prevX), (curY - prevY)], 'double');
    pT = cT;
    prevX = curX;
    prevY = curY;
    scatter(t, s_act*1000);
       
    t = t + dt_act;
    pause(dt_cmd);
end
robot.stop;
robot.encoders.NewMessageFcn=[];

%% Open loop on robot w/ delay
clearvars -except robot;
close all;
format long g;
figure(1);
hold on;
axis([0 6 0 1500])
figure(2);
hold on;
%axis([0 6 -1 2])

hold on;

robot.encoders.NewMessageFcn=@encoderEventListener;

v_max = .25;
a_max = 3*.25;
t_del = .4;

s_f = 1;
t_f = (s_f + ((v_max)^2/a_max))/v_max;

s_act = 0;
t = 0;
U = 0; %a_max*t;


prevX = robot.encoders.LatestMessage.Vector.X;
prevY = robot.encoders.LatestMessage.Vector.Y;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);

dt_cmd = .05;
tStart = tic;
while(t < t_f)
    dt_act = toc(tStart); 
    tStart = tic;
    U = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1);
    robot.sendVelocity(U, U);
    
    s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);
    s_del = trapezoidalVelocityProfileIntegrator(t-t_del, a_max, v_max, s_f, 1);
    
    cT = getT;
    while(eq(cT, pT))     
        cT = getT;
        count = count + 1;
        pause(.001);
    end
    curX = getX;
    curY = getY;
    
    s_act = s_act + mean([(curX - prevX), (curY - prevY)], 'double');
    pT = cT;
    prevX = curX;
    prevY = curY;
    figure(1);
    scatter(t, s_del*1000);
    scatter(t, s_act*1000);
    s_diff = s_del-s_act;
    figure(2);
    scatter(t, s_diff*1000);
       
    t = t + dt_act;
    pause(dt_cmd);
end
robot.stop;
robot.encoders.NewMessageFcn=[];

%% Two degrees of freedom on Robot
clearvars -except robot;
close all;
format long g;
figure(1);
hold on;
axis([0 6 0 1500])
figure(2);
hold on;
%axis([0 6 -1 2])

hold on;

robot.encoders.NewMessageFcn=@encoderEventListener;

k_p = 4;
k_d = .04;
k_i = 4;
err = 0;
p_err = 0;
errorIntegral = 0;

v_max = .25;
a_max = 2*.25;
t_del = .4;

s_f = 1.03;
t_f = (s_f + ((v_max)^2/a_max))/v_max;

s_act = 0;
t = 0;
U = 0; %a_max*t;

prevX = robot.encoders.LatestMessage.Vector.X;
prevY = robot.encoders.LatestMessage.Vector.Y;
pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9);
dt_cmd = .05;
tStart = tic;
while(t < (t_f + t_del + 1))
    if(t == 0)
        pause(t_del);
        dt_act = toc(tStart);
        s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);
        s_del = trapezoidalVelocityProfileIntegrator(t-t_del, a_max, v_max, s_f, 1);
        figure(1);
        scatter(t, s_del*1000);

        count = 0;
        cT = getT;
        while(eq(cT, pT))     
            cT = getT;
            count = count + 1;
            pause(.001);
        end
        curX = getX;
        curY = getY;

        s_act = s_act + mean([(curX - prevX), (curY - prevY)], 'double');
        pT = cT;
        prevX = curX;
        prevY = curY;
        figure(1);
        scatter(t, s_act*1000);
        s_diff = s_del-s_act;
        figure(2);
        scatter(t, s_diff*1000);

        err = s_del - s_act;
        errDer = (err - p_err)/dt_act;
        errorIntegral = errorIntegral + err*dt_act;
        p_err = err;
        U = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1) + k_p*err + k_d*errDer + k_i*errorIntegral;

        if(U > .5)
            U = .5;
        elseif(U < -.5)
            U = -.5;
        end

        robot.sendVelocity(U, U);
        tStart = tic;
        t = t + dt_act;
    else
        dt_act = toc(tStart); 

        s_ref = trapezoidalVelocityProfileIntegrator(t, a_max, v_max, s_f, 1);
        s_del = trapezoidalVelocityProfileIntegrator(t-t_del, a_max, v_max, s_f, 1);
        figure(1);
        scatter(t, s_del*1000);

        count = 0;
        cT = getT;
        while(eq(cT, pT))     
            cT = getT;
            count = count + 1;
            pause(.001);
        end
        curX = getX;
        curY = getY;

        s_act = s_act + mean([(curX - prevX), (curY - prevY)], 'double');
        pT = cT;
        prevX = curX;
        prevY = curY;
        figure(1);
        scatter(t, s_act*1000);
        s_diff = s_del-s_act;
        figure(2);
        scatter(t, s_diff*1000);

        err = s_del - s_act;
        errDer = (err - p_err)/dt_act;
        errorIntegral = errorIntegral + err*dt_act;
        p_err = err;
        U = trapezoidalVelocityProfile(t, a_max, v_max, s_f, 1) + k_p*err + k_d*errDer + k_i*errorIntegral;


        if(U > .5)
            U = .5;
        elseif(U < -.5)
            U = -.5;
        end

        robot.sendVelocity(U, U);
        tStart = tic;
        t = t + dt_act;
        pauseC = (dt_cmd-(.001*count));
        pauseC = max(0, pauseC);
        pause(pauseC);
    end
end
robot.stop;
robot.encoders.NewMessageFcn=[];