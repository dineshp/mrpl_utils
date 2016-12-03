function s_act = moveRelDist(robot,dist)
% Same from lab 4 - the one where we moved in a traight line with feedback
%PROBLEMS - listener callback did not work 100% of the time not sure what
%the issue was. - in the velocity profile integrator, it keeps giving me
%the error on line 23 when using &&. pretty sure its being used correctly

% Need to fix the the vmax and amax to account for the smalle distances we
% will be needing
% Couldnt check if the backwards worked well 

k_p = 3;
k_d = .01;
k_i = 0;
v_max = .15;
a_max = 3*.15;

dir = 1;
if (dist < 0)
    dir = -1;
end

s_f = dist;
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
     
    u_ref = trapezoidalVelocityProfile(t, a_max, v_max, s_f, dir);
    s_del = trapezoidalVelocityProfileIntegrator(t-t_del, a_max, v_max, s_f, dir);   
    
    
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
    robot.sendVelocity(-U, -U);
end



end
