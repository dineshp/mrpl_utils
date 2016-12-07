function vel = trapezoidalTurnProfile(t , alphaMax, omegaMax, theta)
%returns the velcity of the wheels. 
%Wheel base W is definitely not correct but makes the robot turn the correct amount
%sooooo yea


    W = .08943;
    theta_f = theta;
    t_ramp = omegaMax/alphaMax;
    t_f = ((theta_f + ((omegaMax)^2/alphaMax))/omegaMax);
    
    if(t < 0)
        vel = 0;
        return;
    end

    if(t >= t_f)
        vel = 0;
        return;
    end
    
    if(t < t_ramp)
        omegaRef = alphaMax*t;
        vel = omegaRef*W;
    elseif(t > t_ramp && (t < (t_f - t_ramp))) 
        omegaRef = omegaMax;
         vel = omegaRef*W;
    elseif((t_f - t) < t_ramp)
        omegaRef = (alphaMax)*(t_f - t);
         vel = omegaRef*W;
    else
         vel = 0;
    end
    
end
