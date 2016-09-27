function uref = trapezoidalVelocityProfile( t , a_max, v_max, dist, sgn)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative. 
    s_f = dist;
    t_ramp = v_max/a_max;
    t_f = ((s_f + ((v_max)^2/a_max))/v_max);
    
    if(t < 0)
        uref = 0;
        return;
    end

    if(t >= t_f)
        uref = 0;
        return;
    end
    
    if(t < t_ramp)
        uref = a_max*t;
    elseif(t > t_ramp && (t < (t_f - t_ramp))) 
        uref = v_max; 
    elseif((t_f - t) < t_ramp)
        uref = (a_max)*(t_f - t);    
    else
        uref = 0;
    end
    
end