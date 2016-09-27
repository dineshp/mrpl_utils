function ds = trapezoidalVelocityProfileIntegrator( t , a_max, v_max, dist, sgn)

    s_f = dist;
    t_ramp = v_max/a_max;
    t_f = ((s_f + ((v_max)^2/a_max))/v_max);

    if(t < 0)
        ds = 0;
        return;
    end
    
    fun1 = @(x) a_max*x; 
    fun2 = @(x) v_max + 0*x;
    fun3 = @(x) (a_max)*(t_f - x); 
    
    if(t >= t_f)
        ds = integral(fun1, 0, t_ramp) + integral(fun2, t_ramp, (t_f-t_ramp)) + integral(fun3, (t_f-t_ramp), t_f);
        return;
    end
    
    if(t < t_ramp)
        ds = integral(fun1, 0, t);
    elseif(t > t_ramp && t < (t_f - t_ramp))
        ds = integral(fun1, 0, t_ramp) + integral(fun2, t_ramp, t); 
    elseif((t_f - t) < t_ramp)
        ds = integral(fun1, 0, t_ramp) + integral(fun2, t_ramp, (t_f-t_ramp)) + integral(fun3, (t_f-t_ramp), t); 
    else
        ds = integral(fun1, 0, t_ramp) + integral(fun2, t_ramp, (t_f-t_ramp)) + integral(fun3, (t_f-t_ramp), t_f); 
    end
    
end