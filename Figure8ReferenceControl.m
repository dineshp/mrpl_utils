classdef Figure8ReferenceControl < handle

    properties(Constant)
        k_k = 15.1084;
        v = .2;
        v_max = Figure8ReferenceControl.v;
        a_max = 2*.2;
        s_f = 1.0;
        k_th = 2*pi/Figure8ReferenceControl.s_f;
        t_f = ((Figure8ReferenceControl.s_f + ((Figure8ReferenceControl.v_max)^2/Figure8ReferenceControl.a_max))/Figure8ReferenceControl.v_max);
    end
    
    properties(Access = public)
        tPause;
        k_s;
        k_v;
        T_f; %scaled final time
    end

    properties(Access = private)

    end

    methods(Access = private)
        function uT = getUnscaledTime(obj, time)
            uT = time*(obj.k_v/obj.k_s);
        end
    end

    methods(Access = public)       

        function obj = Figure8ReferenceControl(k_s,k_v,tPause)
            obj.k_s = k_s;
            obj.k_v = k_v;
            obj.tPause = tPause;
            obj.T_f = (obj.k_s/obj.k_v)*Figure8ReferenceControl.t_f;
        end

        function [V, w] = computeControl(obj,timeNow)
            if(timeNow < 0)
                error('time less than zero');
            end
            
            if(timeNow <= obj.tPause)
                V = 0;
                w = 0;
                return;
            end
            
            if((timeNow > (obj.T_f+obj.tPause)))
                V = 0;
                w = 0;
                return;
            end
            
            t_del = timeNow - obj.tPause;
            t_uns = obj.getUnscaledTime(t_del);
            s_t = trapezoidalVelocityProfileIntegrator( t_uns , Figure8ReferenceControl.a_max, Figure8ReferenceControl.v_max, Figure8ReferenceControl.s_f, 1);
            Kurv = (Figure8ReferenceControl.k_k/obj.k_s)*sin(Figure8ReferenceControl.k_th*s_t);
            
            V = obj.k_v*obj.getTrapV(t_uns);
            w = Kurv*V; 
        end
        
        function r = getTrapV(obj, t)
            r = trapezoidalVelocityProfile(t, Figure8ReferenceControl.a_max, Figure8ReferenceControl.v_max, Figure8ReferenceControl.s_f, 1);
        end
        
        function duration = getTrajectoryDuration(obj)
            duration = obj.T_f + (2*obj.tPause);
        end    
    end

end

