classdef RobotState < handle

    properties(Access = public)
        t;
        V;
        w;
        s;
        x;
        y;
        th;
        x_g_ref;
        y_g_ref;
        th_g_ref;
        err_x_g_ref;
        err_y_g_ref;
        err_th_g_ref;
        i;
    end
    
    methods(Access = public)    
        function obj = RobotState(n)
            obj.i = 1;
            
            obj.t = zeros(1, n);
            obj.V = zeros(1, n);
            obj.w = zeros(1, n);
            obj.s = zeros(1, n);
            obj.x = zeros(1, n);
            obj.y = zeros(1, n);
            obj.th = zeros(1, n);
            obj.x_g_ref = zeros(1,n);
            obj.y_g_ref = zeros(1,n);
            obj.th_g_ref = zeros(1,n);
            obj.err_x_g_ref = zeros(1,n);
            obj.err_y_g_ref = zeros(1,n);
            obj.err_th_g_ref = zeros(1,n);
            
        end
        
       function new = copy(this)
            new = feval(class(this));
            
            p = properties(this);
            for j = 1:length(p)
                new.(p{j}) = this.(p{j});
            end
        end
        
        function iPlusPlus(obj)
            obj.i = obj.i + 1;
        end
    end
    
end
