classdef RobotState < handle

    properties(Access = public)
        t;
        V;
        w;
        s;
        x;
        y;
        th;
        xl;
        yl;
        thl;
        x_g_ref;
        y_g_ref;
        th_g_ref;
        x_g_act;
        y_g_act;
        th_g_act;
        xl_g;
        yl_g;
        thl_g;
        err_x_g_ref;
        err_y_g_ref;
        err_th_g_ref;
        i;
    end
    
    methods(Access = public)    
        function obj = RobotState(n, init_pose)
            obj.i = 1;
            
            obj.t = zeros(1, n);
            obj.V = zeros(1, n);
            obj.w = zeros(1, n);
            obj.s = zeros(1, n);
            obj.x = zeros(1, n);
            obj.y = zeros(1, n);
            obj.th = zeros(1, n);
            
            obj.xl = zeros(1,n);
            obj.yl = zeros(1,n);
            obj.thl = zeros(1,n);
            obj.xl_g = zeros(1,n);
            obj.yl_g = zeros(1,n);
            obj.thl_g = zeros(1,n);
            
            obj.x_g_act = zeros(1,n);
            obj.y_g_act = zeros(1,n);
            obj.th_g_act = zeros(1,n);
            obj.x_g_ref = zeros(1,n);
            obj.y_g_ref = zeros(1,n);
            obj.th_g_ref = zeros(1,n);
            obj.err_x_g_ref = zeros(1,n);
            obj.err_y_g_ref = zeros(1,n);
            obj.err_th_g_ref = zeros(1,n);
                                         
            
            obj.xl_g(1:n) = init_pose.x;
            obj.yl_g(1:n) = init_pose.y;
            obj.thl_g(1:n) = init_pose.th;
            obj.x_g_act(1:n) = init_pose.x;
            obj.y_g_act(1:n) = init_pose.y;
            obj.th_g_act(1:n) = init_pose.th;
            obj.x_g_ref(1:n) = init_pose.x;
            obj.y_g_ref(1:n) = init_pose.y;
            obj.th_g_ref(1:n) = init_pose.th;

            
            
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
