classdef RobotState < handle

    properties(Access = public)
        t
        V
        w
        s
        x
        y
        th
        i
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
        end
        
        function iPlusPlus(obj)
            obj.i = obj.i + 1;
        end
    end
    
end
