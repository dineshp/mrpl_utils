classdef RangeImage 
    methods (Static)
        function [ x, y, th] = irToXy( i, r )
            % irToXy finds position and bearing of a range pixel endpoint
            % Finds the position and bearing of the endpoint of a range pixel in 
            % the plane.
            deg = i-1;
            if(deg > 180)
               deg = deg - 360; 
            end

            th = deg2rad(deg);
            x = r*cos(deg2rad(deg));
            y = r*sin(deg2rad(deg));
        end
    end
end