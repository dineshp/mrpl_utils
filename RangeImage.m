classdef RangeImage 
    methods (Static)
        function [ pc] = irToXy( img )
            % irToXy finds position and bearing of a range pixel endpoint
            % Finds the position and bearing of the endpoint of a range pixel in 
            % the plane.
            i = img(:,1);
            r = img(:,2);
            deg = i-1;
            %if(deg > 180)
            deg = deg - 360 * (deg >180); 
            %end
            
            th = (i-1)*(pi/180);
            x = r.*cos(th);
            y = r.*sin(th);
            pc = [ x y];
        end
    end
end

% classdef RangeImage 
%     methods (Static)
%         function [ x, y, th] = irToXy( i, r )
%             % irToXy finds position and bearing of a range pixel endpoint
%             % Finds the position and bearing of the endpoint of a range pixel in 
%             % the plane.
%             deg = i-1;
%             if(deg > 180)
%                deg = deg - 360; 
%             end
% 
%             th = deg2rad(deg);
%             x = r*cos(deg2rad(deg));
%             y = r*sin(deg2rad(deg));
%         end
%     end
% end