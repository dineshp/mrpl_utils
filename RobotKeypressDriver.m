classdef RobotKeypressDriver < handle
 %robotKeypressDriver Creates a keyboard event handler and then lets
 % the suser drive the robot with the arrow keys.

    properties(Constant)
    end

    properties(Access = private)
    fh = [];
    end

    properties(Access = public)
    end

    methods(Access = public)
        function r = drive(obj, robot,vGain)
            % drive the robot
            Vmax = 0.02*vGain;
            dV = 0.006*vGain;
            key = pollKeyboard();
            d = getDirection();
            if(key ~= false)
                if(strcmp(key,'uparrow'))

                    robot.sendVelocity(Vmax,Vmax);
                elseif(strcmp(key,'downarrow'))

                    robot.sendVelocity(-Vmax,-Vmax);
                elseif(strcmp(key,'leftarrow'))

                    robot.sendVelocity(d*Vmax,d*(Vmax+dV));
                elseif(strcmp(key,'rightarrow'))

                    robot.sendVelocity(d*(Vmax+dV),d*Vmax);
                elseif(strcmp(key,'s'))

                    robot.sendVelocity(0.0,0.0);
                elseif(strcmp(key,'a'))
                    disp('speeding up');
                elseif(strcmp(key,'z'))
                    disp('slowing down');
                elseif(strcmp(key,'q'))
                    error('quitting');  
                end;


                r = true;
                return;
            end;
            
            r = false;
            return;
        end


        function obj = RobotKeypressDriver(fh)
            % create a robotKeypressDriver for the figure handle
            % normally you call this with gcf for fh
            obj.fh = fh;
            set(fh,'KeyPressFcn',@keyboardEventListener);
        end
    end

end

function d = getDirection()
    global keypressKeyPrev;
    if(strcmp(keypressKeyPrev,'downarrow'))
        d = -1;
    elseif(strcmp(keypressKeyPrev,'uparrow'))
        d = 1;
    else
        d = 1;
    end

end

