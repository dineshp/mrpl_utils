classdef RobotDriver
    properties (Constant)
        DriverW = RobotModel.ModelW;
        P_Gain = .005;
        D_Gain = RobotDriver.P_Gain*10;
        I_Gain = RobotDriver.P_Gain*100;
    end
    
    methods (Static)
        function [ x, y, th ] = integrateDiffEq(V, omega, dt, p_x, p_y, p_th)
            th = p_th + omega*dt;

            k00 = V * cos(p_th);
            k01 = V * sin(p_th);

            k10 = V * cos(p_th + (dt/2.0) * omega);
            k11 = V * sin(p_th + (dt/2.0) * omega);

            k20 = k10;
            k21 = k11;

            k30 = V * cos(p_th + omega*dt);
            k31 = V * sin(p_th + omega*dt);

            x = p_x + ((dt/6.0) * (k00 + 2*(k10 + k20) + k30));
            y = p_y + ((dt/6.0) * (k01 + 2*(k11 + k21) + k31));
        end
        
        function driveTrajectory2(robot, vl, vr, t_f)
            equivCheck = MatrixFxns.matrixDimensionEqual(vl, vr);
            
            if(~equivCheck)
                error('velocity vector dimensions not equal');
            end
     
            n = length(vl);
         
            x = zeros(1,n+1);
            y = zeros(1,n+1);
            th = zeros(1,n+1);
            x(1)=0;
            y(1)=0;
            th(1)=0;
            prevX = getX;
            prevY = getY;
            pT = getT;
            cT = pT;
            curX = prevX;
            curY = prevY;

            
            fig1 = plot(x, y, 'b-o');
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title('Real Time State Estimate');
            
            i = 0;
            t = .001;
            t_del = .5;
            while(t <= (t_f + t_del))
                i = i + 1;
                vel_index = max(1, floor((t-t_del)/.001));
                robot.sendVelocity(vl(vel_index), vr(vel_index)); 
               
                count = 0;
                while(eq(cT, pT))     
                    count = count + 1;
                    cT = getT;
                    curX = getX;
                    curY = getY;
                    pause(.001);
                end
                
                dt_i = cT - pT;      
                vl_i = (curX-prevX)/dt_i;
                vr_i = (curY-prevY)/dt_i;
                pT = cT;
                prevX = curX;
                prevY = curY;

                V = (vr_i + vl_i)/2;
                omega =  (vr_i  - vl_i)/RobotDriver.DriverW;
                [x, y, th] = RobotDriver.integrateDiffEq(V, omega, dt_i, x(i), y(i), th(i));
                th(i+1) = th;  
                x(i+1) = x;
                y(i+1) = y;
                
                set(fig1, 'xdata', [get(fig1,'xdata') -y(i+1)], 'ydata', [get(fig1,'ydata') x(i+1)]);
                t = t + dt_i;
                pause(.05 - (.001*count));
            end
            robot.stop(); 
        end
        
        function r = driveTrajectory(robot, vl, vr, pause_t, dt_traj)
            equivCheck = MatrixFxns.matrixDimensionEqual(vl, vr);
            
            if(~equivCheck)
                error('velocity vector dimensions not equal');
            end
     
            n = length(vl);
         
            x = zeros(1,n+1);
            y = zeros(1,n+1);
            th = zeros(1,n+1);
            x(1)=0;
            y(1)=0;
            th(1)=0;
            prevX = robot.encoders.LatestMessage.Vector.X;
            prevY = robot.encoders.LatestMessage.Vector.Y;
            pT = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + (double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000);
            
            fig1 = plot(x, y, 'b-o');
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title('Real Time State Estimate');
            
            tStart = tic;
            totalError = 0;
            p_err = 0;
            for i=1:1:n
                robot.sendVelocity(vl(i), vr(i)); 
                
                dt_e = toc(tStart);
                tStart = tic;
                if(i == 1)
                    err = 0;
                    changeError = 0;
                else
                    err = dt_traj - dt_e;
                    changeError = err - p_err;
                    p_err = err;
                    totalError = totalError + err;   
                end
                
                count = 0;
                cT = getT;
                while(eq(cT, pT))     
                    cT = getT;
                    count = count + 1;
                    pause(.001);
                end
                curX = getX;
                curY = getY;

                dt_i = (cT - pT);
                pT = cT;
                vl_i = (curX-prevX)/dt_i;
                vr_i = (curY-prevY)/dt_i;
                prevX = curX;
                prevY = curY;

                V = (vr_i + vl_i)/2;
                omega =  (vr_i  - vl_i)/RobotDriver.DriverW;
                [x, y, th] = RobotDriver.integrateDiffEq(V, omega, dt_i, x(i), y(i), th(i));
                th(i+1) = th;  
                x(i+1) = x;
                y(i+1) = y;
                
                set(fig1, 'xdata', [get(fig1,'xdata') -y(i+1)], 'ydata', [get(fig1,'ydata') x(i+1)]);
                
                pauseC = (pause_t-(.001*count)) + err*RobotDriver.P_Gain + changeError*RobotDriver.D_Gain + totalError*RobotDriver.I_Gain;
                pauseC = max(0, pauseC);
                pause(pauseC);
            end
            r = totalError/n;
            robot.stop(); 
        end
    end
end