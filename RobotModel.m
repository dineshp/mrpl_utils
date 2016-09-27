classdef RobotModel
    properties (Constant)
        ModelW = .09;%.08943;
    end
    
    methods (Static)        
        function [ x, y, th] = modelTrajectory(vl, vr, t0, tf, dt)
            totalTimeSteps = tf/dt + 1;
            n = floor(totalTimeSteps);
            equivCheck = MatrixFxns.matrixDimensionEqual(vl, vr);
            if(equivCheck && eq(n, length(vr)))
                x = zeros(1,n+1);
                y = zeros(1,n+1);
                
%                 fig1 = plot(x, y, 'b-');
%                 xlim([-0.5 0.5]);
%                 ylim([-0.5 0.5]);
%                 title('Real Time State Simulation');
                
                th = zeros(1,n+1);
                x(1)=0;
                y(1)=0;
                th(1)=0;
                               
                for i=1:1:n
                    V = (vr(i) + vl(i))/2;
                    omega =  (vr(i) - vl(i))/RobotModel.ModelW;
                    th(i+1) = th(i) + omega*dt;
                    
                    k00 = V * cos(th(i));
                    k01 = V * sin(th(i));

                    k10 = V * cos(th(i) + (dt/2.0) * omega);
                    k11 = V * sin(th(i) + (dt/2.0) * omega);

                    k20 = k10;
                    k21 = k11;

                    k30 = V * cos(th(i) + omega*dt);
                    k31 = V * sin(th(i) + omega*dt);
                    
                    x(i+1) = x(i) + ((dt/6.0) * (k00 + 2*(k10 + k20) + k30));
                    y(i+1) = y(i) + ((dt/6.0) * (k01 + 2*(k11 + k21) + k31));
%                     set(fig1, 'xdata', [get(fig1,'xdata') x(i+1)], 'ydata', [get(fig1,'ydata') y(i+1)]);
%                     pause(dt);
                end
            else
                error('velocity vector dimensions not equal');
            end
        end
        
        function [ vl, vr ] = getCorSpiralTrajectory(V, k, tf, dt)
            totalTimeSteps = tf/dt;
            n = floor(totalTimeSteps)+1;

            time = 0;

            vl = zeros(1,n);
            vr = zeros(1,n);
            for i=1:1:n
                omega = k*time;
                vl_t = V - ((RobotModel.ModelW/2)*omega);
                vr_t = V + ((RobotModel.ModelW/2)*omega);
                vl(i) = vl_t; 
                vr(i) = vr_t;
                
                time = time + dt;
            end

        end
        
        function [ vl, vr ] = getFigureEightTrajectory(V, tf)
            totalTimeSteps = tf/.001;
            n = ceil(totalTimeSteps)+1;

            time = 0;

            vl = zeros(1,n);
            vr = zeros(1,n);
            k_s = 3.0;
            s_f = 1.0*k_s;
            k_th = (2.0*pi)/s_f;
            k_k = (15.1084/k_s);
            for i=1:1:n
                s = V*time;
                omega = k_k*sin(k_th*s)*V;
                vl_t = V - ((RobotModel.ModelW/2.0)*omega);
                vr_t = V + ((RobotModel.ModelW/2.0)*omega);
                vl(i) = vl_t; 
                vr(i) = vr_t;
                 
                time = time + .001;
            end
        end
    end
end