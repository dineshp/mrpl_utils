%%
clc;
close all;
clearvars -except robot;
figure;
axis([-.25 1.4 -.25 1.4]);
trajFollower = TrajectoryFollower();
init_pose = Pose(.3048, .3048, -pi()/2.0);
trajFollower.last_pose = init_pose;

p1 = [0 ; 0];
p2 = [ 48*.0254 ; 0];
p3 = [0 ; 48*.0254 ];
p4 = [48*.0254; 48*.0254];
lines_p1 = [p2 p1 p2];
lines_p2 = [p1 p3 p4];

for i = 1:length(lines_p1(1,:))
    hold on;
    
    plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
end
robot = raspbot;
robot.laser.NewMessageFcn=@laserEventListener;
robot.encoders.NewMessageFcn=@encoderEventListener;
pause(1);

pickUp1 = Pose(1*.0254*12,12*3.5*.0254,pi/2);
aquisition1 = aquisitionPose(pickUp1);
trajFollower.turn(robot);
trajFollower.localize_and_plot(robot);
trajFollower.executeTrajectory(robot, trajFollower.last_pose, aquisition1, .2);

foundSail = 0;
robot.forksDown();
while(foundSail == 0);
    foundSail = trajFollower.go_to_sail(robot); 
    if(foundSail)
        trajFollower.ramSail(robot);
        pause(2);
        robot.forksUp();
        pause(2);
    end
    pause(.05);
end
trajFollower.turn(robot);
drop1 = Pose(12*1.5*.0254,12*0.5*.0254,-pi/2);
trajFollower.executeTrajectory(robot, trajFollower.last_pose, drop1, .2);
pause(2);
robot.forksDown();
pause(2);
trajFollower.backUp(robot);
trajFollower.turn(robot);
pause(1);



clf;
axis([-.25 1.4 -.25 1.4]);

p1 = [0 ; 0];
p2 = [ 48*.0254 ; 0];
p3 = [0 ; 48*.0254 ];
p4 = [48*.0254; 48*.0254];
lines_p1 = [p2 p1 p2];
lines_p2 = [p1 p3 p4];

for i = 1:length(lines_p1(1,:))
    hold on;
    
    plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
end


pickUp2 = Pose(12*2*.0254,12*3.5*.0254,pi/2);
aquisition2 = aquisitionPose(pickUp2);
trajFollower.localize_and_plot(robot);
trajFollower.executeTrajectory(robot, trajFollower.last_pose, aquisition2, .2);

foundSail = 0;
robot.forksDown();
while(foundSail == 0);
    foundSail = trajFollower.go_to_sail(robot); 
    if(foundSail)
        trajFollower.ramSail(robot);
        pause(2);
        robot.forksUp();
        pause(2);
    end
    pause(.05);
end
trajFollower.turn(robot);
drop2 = Pose(12*(2.25)*.0254,12*0.5*.0254,-pi/2);
trajFollower.executeTrajectory(robot, trajFollower.last_pose, drop2, .2);
pause(2);
robot.forksDown();
pause(2);
trajFollower.backUp(robot);
trajFollower.turn(robot);
pause(1);

clf;
axis([-.25 1.4 -.25 1.4]);

p1 = [0 ; 0];
p2 = [ 48*.0254 ; 0];
p3 = [0 ; 48*.0254 ];
p4 = [48*.0254; 48*.0254];
lines_p1 = [p2 p1 p2];
lines_p2 = [p1 p3 p4];

for i = 1:length(lines_p1(1,:))
    hold on;
    
    plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
end


pickUp3 = Pose(12*3*.0254,12*3.5*.0254,pi/2);
aquisition3 = aquisitionPose(pickUp3);
trajFollower.localize_and_plot(robot);
trajFollower.executeTrajectory(robot, trajFollower.last_pose, aquisition3, .2);

foundSail = 0;
robot.forksDown();
while(foundSail == 0);
    foundSail = trajFollower.go_to_sail(robot); 
    if(foundSail)
        trajFollower.ramSail(robot);
        pause(2);
        robot.forksUp();
        pause(2);
    end
    pause(.05);
end
trajFollower.turn(robot);
drop3 = Pose(12*(3)*.0254,12*0.5*.0254,-pi/2);
trajFollower.executeTrajectory(robot, trajFollower.last_pose, drop3, .2);
pause(2);
robot.forksDown();
pause(2);
trajFollower.backUp(robot);
trajFollower.turn(robot);
pause(1);



%%
clf;
close all;
figure;
            scan = getS;
                                    x = zeros(1,1);
            y = zeros(1,1);
            for n=1:1:360
                th = (n-1)*pi/180;
                th = th - atan2(.024, .28);
                if((abs(scan(n)) <= 1.3 && abs(scan(n)) > .1) && (abs(th) < pi()/6))
                    x(n) = scan(n)*cos(th);
                    y(n) = scan(n)*sin(th);

                else
                  x(n) = 0;
                  y(n) = 0;
                end
            end
            x_full = x;
            y_full = y;


            w_full = ones(1,length(x_full));
            laserPts = [x_full; y_full; w_full];
            laserPts_full = laserPts;
            world_laserPts_full = laserPts_full;
            scatter(world_laserPts_full(1,:), world_laserPts_full(2,:), 'r');

            hold on;
