%%
clc;
close all;
clearvars -except robot;
figure('Position',[1500, 0, 1400, 1700]);
% figure('Position',[1500, 0, 1700, 1400]);
% axis([-.1 12*8*.0254+.1 -.1 12*6*.0254+.1]);


robot = raspbot;
robot.startLaser();
robot.laser.NewMessageFcn=@laserEventListener;
robot.encoders.NewMessageFcn=@encoderEventListener;
pause(1);

resetFigureAndDrawMap();


pickUp1 = Pose(12*1*.0254,12*6*.0254,pi/2);
pickUp2 = Pose(12*2*.0254,12*6*.0254,pi/2); 
pickUp3 = Pose(12*3*.0254,12*6*.0254,pi/2);
pickUp4 = Pose(12*4*.0254,12*6*.0254,pi/2);
pickUp5 = Pose(12*5*.0254,12*6*.0254,pi/2); 
pickUp6 = Pose(12*6*.0254,12*6*.0254,pi/2);
pickUp7 = Pose(12*7*.0254,12*6*.0254,pi/2);
pickUp8 = Pose(12*7*.0254,12*4*.0254,0); 
pickUp9 = Pose(12*7*.0254,12*3*.0254,0);
pickUp10 = Pose(12*7*.0254,12*2*.0254,0);
%pickup_array = [pickUp6 pickUp5 pickUp4 pickUp3 pickUp2 pickUp1 pickUp7 pickUp8 pickUp9 pickUp10 ];
pickup_array = [pickUp1 pickUp2 pickUp3 pickUp4 pickUp10 pickUp9 pickUp8 pickUp5 pickUp6 pickUp7 ];
drop1 = Pose(12*1*.0254,12*1*.0254,-pi/2);
drop2 = Pose(12*2*.0254,12*1*.0254,-pi/2);
drop3 = Pose(12*3*.0254,12*1*.0254,-pi/2);
drop4 = Pose(12*4*.0254,12*1*.0254,-pi/2);
drop5 = Pose(12*5*.0254,12*1*.0254,-pi/2);
drop6 = Pose(12*6*.0254,12*1*.0254,-pi/2);
drop7 = Pose(12*7*.0254,12*1*.0254,-pi/2);
%dropoff_array = [ drop4 drop3 drop2 drop1 drop5 drop6 drop7];
dropoff_array = [ drop1 drop2 drop3 drop4 drop5 drop6 drop7 drop7 drop7 drop7];

maxV = .25;
maxVSail = .15;
errorT = .003;
trajFollower = TrajectoryFollower(errorT, maxVSail);
init_pose = Pose(.3048, .3048, -pi()/2.0);
trajFollower.last_pose = init_pose;


dropped = 0;
pickup_index = 1;
robot.forksDown();

while(dropped < 10)
    pickUp = pickup_array(pickup_index);
    drop = dropoff_array(dropped+1);
    
    current_robot_pose = trajFollower.last_pose;
    aquisition = aquisitionPose(pickUp, maxV);
    
    turn_angle = atan2((aquisition.y - current_robot_pose.y), (aquisition.x - current_robot_pose.x)) - current_robot_pose.th;
    trajFollower.turn(robot, turn_angle);
    
    trajFollower.executeTrajectory(robot, trajFollower.last_pose, aquisition, maxV);

    foundSail = 0;
    count = 0;
    while(foundSail == 0)
        if(count > 10)
            %sail not found, need to move on to the next sail
            break;
            
        end
        foundSail = trajFollower.go_to_sail(robot); 
        if(foundSail)
            trajFollower.ramSail(robot);
            robot.forksUp();
            pause(1);
        end
        count = count+1;
    end
    
    if(foundSail)
        turn_angle = atan2((drop.y - trajFollower.last_pose.y), (drop.x - trajFollower.last_pose.x)) - trajFollower.last_pose.th;
        trajFollower.turn(robot, turn_angle);
        trajFollower.executeTrajectory(robot, trajFollower.last_pose, drop, maxV);
        robot.forksDown();
        pause(1);
        dropped = dropped + 1;
        trajFollower.backUp(robot, .15);
    else      
        trajFollower.backUp(robot, .5);
    end
    
    pickup_index = pickup_index + 1;
    resetFigureAndDrawMap();
end
