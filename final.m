%%

robot = raspbot;
robot.startLaser();
robot.laser.NewMessageFcn=@laserEventListener;
robot.encoders.NewMessageFcn=@encoderEventListener;
pause(5);

%%
clc;
close all;
clearvars -except robot;
figure('Position',[1500, 0, 1400, 1700]);
% figure('Position',[1500, 0, 1700, 1400]);
% axis([-.1 12*8*.0254+.1 -.1 12*6*.0254+.1]);


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

pickUp11 = Pose(12*1*.0254,12*8*.0254,0); 
pickUp12 = Pose(12*1*.0254,12*9*.0254,0);
pickUp13 = Pose(12*1*.0254,12*10*.0254,0);

%final             1       2       3       4       5       6       7      8        9        10      11        12      13      
pickup_array = [pickUp6 pickUp5 pickUp4 pickUp3 pickUp2 pickUp1 pickUp10 pickUp9 pickUp8 pickUp7 pickUp11 pickUp12 pickUp13];
pickup_array_translate = [6, 5, 4, 3, 2, 1, 10, 9, 8, 7, 11, 12, 13];
% %lab13
% pickup_array = [pickUp1 pickUp2 pickUp3 pickUp4 pickUp10 pickUp9 pickUp8 pickUp5 pickUp6 pickUp7 ];

drop1 = Pose(12*1*.0254,12*1*.0254,-pi/2);
drop2 = Pose(12*2*.0254,12*1*.0254,-pi/2);
drop3 = Pose(12*3*.0254,12*1*.0254,-pi/2);
drop4 = Pose(12*4*.0254,12*1*.0254,-pi/2);
drop5 = Pose(12*5*.0254,12*1*.0254,-pi/2);
drop6 = Pose(12*6*.0254,12*1*.0254,-pi/2);
drop7 = Pose(12*7*.0254,12*1*.0254,-pi/2);

%final               1    2      3    4     5    6      7     8     9      10   11    12   13  
dropoff_array = [ drop1 drop2 drop3 drop4 drop5 drop6 drop7 drop7 drop7 drop7 drop7 drop7 drop7];

% %lab13
% dropoff_array = [ drop1 drop2 drop3 drop4 drop5 drop6 drop7 drop7 drop7 drop7];

%final
maxV = .5;
maxVSail = .35;
errorT = .001;

% %lab13
% maxV = .25;
% maxVSail = .15;
% errorT = .001;

trajFollower = TrajectoryFollower(errorT, maxVSail);
init_pose = Pose(.3048, .3048, -pi()/2.0);
trajFollower.last_pose = init_pose;


dropped = 0;
pickup_index = 1;
robot.forksDown();
missing_sails = [];
while(pickup_index <= 10) 
    pickUp = pickup_array(pickup_index);
    sail_index = pickup_array_translate( pickup_index );
    
    drop = dropoff_array(sail_index);
    
    if((sail_index == 7 || sail_index == 8 || sail_index == 9 || sail_index == 10) && (~isempty(missing_sails)))
        missing_sails = unique(missing_sails);
        missing_sails = sort(missing_sails, 'descend');
        sail_index_shift = missing_sails(1);
        missing_sails = missing_sails(2:end);
        drop = dropoff_array(sail_index_shift);
    end
    
    current_robot_pose = trajFollower.last_pose;
    aquisition = aquisitionPose(pickUp, maxV);
    
    turn_angle = atan2((aquisition.y - current_robot_pose.y), (aquisition.x - current_robot_pose.x)) - current_robot_pose.th;
    trajFollower.turn(robot, turn_angle);
    
    if(sail_index >=8)
        trajFollower.executeTrajectory(robot, trajFollower.last_pose, aquisition, .25);
    else
        trajFollower.executeTrajectory(robot, trajFollower.last_pose, aquisition, maxV);
    end

    foundSail = 0;
    sail_to_left = 0;
    count = 0;
    while(foundSail == 0)
        if(count > 5)
            %sail not found, need to move on to the next sail
            break;
            
        end
        [foundSail, sail_to_left] = trajFollower.go_to_sail(robot, sail_index); 
        if(foundSail)
            trajFollower.ramSail(robot);
            robot.forksUp();
            pause(.1);            
            break;
        end

        
        count = count+1;
    end
    
    
    
    if(foundSail)
        turn_angle = atan2((drop.y - trajFollower.last_pose.y), (drop.x - trajFollower.last_pose.x)) - trajFollower.last_pose.th;    
        picked_up = trajFollower.is_sail_on_fork();
        if(picked_up)
            trajFollower.turn(robot, turn_angle);      
            trajFollower.executeTrajectory(robot, trajFollower.last_pose, drop, maxV);
            robot.forksDown();
            dropped = dropped + 1;
            trajFollower.backUp(robot, .15);
        else
            trajFollower.turn(robot, pi());
            missing_sails = [sail_index missing_sails];
            trajFollower.backUp(robot, .5 + 1.5*(.3048));
            robot.forksDown();
        end
        
    else
        missing_sails = [sail_index missing_sails];
        trajFollower.backUp(robot, .5);
    end
    
        %do something with sails and determine if next sail is present
    switch sail_index
        case 6
            if(sail_to_left == 0)             
                pickup_index = pickup_index + 1;
                missing_sails = [ pickup_array_translate(pickup_index) missing_sails];
                disp('skipping the next sail!');
            end
        case 5
            if(sail_to_left == 0)
                pickup_index = pickup_index + 1;
                missing_sails = [ pickup_array_translate(pickup_index) missing_sails];
                disp('skipping the next sail!');
            end
        case 4
            if(sail_to_left == 0)
                pickup_index = pickup_index + 1;
                missing_sails = [ pickup_array_translate(pickup_index) missing_sails];
                disp('skipping the next sail!');
            end
        case 3
            if(sail_to_left == 0)
                pickup_index = pickup_index + 1;
                missing_sails = [ pickup_array_translate(pickup_index) missing_sails];
                disp('skipping the next sail!');
            end
        case 2
            if(sail_to_left == 0)
                pickup_index = pickup_index + 1;
                if(~(sail_index == 7 || sail_index == 8 || sail_index == 9 || sail_index == 10))
                    missing_sails = [ pickup_array_translate(pickup_index) missing_sails];
                end
                disp('skipping the next sail!');
            end  
        otherwise
            disp('going to the next sail!');
    end
    
    pickup_index = pickup_index + 1;
    
    resetFigureAndDrawMap();
end


%% avg count .1 in front
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

off = 0;
p1 = [0 ; 0];
p2 = [ 12*8*.0254 ; 0];
p3 = [0 ; 12*12*.0254 ];
p4 = [12* 8*.0254; 12*12*.0254];
lines_p1 = [p2-[off;0] p1+[0;off] p2+[0;off] p3+[off;0]];
lines_p2 = [p1+[off;0] p3-[0;off] p4-[0;off] p4-[off;0]];
lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);


maxV = .25;
maxVSail = .15;
errorT = .001;

trajFollower = TrajectoryFollower(errorT, maxVSail);
start_pose = Pose(3*0.3048, 6*0.3048, -pi()/2.0);
init_pose = start_pose;
trajFollower.last_pose = init_pose;
[rc, init_pose] = trajFollower.localize(init_pose);
while(1)
    total_count = 0;
    for m = 1:1:20
      
        x = [];
        y = [];
        %offset is .25 for 7 and .75 for 8, .5 for 6
        offset = .25;
        scan = getS;
        count = 0;
        for n=1:1:360
            th = (n-1)*pi/180;
            wedge_width_left = 2*pi();
            wedge_width_right = 2*pi();
            if((abs(scan(n)) <= .1) && ((th <= wedge_width_left ) || (th >= 2*pi() - wedge_width_right)) )
                x(n) = scan(n)*cos(th);
                y(n) = scan(n)*sin(th);

            else
              x(n) = 0;
              y(n) = 0;
            end

            if(abs(scan(n)) <= .1)
                count = count + 1;
                x_full_nowedge(n) = scan(n)*cos(th);
                y_full_nowedge(n) = scan(n)*sin(th);

            else
              x_full_nowedge(n) = 0;
              y_full_nowedge(n) = 0;
            end

        end
        %<123 sail is on fork
        %>
        x_full = x;
        y_full = y;

                w_nowedge = ones(1,length(x_full_nowedge));
                laserPts_nowedge = [x_full_nowedge; y_full_nowedge; w_nowedge];
                ids = lml.throwInliers(init_pose,laserPts_nowedge);
                laserPts_nowedge(:,ids) = 0;
                world_laserPts_nowedge = init_pose.bToA()*laserPts_nowedge;


                w_full = ones(1,length(x_full));
                laserPts = [x_full; y_full; w_full];


                x = laserPts(1,:);
                y = laserPts(2,:);
                world_laserPts = init_pose.bToA()*laserPts;


                    pause(.01);

        total_count = total_count + count;
    end
    disp(total_count/20);
end

%% detect pick up




%% verify sail on hook
clc;
close all;
clearvars -except robot;
figure('Position',[1500, 0, 1400, 1700]);
% figure('Position',[1500, 0, 1700, 1400]);
% axis([-.1 12*8*.0254+.1 -.1 12*6*.0254+.1]);


resetFigureAndDrawMap();
off = 0;
p1 = [0 ; 0];
p2 = [ 12*8*.0254 ; 0];
p3 = [0 ; 12*12*.0254 ];
p4 = [12* 8*.0254; 12*12*.0254];
lines_p1 = [p2-[off;0] p1+[0;off] p2+[0;off] p3+[off;0]];
lines_p2 = [p1+[off;0] p3-[0;off] p4-[0;off] p4-[off;0]];
lml = LineMapLocalizer(lines_p1,lines_p2,.01,.001,.0005);
init_pose = Pose(4*0.3048, 6*0.3048, -pi()/2.0);

%lab13
maxV = .25;
maxVSail = .15;
errorT = .001;

trajFollower = TrajectoryFollower(errorT, maxVSail);
start_pose = Pose(4*0.3048, 6*0.3048, -pi()/2.0);

load('rangeImages_nosail_4ft_6ft_neg_pi-over-2.mat');
for i=1:1:10000
    scan = transpose(rangeImages(i,:));  
 
        init_pose = trajFollower.localizeSim(init_pose, scan);
        
    
    hold on;
    plotRobotAnotateC(init_pose);
    x = [];
    y = [];
    %offset is .25 for 7 and .75 for 8, .5 for 6
    offset = .25;
    for n=1:1:360
        th = (n-1)*pi/180;
        wedge_width = pi()/2;
        if((abs(scan(n)) <= 3 && abs(scan(n)) > .1) && ((th <= wedge_width) || (th >= 2*pi() - wedge_width)) )
            x(n) = scan(n)*cos(th);
            y(n) = scan(n)*sin(th);

        else
          x(n) = 0;
          y(n) = 0;
        end

        if((abs(scan(n)) <= 3 && abs(scan(n)) > .1))
            x_full_nowedge(n) = scan(n)*cos(th);
            y_full_nowedge(n) = scan(n)*sin(th);

        else
          x_full_nowedge(n) = 0;
          y_full_nowedge(n) = 0;
        end

    end
    x_full = x;
    y_full = y;

    plotRobotAnotateC(init_pose);

            w_nowedge = ones(1,length(x_full_nowedge));
            laserPts_nowedge = [x_full_nowedge; y_full_nowedge; w_nowedge];
            world_laserPts_nowedge = init_pose.bToA()*laserPts_nowedge;
            scatter(world_laserPts_nowedge(1,:), world_laserPts_nowedge(2,:), 'b');
            hold on;






            
            
            w_full = ones(1,length(x_full));
            laserPts = [x_full; y_full; w_full];
            x = laserPts(1,:);
            y = laserPts(2,:);
            world_laserPts = init_pose.bToA()*laserPts;
            scatter(world_laserPts(1,:), world_laserPts(2,:), 'b', 'filled');
            hold on;
            

                pause(1);
    disp('reset');
    resetFigureAndDrawMap();
    hold on;

end



