clc;
%clearvars -except robot;
close all;
format long g;
% if(~exist('robot', 'var'))
%     robot = raspbot();
%     robot.encoders.NewMessageFcn=@encoderEventListener;
%     robot.startLaser();
%     pause(3);
% end
%
pause(1);
p1 = [0 ; 0];
    p2 = [ 48*.0254 ; 0];
    p3 = [0 ; 48*.0254 ];
    p4 = [48*.0254; 48*.0254];
    lines_p1 = [p2 p1 p2];
    lines_p2 = [p1 p3 p4];
intialPose = Pose(0.6096, 0.6096, pi()/2.0);
%sf = sailFinder(lines_p1,lines_p2,.01,.001,.005);

axis([-.25 2 -.25 2]);
for i = 1:length(lines_p1(1,:))
    hold on;
    
    plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
end
 hold off;  
 %% 
 pickUp1 = Pose(1*.0254,3.5*.0254,pi/2);
  pickUp2 = Pose(2*.0254,3.5*.0254,pi/2); 
  pickUp3 = Pose(3*.0254,3.5*.0254,pi/2);
  pickUpArray = [pickUp1 pickUp2 pickUp3];
  drop1 = Pose(1.75*.0254,0.5*.0254,-pi/2);
  drop2 = Pose((1.75+.5)*.0254,0.5*.0254,-pi/2);
  drop3 = Pose((1.75+1)*.0254,0.5*.0254,-pi/2);
  dropArray = [drop1 drop2 drop3];