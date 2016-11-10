%% scan = robot.laser.LatestMessage.Ranges;

clc;
clearvars -except robot;
close all;
format long g;
if(~exist('robot', 'var'))
    robot = raspbot();
    robot.encoders.NewMessageFcn=@encoderEventListener;
    pause(3);
end
pause(1);


try

    
    
    
catch ME
    robot.encoders.NewMessageFcn=[];
    robot.encoders.NewMessageFcn=@encoderEventListener;
    if (strcmp(ME.identifier,'TRJFC:ENCODER_ISSUE'))
        msg = [ME.identifier];
        causeException = MException('MAIN:TRJFC:ENCODER_ISSUE',msg);
        ME = addCause(ME,causeException);
    end
   rethrow(ME);
end 
