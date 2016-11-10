function laserEventListener(handle,event)
global S
global ST
% EncoderEventListener Invoked when new encoder data arrives.
    % A MATLAB event listener for the Robot. Invoked when
    % encoder data arrives.
   
      S = event.Ranges;
      ST = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1e9;
      setST(ST);
      setS(S);
    
               
    %disp(event.Header);
    % MessageType: 'std_msgs/Header'
    %         Seq: 16440
    %       Stamp: [1x1 Time]
    %     FrameId: ''

    %disp(event.Header.Stamp);
    % Sec: 1471645779
    % Nsec: 347089052

    %disp(event.Vector);
    % MessageType: 'geometry_msgs/Vector3'
    %           X: 0
    %           Y: 0
    %           Z: 0


    % Do some stuff

    %encoderDataTimestamp = double(event.Header.Stamp.Sec) + ...
    %double(event.Header.Stamp.Nsec)/1000000000.0;
    %encoderFrame = encoderFrame + 1;

    % Do some more stuff

end