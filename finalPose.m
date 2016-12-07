function pose = finalPose( p_pose)
    %Determine the final position and orientation we want to go to to reach the
    %object at coordinates x0, y0, w0
    xf1 = p_pose.x;
    yf1 = p_pose.y;
    th = p_pose.th;

    % distance s from center of robot to front of robot 
    s = 0.0635; %for aligning the camera 
  

    % distance d from F1 to the goal 
    %d = -.03985;
    d = -.06985;
    
    f = s+d;
    
    % Homogenous Transf. from front of robot, s, to center of robot( assumed to
    % be 0
    T_sr = bToA(Pose(s, 0, 0));

    % Get Translation and rotation seperatly for f1 to s and then get full 
    % Homogenous Transform
    T_f1s_trans = Pose(xf1-s, yf1, 0);
    T_f1s_rot = Pose(0, 0, th);
    T_f1s = bToA(T_f1s_trans) * bToA(T_f1s_rot);

    % Homogenous Transf. from goal to f1
    T_gf1 = bToA(Pose(d-s, 0, 0));

    T_gr = T_sr * T_f1s * T_gf1;

    % Not sure how to call this on Pose Class :/
    x = T_gr(1,3);
    y = T_gr(2,3);
    w = atan2(-T_gr(1,2),T_gr(1,1));
    pose = Pose(x, y, w);
    %details(pose);
end
