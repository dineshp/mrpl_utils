function bodyPts = bodyGraph()
    rad = 0.06;
    % robot radius
      % laser housing radius
    % return an array of points that can be used to plot the robot
    % body in a window.
    % angle arrays
    step = pi/20;
    cir = 0: step: 2.0*pi;
    s = 0.0635;

    % distance d from F1 to the goal 
    d = .03985;
    
    f = s+d;

    % body with implicit line to laser circle
    bx = [0 f (f/5)*cos(cir)];
    by = [0 0 (f/5)*sin(cir)];

    %create homogeneous points
    bodyPts = [bx ; by ; ones(1,size(bx,2))];
end