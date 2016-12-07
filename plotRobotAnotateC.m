function plotRobotAnotateC(pose)
    persistent body; 

    if(isempty(body))
        body = bodyGraph();
    end
    body_world = pose.bToA()*body;
    plot(body_world (1,:), body_world (2,:), 'c-', 'Linewidth', 2);
    hold on;
end