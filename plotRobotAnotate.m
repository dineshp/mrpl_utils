function plotRobotAnotate(pose)
    persistent body; 

    if(isempty(body))
        body = bodyGraph();
    end
    body_world = pose.bToA()*body;
    plot(body_world (1,:), body_world (2,:), 'c-', 'Linewidth', 2);
    txt = sprintf('[%0.2f, %0.2f, %0.2f]', pose.x, pose.y, rad2deg(pose.th));
    txt = strcat('\leftarrow', txt); 
    t = text(body_world(1,1),body_world(2,1),txt,'FontSize',5);
    t.Color = 'c';
    hold on;
    
end