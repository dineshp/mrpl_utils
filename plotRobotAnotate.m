function plotRobotAnotate(pose, laserPts)
    persistent body; 
    persistent t;
    persistent signal;
    
    if(isempty(body))
        body = bodyGraph();
    end
    
    if(isempty(t))
        t = text(0,0,'origin', 'FontSize',10);
        t.Color = 'k';
        hold on;
    end
    
    if(isempty(signal))
        signal = scatter(0, 0, 'r', 'filled');
    end
    
    body_world = pose.bToA()*body;
    %plot(body_world (1,:), body_world (2,:), 'm-', 'Linewidth', 3);
    txt = sprintf('[%0.2f, %0.2f, %0.2f]', pose.x, pose.y, rad2deg(pose.th));
    txt = strcat('\leftarrow', txt); 
    delete(t);
    
    t = text(body_world(1,1),body_world(2,1),txt,'FontSize',10);
    t.Color = 'k';
    hold on;
    
    delete(signal);
    laserPts_world = pose.bToA()*laserPts;
    signal = scatter(laserPts_world(1,:), laserPts_world(2,:), 'r', 'filled');
    hold on;
end