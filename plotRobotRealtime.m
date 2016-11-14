function plotRobotRealtime(pose)
    persistent body;
    persistent signal;

    if(isempty(body))
        body = bodyGraph();
    end

    if(isempty(signal))
        signal = plot(0, 0, 'c-', 'Linewidth', 3);
    end

    body_world = pose.bToA()*body;
    delete(signal);
    signal = plot(body_world(1,:), body_world(2,:), 'c-', 'Linewidth', 3);
    hold on;
    pause(.01);
end

