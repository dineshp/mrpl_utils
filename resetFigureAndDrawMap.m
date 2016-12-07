function resetFigureAndDrawMap()
    clf;
    axis([-.1 12*8*.0254+.1 -.1 12*12*.0254+.1]);
    off = .0254*6;
    p1 = [0 ; 0];
    p2 = [ 12*8*.0254 ; 0];
    p3 = [0 ; 12*12*.0254 ];
    p4 = [12* 8*.0254; 12*12*.0254];
    lines_p1 = [p2-[off;0] p1+[0;off] p2+[0;off] p3+[off;0]];
    lines_p2 = [p1+[off;0] p3-[0;off] p4-[0;off] p4-[off;0]];
    for i = 1:length(lines_p1(1,:))
        hold on;

        plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    end

	%sail map

    %8 9 10 line points
    margin = 8; %inches
    p1 = [(7*12)*.0254 ; 12*.0254];
    p2 = [ 7*12*.0254 ; 5*12*.0254 ];

    %midfield
    p3 = [(0+margin)*0.0254 ; 6*12*.0254 ];
    p4 = [(8*12-margin)*.0254; 6*12*.0254];
    lines_p1 = [p2 p3];
    lines_p2 = [p1 p4];
    
    for i = 1:length(lines_p1(1,:))
        hold on;

        plot([lines_p1(1,i), lines_p2(1,i)],[lines_p1(2,i), lines_p2(2,i)], 'b-', 'Linewidth', 1, 'DisplayName', 'map');
    end
end