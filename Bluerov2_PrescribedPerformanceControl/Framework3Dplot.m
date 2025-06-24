function [lines, points, num] = Framework3Dplot(q, E)
    lE = length(E);
    nP = length(q(:,1))/3;

    Pij = zeros(lE,6);
    lines = zeros(lE,1);
    points = zeros(nP,1);
    num = zeros(nP,1);

    hold on
    for i = 1: lE
        Pij(i,:) = [q(E(i,1)*3-2), q(E(i,2)*3-2), q(E(i,1)*3-1), q(E(i,2)*3-1), q(E(i,1)*3), q(E(i,2)*3)];
        lines(i) = line(Pij(i, 1:2), Pij(i, 3:4), Pij(i, 5:6),"Linewidth",0.5,'Color','#241468','LineStyle','--','HandleVisibility', 'off');
        if i <= nP
            points(i) = scatter3(q(i*3-2),q(i*3-1),q(i*3),'filled','SizeData',90, 'MarkerFaceColor', '#77037B', 'MarkerEdgeColor', '#77037B','HandleVisibility', 'off');

            num(i) = text(q(i*3-2)+0.2,q(i*3-1)+0.2,q(i*3)+8,num2str(i),'Color','#02315E','FontWeight','bold','FontSize',14);
        end
    end
    hold off
    set(gca, 'FontSize', 14);
    xlabel('Eje-X [m]', 'FontSize', 14);
    ylabel('Eje-Y [m]', 'FontSize', 14);
    zlabel('Eje-Z [m]', 'FontSize', 14);
    grid on
end


