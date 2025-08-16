function plotAgentVsLeaderPosition( leaderPos, agentPosArray, spaceDim, timeVec)
    figure;
    subplot(3,1,1);
    plot(timeVec, leaderPos(1, :),'--' ,'LineWidth', 2, 'DisplayName', 'Desired X');
    hold on;
    plot(timeVec, agentPosArray(3 * spaceDim - 2, :), 'LineWidth', 2, 'DisplayName', 'Agent 3 X');
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 12);
    ylabel('X [m]', 'FontSize', 12);
    title('Posición X', 'FontSize', 14);
    legend show;

    subplot(3,1,2);
    plot(timeVec, leaderPos(2, :),'--', 'LineWidth', 2, 'DisplayName', 'Desired Y');
    hold on;
    plot(timeVec, agentPosArray(3 * spaceDim - 1, :), 'LineWidth', 2, 'DisplayName', 'Agent 3 Y');
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 12);
    ylabel('Y [m]', 'FontSize', 12);
    title('Posición Y', 'FontSize', 14);
    legend show;

    subplot(3,1,3);
    plot(timeVec, leaderPos(3, :),'--', 'LineWidth', 2, 'DisplayName', 'Desired Z');
    hold on;
    plot(timeVec, agentPosArray(3 * spaceDim, :), 'LineWidth', 2, 'DisplayName', 'Agent 3 Z');
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 12);
    ylabel('Z [m]', 'FontSize', 12);
    title('Posición Z', 'FontSize', 14);
    legend show;


    figure;
    plot3(leaderPos(1, :), leaderPos(2, :), leaderPos(3, :), '--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');
    hold on;
    plot3(agentPosArray(3 * spaceDim - 2, :), agentPosArray(3 * spaceDim - 1, :), agentPosArray(3 * spaceDim, :), 'LineWidth', 2, 'DisplayName', 'Agent 3 Trajectory');
    grid on;
    xlabel('X [m]', 'FontSize', 12);
    ylabel('Y [m]', 'FontSize', 12);
    zlabel('Z [m]', 'FontSize', 12);
    title('Trayectoria en 3D', 'FontSize', 14);
    legend show;
    view(3);


end
