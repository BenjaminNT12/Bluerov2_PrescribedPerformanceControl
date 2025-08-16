function plotControlInputs(u, SimulationParams, Framework)
    figure;
    agent_labels = arrayfun(@(i) sprintf('Agente %d', i), 1:Framework.NUM_AGENTS, 'UniformOutput', false);

    for i = 1:Framework.NUM_AGENTS
        % X axis control input
        subplot(3,1,1);
        plot(SimulationParams.TIME(1:end-1), u(i*Framework.SPACE_DIM-2, :), 'LineWidth', 2, 'DisplayName', agent_labels{i});
        hold on;
        ylim([-1000, 1000]);
        title('Entrada de Control $u_x$ para Cada Agente', 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Tiempo [s]', 'FontSize', 12);
        ylabel('$u_x$', 'Interpreter', 'latex', 'FontSize', 12);
        grid on;

        % Y axis control input
        subplot(3,1,2);
        plot(SimulationParams.TIME(1:end-1), u(i*Framework.SPACE_DIM-1, :), 'LineWidth', 2, 'DisplayName', agent_labels{i});
        hold on;
        ylim([-1000, 2000]);
        title('Entrada de Control $u_y$ para Cada Agente', 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Tiempo [s]', 'FontSize', 12);
        ylabel('$u_y$', 'Interpreter', 'latex', 'FontSize', 12);
        grid on;

        % Z axis control input
        subplot(3,1,3);
        plot(SimulationParams.TIME(1:end-1), u(i*Framework.SPACE_DIM, :), 'LineWidth', 2, 'DisplayName', agent_labels{i});
        hold on;
        ylim([-1000, 1000]);
        title('Entrada de Control $u_z$ para Cada Agente', 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Tiempo [s]', 'FontSize', 12);
        ylabel('$u_z$', 'Interpreter', 'latex', 'FontSize', 12);
        grid on;
    end

    % Add legends to each subplot
    subplot(3,1,1); legend('show', 'Location', 'best');
    subplot(3,1,2); legend('show', 'Location', 'best');
    subplot(3,1,3); legend('show', 'Location', 'best');
end