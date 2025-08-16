function plotOrientationTracking(X, TrajectoryDefVelocity, Framework, SimulationParams)
    figure;
    desired_trajectory = zeros(3, length(SimulationParams.TIME)-1);

    % Compute the desired orientation trajectory for all time steps
    for i = 1:length(SimulationParams.TIME)-1
        desired_trajectory(:, i) = TrajectoryDefVelocity.eta2_desired(SimulationParams.TIME(i));
    end

    % Loop through each agent to plot orientation tracking
    for agent_idx = 1:Framework.NUM_AGENTS
        % Extract orientation states (φ, θ, ψ) for the current agent
        start_idx = (agent_idx - 1) * Framework.STATES_PER_VEHICLE + 4;
        phi   = X(start_idx,   1:end-1); % Roll
        theta = X(start_idx+1, 1:end-1); % Pitch
        psi   = X(start_idx+2, 1:end-1); % Yaw

        time_vec = SimulationParams.TIME(1:end-1);

        % Plot Roll (φ) Tracking
        subplot(Framework.NUM_AGENTS, 3, (agent_idx - 1) * 3 + 1);
        plot(time_vec, phi, 'LineWidth', 2, 'DisplayName', 'Actual');
        hold on;
        plot(time_vec, desired_trajectory(1, :), 'LineWidth', 2, 'DisplayName', 'Desired');
        grid on;
        ylim([-0.5, 0.5]);
        xlabel('Tiempo [Seg]', 'FontSize', 18);
        ylabel(['$\phi_{', num2str(agent_idx), '}$ [rad]'], 'Interpreter', 'latex', 'FontSize', 18);
        title(['Agent ', num2str(agent_idx), ' Roll'], 'Interpreter', 'latex', 'FontSize', 18);
        legend('show');

        % Plot Pitch (θ) Tracking
        subplot(Framework.NUM_AGENTS, 3, (agent_idx - 1) * 3 + 2);
        plot(time_vec, theta, 'LineWidth', 2, 'DisplayName', 'Actual');
        hold on;
        plot(time_vec, desired_trajectory(2, :), 'LineWidth', 2, 'DisplayName', 'Desired');
        grid on;
        ylim([-0.5, 0.5]);
        xlabel('Tiempo [Seg]', 'FontSize', 18);
        ylabel(['$\theta_{', num2str(agent_idx), '}$ [rad]'], 'Interpreter', 'latex', 'FontSize', 18);
        title(['Agent ', num2str(agent_idx), ' Pitch'], 'Interpreter', 'latex', 'FontSize', 18);
        legend('show');

        % Plot Yaw (ψ) Tracking
        subplot(Framework.NUM_AGENTS, 3, (agent_idx - 1) * 3 + 3);
        plot(time_vec, psi, 'LineWidth', 2, 'DisplayName', 'Actual');
        hold on;
        plot(time_vec, desired_trajectory(3, :), 'LineWidth', 2, 'DisplayName', 'Desired');
        grid on;
        ylim([-0.5, 0.5]);
        xlabel('Tiempo [Seg]', 'FontSize', 18);
        ylabel(['$\psi_{', num2str(agent_idx), '}$ [rad]'], 'Interpreter', 'latex', 'FontSize', 18);
        title(['Agent ', num2str(agent_idx), ' Yaw'], 'Interpreter', 'latex', 'FontSize', 18);
        legend('show');
    end
end
