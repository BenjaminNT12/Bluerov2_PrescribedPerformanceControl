function plotPrescribedPerformanceBounds(ErrorSignals, EdgesFormation, Framework, SimulationParams)
    temp = 0; % Flag to track if any error exceeds its bounds
    once_time = 0; % Flag to ensure plots are only created once

    for i = 1:Framework.NUM_EDGES
        % Select figure and legend label based on edge index
        if ismember(i, [1])
            %-------------------------------
            % Plot for Edge 1 (Figure 33)
            %-------------------------------
            figure(33);
            % Plot upper performance bound (e_plus)
            plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e_plus(i, :), ...
                'Color', 'r', 'LineStyle', '--', 'DisplayName', 'Cota de Rendimiento Superior-Inferior', 'LineWidth', 1.5);
            hold on;
            % Plot lower performance bound (-e_minus)
            plot(SimulationParams.TIME(1:end - 1), -ErrorSignals.e_minus(i, :), ...
                'Color', 'r', 'LineStyle', '--', 'HandleVisibility', 'off', 'LineWidth', 1.5);
            hold on;
            % Plot actual edge error (e)
            % Get agent indices for this edge
            agent_i = EdgesFormation.E_ARRAY(i,2);
            agent_j = EdgesFormation.E_ARRAY(i,1);
            plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e(i, :), ...
                'DisplayName', ['Error entre vehículos (', num2str(agent_i), '-', num2str(agent_j), ')'], 'LineWidth', 1.5);
            hold on;
            grid on;
            xlabel('Tiempo [Seg]', 'FontSize', 18);
            ylabel('Error $e_{ij}$', 'Interpreter', 'latex', 'FontSize', 18);
            legend('show');
            % Check if error exceeds bounds
            if any(ErrorSignals.e(i, :) > ErrorSignals.e_plus(i, :) | ...
                   ErrorSignals.e(i, :) < -ErrorSignals.e_minus(i, :)) && temp ~= i
                temp = i;
            end
        end

        if ismember(i, [2, 3])
            %-------------------------------
            % Plot for Edges 2 and 3 (Figure 34)
            %-------------------------------
            figure(34);
            if once_time == 0
                % Create a new figure for edges 2 and 3
                % Plot upper performance bound (e_plus)
                plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e_plus(i, :), ...
                    'Color', 'r', 'LineStyle', '--', 'DisplayName', 'Cota de Rendimiento Superior-Inferior','LineWidth', 1.5);
                hold on;
                % Plot lower performance bound (-e_minus)
                plot(SimulationParams.TIME(1:end - 1), -ErrorSignals.e_minus(i, :), ...
                    'Color', 'r', 'LineStyle', '--', 'HandleVisibility', 'off', 'LineWidth', 1.5);
                hold on;
                once_time = 1; % Set flag to indicate figure has been created
            end
            % Plot actual edge error (e)
            % Get agent indices for this edge
            agent_i = EdgesFormation.E_ARRAY(i,2);
            agent_j = EdgesFormation.E_ARRAY(i,1);
            plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e(i, :), ...
                'DisplayName', ['Error entre vehículos (', num2str(agent_i), '-', num2str(agent_j), ')'], 'LineWidth', 1.5);
            hold on;
            grid on;
            xlabel('Tiempo [Seg]', 'FontSize', 18);
            ylabel('Error $e_{ij}$', 'Interpreter', 'latex', 'FontSize', 18);
            if i == 2
                legend('show');
            end
            % Check if error exceeds bounds
            if any(ErrorSignals.e(i, :) > ErrorSignals.e_plus(i, :) | ...
                   ErrorSignals.e(i, :) < -ErrorSignals.e_minus(i, :)) && temp ~= i
                temp = i;
            end
        end
    end
end
