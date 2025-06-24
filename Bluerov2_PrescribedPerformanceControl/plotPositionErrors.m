function plotPositionErrors(agentIndex, q, qd, m, t)
    % plotPositionErrors: Grafica los errores de posición en los ejes X, Y y Z para un agente específico.
    %
    % Entradas:
    %   agentIndex - Índice del agente (ejemplo: 3 para el agente 3)
    %   q - Matriz de posiciones actuales de los agentes [(m * n) x num_steps]
    %   qd - Matriz de posiciones deseadas [(m * n) x 1] (posición deseada inicial)
    %   m - Dimensión del espacio (ejemplo: 3 para 3D)
    %   t - Vector de tiempo [1 x num_steps]
    %
    % Salida:
    %   Gráficos de errores de posición en los ejes X, Y y Z.

    % Índices para las posiciones del agente específico
    idx_agent = agentIndex * m - 2 : agentIndex * m;
    

    % Calcular los errores de posición para cada eje
    errorX = q(idx_agent(1),:) - qd(1,:);
    errorY = q(idx_agent(2),:) - qd(2,:);
    errorZ = q(idx_agent(3),:) - qd(3,:);

    % Crear la figura
    figure('Name', sprintf('Errores de Posición del Agente %d', agentIndex));

    % Subplot para el error en el eje X
    subplot(3, 1, 1);
    plot(t, errorX, 'LineWidth', 2);
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 12);
    ylabel('Error X [m]', 'FontSize', 12);
    ylim([-20 20]); % Ajustar límites del eje Y
    title(sprintf('Error de Posición en el Eje X (Agente %d)', agentIndex), 'FontSize', 14);

    % Subplot para el error en el eje Y
    subplot(3, 1, 2);
    plot(t, errorY, 'LineWidth', 2);
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 12);
    ylabel('Error Y [m]', 'FontSize', 12);
    ylim([-20 20]); % Ajustar límites del eje Y
    title(sprintf('Error de Posición en el Eje Y (Agente %d)', agentIndex), 'FontSize', 14);

    % Subplot para el error en el eje Z
    subplot(3, 1, 3);
    plot(t, errorZ, 'LineWidth', 2);
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 12);
    ylabel('Error Z [m]', 'FontSize', 12);
    ylim([-20 20]); % Ajustar límites del eje Y
    title(sprintf('Error de Posición en el Eje Z (Agente %d)', agentIndex), 'FontSize', 14);
end
