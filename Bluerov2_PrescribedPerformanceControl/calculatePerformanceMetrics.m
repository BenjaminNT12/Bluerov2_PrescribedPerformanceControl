function [avgFormationError, avgTrackingError, totalControlEffort, avgControlEffort] = calculatePerformanceMetrics(t, edgeError, V, vds, u, T, m, n)
    % calculatePerformanceMetrics calcula los indicadores de desempeño para el sistema multiagente.
    %
    % Entradas:
    %   t          - Vector de tiempo [1 x num_steps]
    %   edgeError  - Matriz de errores por arista [num_edges x num_steps]
    %   V          - Matriz de velocidades [m*n x num_steps]
    %   vds        - Matriz de velocidades deseadas [3 x num_steps]
    %   u          - Matriz de entradas de control [m*n x num_steps]
    %   T          - Tiempo de muestreo
    %   m          - Dimensión del espacio (ej. 3)
    %   n          - Número de agentes
    %
    % Salidas:
    %   avgFormationError  - Error promedio de formación (escalares promediados en el tiempo)
    %   avgTrackingError   - Error promedio de seguimiento de trayectoria (norma promedio entre velocidad real y deseada)
    %   totalControlEffort - Esfuerzo total de control (integración de la energía aplicada)
    %   avgControlEffort   - Suavidad de la entrada de control (variación total promedio por agente)
    %
    % La función también genera una figura con subplots que muestran la evolución temporal de cada indicador.
    %
    % Dr. en Control Automático – Investigación y Desarrollo
    
        num_steps = length(t);
    
        %% 1) Cálculo del Error Promedio de Formación
        % Se promedian los errores absolutos en todas las aristas para cada instante.
        formationError_time = mean(abs(edgeError), 1);  % Promedio por instante
        avgFormationError = mean(formationError_time);
        fprintf('Average Formation Error: %f\n', avgFormationError);
        
        %% 2) Cálculo del Error de Seguimiento de Trayectoria
        % Para cada instante se evalúa la norma de la diferencia entre la velocidad de cada agente y
        % la velocidad deseada, promediándola entre los n agentes.
        trackingError_time = zeros(1, num_steps);
        for kk = 1:num_steps
            error_sum = 0;
            for j = 1:n
                idx = (j-1)*m + (1:m);
                error_sum = error_sum + norm(V(idx, kk) - vds(:, kk));
            end
            trackingError_time(kk) = error_sum / n;
        end
        avgTrackingError = mean(trackingError_time);
        fprintf('Average Trajectory Tracking Error: %f\n', avgTrackingError);
        
        %% 3) Cálculo del Esfuerzo de Control
        % Se estima el esfuerzo total integrando el cuadrado de las entradas de control.
        totalControlEffort = sum(u(:).^2) * T;
        fprintf('Total Control Effort: %f\n', totalControlEffort);
        
        %% 4) Cálculo de la Suavidad de la Entrada de Control (Variación Total)
        % Para cada agente, se suma la norma de la diferencia entre valores consecutivos de la señal.
        controlSmoothness_per_agent = zeros(n, 1);
        for j = 1:n
            idx = (j-1)*m + (1:m);
            TV_agent = 0;
            for kk = 1:(num_steps-1)
                TV_agent = TV_agent + norm(u(idx, kk+1) - u(idx, kk));
            end
            controlSmoothness_per_agent(j) = TV_agent;
        end
        avgControlEffort = mean(controlSmoothness_per_agent);
        fprintf('Average Control Input Smoothness (Total Variation) per Agent: %f\n', avgControlEffort);
        
        % (Opcional) Cálculo de la evolución temporal de la variación total para cada agente
        controlSmoothness_time = zeros(n, num_steps-1);
        for j = 1:n
            idx = (j-1)*m + (1:m);
            for kk = 1:(num_steps-1)
                controlSmoothness_time(j, kk) = norm(u(idx, kk+1) - u(idx, kk));
            end
        end
        
       %% Gráficos combinados en una única ventana (Subplots)
        figure;

        % Subplot 1: Error de Formación
        subplot(3,1,1);
        plot(t, formationError_time, 'LineWidth', 2);
        grid on;
        xlabel('Time [s]');
        ylabel('[m]');
        title('Error promedio de formación');
        set(gca, 'FontSize', 12); % Added for consistency

        % Subplot 2: Error de Seguimiento de Trayectoria
        subplot(3,1,2);
        plot(t, trackingError_time, 'LineWidth', 2);
        grid on;
        xlabel('Time [s]');
        ylabel('[m/s]');
        title('Error de seguimiento de trayectoria');
        set(gca, 'FontSize', 12); % Added for consistency

        % Subplot 3: Suavidad de la Entrada de Control (Variación Total)
        subplot(3,1,3);
        plot(t(2:end), mean(controlSmoothness_time, 1), 'LineWidth', 2); % Using consistent var name
        grid on;
        xlabel('Time [s]');
        ylabel('[Newtons/s]'); % Adjusted unit based on u being in Newtons
        title('Entrada de control');
        set(gca, 'FontSize', 12); % Added for consistency
 
        
    end