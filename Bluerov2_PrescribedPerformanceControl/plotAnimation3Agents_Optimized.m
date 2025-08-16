function plotAnimation3Agents_Optimized(P, v0, t, p, E, f)
    % --- 1. Configuración Inicial de la Figura ---
    figure(f);       % Asegurarse que la figura f está activa
    clf;             % LIMPIA la figura para evitar objetos de ejecuciones anteriores
    % f.Position = [500 500 1000 1250];
    view([-45, -90, 45]);
    grid on;
    hold on;         % Activa hold ON y no lo desactives hasta el final

    % --- 2. Pre-cálculo de la Trayectoria ---
    trajectory_x = v0(1, :)';
    trajectory_y = v0(2, :)';
    trajectory_z = v0(3, :)';

    % --- 3. Inicialización de Objetos Gráficos ---
    
    % Inicializamos las animatedline para las trayectorias de los agentes
    h1 = animatedline('LineStyle', '-.', 'Color', 'white', 'LineWidth', 0.8);
    h2 = animatedline('LineStyle', '-.', 'Color', 'white', 'LineWidth', 0.8);
    h3 = animatedline('LineStyle', '-.', 'Color', '#0072BD', 'LineWidth', 1.5);
    trajectory = animatedline('LineStyle', '-.', 'Color', 'red', 'LineWidth', 1.5);

    % Inicializamos el Framework UNA SOLA VEZ usando la nueva función
    framework_handles = Framework3Dplot_Initial(p(:, 1), E);

    % Configuración de Títulos y Etiquetas (se hace una vez)
    title('Adquisición de la formación', 'FontSize', 20);
    xlabel('Eje-X', 'FontSize', 14);
    ylabel('Eje-Y', 'FontSize', 14);
    zlabel('Eje-Z', 'FontSize', 14);
    
    num_frames = length(t);
    % --- 4. Bucle de Animación Optimizado ---
    for i = 1:num_frames-1
        % Actualizar trayectorias de los agentes
        addpoints(h1, P(i, 1, 1), P(i, 1, 2), P(i, 1, 3));
        addpoints(h2, P(i, 2, 1), P(i, 2, 2), P(i, 2, 3));
        addpoints(h3, P(i, 3, 1), P(i, 3, 2), P(i, 3, 3));
        addpoints(trajectory, trajectory_x(i), trajectory_y(i), trajectory_z(i));

        % --- Actualización del Framework ---
        
        % 4.1. Obtener las nuevas coordenadas (muy rápido)
        new_coords = calculateFrameworkCoords(p(:, i), E);
        
        % 4.2. Actualizar los objetos existentes usando los handles (muy rápido)
        
        % Actualizar las líneas del framework
        for k = 1:length(framework_handles.lines)
            line_data = new_coords.lines{k};
            set(framework_handles.lines(k), ...
                'XData', line_data{1}, ...
                'YData', line_data{2}, ...
                'ZData', line_data{3});
        end
        
        % Actualizar los puntos del framework
        set(framework_handles.points, ...
            'XData', new_coords.points(:, 1), ...
            'YData', new_coords.points(:, 2), ...
            'ZData', new_coords.points(:, 3));
            
        % Actualizar los textos del framework
        for k = 1:length(framework_handles.text)
            set(framework_handles.text(k), 'Position', new_coords.text(k, :));
        end
        
        % Dibujar todos los cambios en la pantalla
        drawnow limitrate;
    end
    
    hold off;
end