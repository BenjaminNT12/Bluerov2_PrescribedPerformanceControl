% Esta función se llama UNA VEZ para crear los objetos gráficos.
function handles = Framework3Dplot_Initial(q_initial, E)
    % 1. Calcular las coordenadas iniciales usando la nueva función de cálculo
    initial_coords = calculateFrameworkCoords(q_initial, E);
    
    lE = size(E, 1);
    nP = length(q_initial) / 3;

    % Pre-alocamos los arrays de handles
    line_h = gobjects(lE, 1);
    
    % 2. Crear los objetos gráficos
    
    % Crear las LÍNEAS una por una
    for i = 1:lE
        coords = initial_coords.lines{i};
        line_h(i) = line(coords{1}, coords{2}, coords{3}, ...
                         "Linewidth", 0.5, ...
                         'Color', '#241468', ...
                         'LineStyle', '--');
    end
    
    % Crear TODOS LOS PUNTOS con una sola llamada a scatter3 (más eficiente)
    points_h = scatter3(initial_coords.points(:,1), ...
                        initial_coords.points(:,2), ...
                        initial_coords.points(:,3), ...
                        'filled', 'SizeData', 90, ...
                        'MarkerFaceColor', '#77037B', ...
                        'MarkerEdgeColor', '#77037B');
    
    % Crear los TEXTOS uno por uno
    text_h = gobjects(nP, 1);
    for i = 1:nP
        pos = initial_coords.text(i, :);
        text_h(i) = text(pos(1), pos(2), pos(3), num2str(i), ...
                         'Color', '#02315E', ...
                         'FontWeight', 'bold', ...
                         'FontSize', 14);
    end
    
    % 3. Empaquetar los handles en una estructura para la salida
    handles.lines = line_h;
    handles.points = points_h;
    handles.text = text_h;
end