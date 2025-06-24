% Esta función solo calcula las coordenadas, no dibuja nada. Es muy rápida.
function coords = calculateFrameworkCoords(q, E)
    % Entradas:
    % q: Vector columna de posiciones [x1; y1; z1; x2; y2; z2; ...]
    % E: Matriz de aristas/conexiones [nodo1, nodo2]

    lE = size(E, 1);
    nP = length(q) / 3;

    % Pre-alocamos los contenedores para las coordenadas
    line_coords = cell(lE, 1);
    point_coords = zeros(nP, 3);
    text_pos = zeros(nP, 3);

    % --- Cálculo de las coordenadas de las líneas ---
    for i = 1:lE
        % Índices de los nodos de inicio y fin de la arista
        idx1 = E(i, 1) * 3 - 2;
        idx2 = E(i, 2) * 3 - 2;
        
        % Coordenadas [x1, x2], [y1, y2], [z1, z2] para cada línea
        x_coords = [q(idx1), q(idx2)];
        y_coords = [q(idx1 + 1), q(idx2 + 1)];
        z_coords = [q(idx1 + 2), q(idx2 + 2)];
        
        line_coords{i} = {x_coords, y_coords, z_coords};
    end

    % --- Cálculo de las coordenadas de los puntos y textos (Vectorizado) ---
    % Esto es más eficiente que un bucle
    idx_x = 1:3:length(q);
    idx_y = 2:3:length(q);
    idx_z = 3:3:length(q);
    
    point_coords(:, 1) = q(idx_x);
    point_coords(:, 2) = q(idx_y);
    point_coords(:, 3) = q(idx_z);
    
    % Las posiciones del texto tienen un pequeño offset
    text_pos = point_coords + 0.2;

    % Empaquetamos todo en una estructura para una salida limpia
    coords.lines = line_coords;
    coords.points = point_coords;
    coords.text = text_pos;
end