function R = rotationMatrix(phi, theta, psi)
    % rotationMatrix: Calcula la matriz de rotación 3x3 correspondiente
    % a los ángulos de Euler: roll (phi), pitch (theta) y yaw (psi).
    %
    % Entrada:
    %   phi   - ángulo de rotación en torno al eje x (roll)
    %   theta - ángulo de rotación en torno al eje y (pitch)
    %   psi   - ángulo de rotación en torno al eje z (yaw)
    %
    % Salida:
    %   R     - matriz de rotación 3x3
    
    R = [ cos(psi)*cos(theta), ...
          -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi), ...
           sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
          sin(psi)*cos(theta), ...
           cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi), ...
          -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
         -sin(theta), ...
           cos(theta)*sin(phi), ...
           cos(theta)*cos(phi) ];
end