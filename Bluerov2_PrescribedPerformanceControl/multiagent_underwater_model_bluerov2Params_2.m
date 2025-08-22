function dxdt = multiagent_underwater_model_bluerov2Params_2(t, X, u, m, n, modelParameters)
% Estado por agente i (12): [eta1(3); eta2(3); nu1(3); nu2(3)]
% Entradas:
%   t: tiempo
%   X: [12*n x 1]
%   u: [6*n x 1]  (tau por agente)
%   m: dimension espacial (=3)
%   n: numero de agentes
%   modelParameters: struct opcional (ver getBaseParams)

% ---- validaciones y saneo ----
if m ~= 3, error('Este modelo está definido para m=3.'); end
if numel(X) ~= 12*n
    error('X debe ser de tamaño 12*n. Recibido %d (n=%d).', numel(X), n);
end

% forzar a vector columna y validar longitud
u = u(:);
if numel(u) ~= 6*n
    error('u debe tener longitud 6*n. Recibido %d, esperado %d.', numel(u), 6*n);
end

dxdt = zeros(12*n,1);
idx12 = @(i) (12*(i-1)+1) : 12*i;
idx6  = @(i) ( 6*(i-1)+1) :  6*i;

for i = 1:n
    Xi   = X(idx12(i));
    taui = u(idx6(i));             % [6x1] para el agente i

    eta  = Xi(1:6);                % [x y z phi theta psi]
    nu   = Xi(7:12);               % [u v w p q r]
    eta2 = eta(4:6);

    % Cinemática
    Rnb  = rotationMatrixZYX(eta2);
    Ta   = transformationMatrixEuler(eta2);
    J    = blkdiag(Rnb, Ta);
    eta_dot = J * nu;

    % Dinámica
    baseParams = getBaseParams(modelParameters, i);
    MP = BlueROV2ModelMatrices(nu, eta, baseParams);

    rhs   = taui - MP.C*nu - MP.D*nu - [MP.g1; MP.g2];
    nu_dot = MP.M \ rhs;

    dxdt(idx12(i)) = [eta_dot; nu_dot];
end
end
