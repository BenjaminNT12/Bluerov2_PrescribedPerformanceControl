function [eta_dot, nu_dot] = modelFunction(t, X, tau, vehicleModel)
    % Extract states
    eta = X(1:6);   % Positions and orientations
    nu  = X(7:12);  % Linear and angular velocities
    
    % Compute derivatives using vehicle model
    [eta_dot, nu_dot] = vehicleModel.computeDerivatives(eta, nu, tau);
end
