classdef VehicleModel
    properties
        params
    end
    methods
        function obj = VehicleModel(params)
            obj.params = params;
        end

        function [eta_dot, nu_dot] = computeDerivatives(obj, eta, nu, tau)
            % Extract parameters from obj.params
            M11 = obj.params.M11; % 3x3
            M12 = obj.params.M12; % 3x3
            M21 = obj.params.M21; % 3x3
            M22 = obj.params.M22; % 3x3
            
            C11 = obj.params.C11; % 3x3
            C12 = obj.params.C12; % 3x3
            C21 = obj.params.C21; % 3x3
            C22 = obj.params.C22; % 3x3
            
            D11 = obj.params.D11; % 3x3
            D22 = obj.params.D22; % 3x3
            
            g1  = obj.params.g1;  % 3x1
            g2  = obj.params.g2;  % 3x1

            % Extract states
            eta1 = eta(1:3);   % [x, y, z]
            eta2 = eta(4:6);   % [phi, theta, psi]
            nu1  = nu(1:3);    % [u, v, w]
            nu2  = nu(4:6);    % [p, q, r]

            tau1 = tau(1:3);
            tau2 = tau(4:6);

            % Construct the full matrices
            M = [M11 M12;
                 M21 M22];

            C = [C11 C12;
                 C21 C22];

            D = blkdiag(D11, D22);

            g = [g1; g2];    % 6x1 vector

            % Compute transformation matrices
            R_n_b = rotationMatrix(eta2);  % 3x3 rotation for linear motion
            T_a   = transformationMatrix(eta2); % 3x3 transformation for angular motion

            % Form the full J(eta)
            % J(eta) = [ R_n_b   0
            %             0       T_a ]
            J = [R_n_b, zeros(3,3);
                 zeros(3,3), T_a];

            % Compute eta_dot
            
            % Compute nu_dot
            % nu_dot = M^{-1}(tau - C nu - D nu - g)
            % nu_combined  = [nu1;   nu2];   % 6x1
            % tau_combined = [tau1; tau2];% 6x1
            M_inv = inv(M);
            
            eta_dot = J * nu;  % 6x1
            nu_dot = M_inv * (tau - C * nu - D * nu - g);
        end
    end
end


function R_n_b = rotationMatrix(eta2)
    % eta2 = [phi, theta, psi]
    phi = eta2(1); theta = eta2(2); psi = eta2(3);
    
    cphi = cos(phi); sphi = sin(phi);
    ctheta = cos(theta); stheta = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    R_n_b = [
        cpsi*ctheta, -spsi*cphi + cpsi*stheta*sphi, spsi*sphi + cpsi*stheta*cphi;
        spsi*ctheta, cpsi*cphi + spsi*stheta*sphi, -cpsi*sphi + spsi*stheta*cphi;
        -stheta,      ctheta*sphi,                 ctheta*cphi
    ];
end

% function T_a = transformationMatrix(eta2)
%     % eta2 = [phi, theta, psi]
%     phi = eta2(1); theta = eta2(2);
    
%     T_a = [
%         1, sphi_tan_theta(phi,theta), cphi_tan_theta(phi,theta);
%         0, cos(phi),                -sin(phi);
%         0, sin(phi)/cos(theta),      cos(phi)/cos(theta)
%     ];

%     function val = sphi_tan_theta(phi,theta)
%         val = sin(phi)*tan(theta);
%     end
%     function val = cphi_tan_theta(phi,theta)
%         val = cos(phi)*tan(theta);
%     end
% end

function T_a = transformationMatrix(eta2)
    phi   = eta2(1);
    theta = eta2(2);

    % calcula cos(theta) de forma normal
    ct = cos(theta);
    st = sin(theta);

    % calcula tan(theta) y luego satúralo
    tan_theta = st/ct;
    tan_max   = 10;               % ajusta según rango de operación
    tan_theta = max(min(tan_theta, tan_max), -tan_max);

    % y la secante solo hasta el mismo ratio
    sec_theta = sqrt(1 + tan_theta^2);

    T_a = [ ...
      1,           sin(phi)*tan_theta,    cos(phi)*tan_theta; 
      0,           cos(phi),             -sin(phi); 
      0,           sin(phi)*sec_theta,    cos(phi)*sec_theta ...
    ];
end