function modelParameters = BlueROV2ModelParameters(nu, eta)
% BlueROV2ModelParameters: ROV 6DOF model using CRB skew-form (Fossen eq. 3.63)
%
% Inputs:
%   nu  : [6x1] vector [u; v; w; p; q; r] (body linear and angular velocity)
%   eta : [6x1] vector [x; y; z; phi; theta; psi] (pose)
%
% Output:
%   modelParameters : struct with Mij, Cij, Dij, gi blocks

% === Parameters from BlueROV2 paper Table A1 ===
    params = struct( ...
        'm',       13.5, ...
        'xg',      0.0,  'yg', 0.0,  'zg', 0.0, ...
        'Ixx',     0.26, 'Iyy', 0.23, 'Izz', 0.37, ...
        'Ixy',     0.0,  'Ixz', 0.0,  'Iyz', 0.0, ...
        'X_u_dot', 6.36, 'Y_v_dot', 7.12, 'Z_w_dot', 18.68, ...
        'K_p_dot', 0.189, 'M_q_dot', 0.135, 'N_r_dot', 0.222, ...
        'X_u',     13.7, 'Y_v', 0.0, 'Z_w', 33.0, ...
        'K_p',     0.0,   'M_q', 0.8, 'N_r', 0.0, ...
        'X_uu',    141.0, 'Y_vv', 217.0, 'Z_ww', 190.0, ...
        'K_pp',    1.19,  'M_qq', 0.47, 'N_rr', 1.15 ...
    );

    % === Inertia terms ===
    m  = params.m;
    r_g = [params.xg; params.yg; params.zg];
    I_CG = [params.Ixx, -params.Ixy, -params.Ixz;
            -params.Ixy, params.Iyy, -params.Iyz;
            -params.Ixz, -params.Iyz, params.Izz];

    % === Body velocities ===
    u = nu(1); v = nu(2); w = nu(3);
    p = nu(4); q = nu(5); r = nu(6);
    nu_1 = [u; v; w];
    nu_2 = [p; q; r];

    % === Mass matrix ===
    S_rg = [ 0 -r_g(3) r_g(2);
            r_g(3) 0 -r_g(1);
            -r_g(2) r_g(1) 0 ];
    MRB = [ m*eye(3), -m*S_rg;
            m*S_rg,    I_CG];

    MA = diag([params.X_u_dot, params.Y_v_dot, params.Z_w_dot, ...
                params.K_p_dot, params.M_q_dot, params.N_r_dot]);
    M = MRB + MA;

    modelParameters.M11 = M(1:3,1:3);
    modelParameters.M12 = M(1:3,4:6);
    modelParameters.M21 = M(4:6,1:3);
    modelParameters.M22 = M(4:6,4:6);

    % === Coriolis matrix using skew-form ===
    S = @(v)[     0, -v(3),  v(2); 
               v(3),     0, -v(1); 
              -v(2),  v(1),     0];


    S_nu_2 = S(nu_2);
    T12 = m * S_nu_2;
    T21 = -m * S_nu_2;
    T22 = [                 0,              -params.Izz*r,                  -params.Iyy*q;
                 params.Izz*r,                          0,                  -params.Ixx*p;
                 params.Iyy*q,              -params.Ixx*p,                              0];

    CRB = [zeros(3), T12; T21, T22];

    % === Hydrodynamic Coriolis CA (simplified diagonal only) ===
    CA_12 = [               0,           -params.Z_w_dot*w,              -params.Y_v_dot*v;
             params.Z_w_dot*w,                           0,              -params.X_u_dot*u;
            -params.Y_v_dot*v,            params.X_u_dot*u,                             0];

    CA_22 = [               0,           -params.N_r_dot*r,              -params.M_q_dot*q;
             params.N_r_dot*r,                           0,              -params.K_p_dot*p;
            -params.M_q_dot*q,            params.K_p_dot*p,                             0];
    
    CA = [zeros(3), CA_12; 
             CA_12, CA_22];

    C = CRB + CA;

    modelParameters.C11 = C(1:3,1:3);
    modelParameters.C12 = C(1:3,4:6);
    modelParameters.C21 = C(4:6,1:3);
    modelParameters.C22 = C(4:6,4:6);

    % === Damping (linear + quadratic, negative definite) ===
    D_L = diag([params.X_u, params.Y_v, params.Z_w, ...
                params.K_p, params.M_q, params.N_r]);

    D_Q = diag([params.X_uu*abs(u), params.Y_vv*abs(v), params.Z_ww*abs(w), ...
                params.K_pp*abs(p), params.M_qq*abs(q), params.N_rr*abs(r)]);
    D = D_L + D_Q;

    modelParameters.D11 = D(1:3,1:3);
    modelParameters.D12 = D(1:3,4:6);
    modelParameters.D21 = D(4:6,1:3);
    modelParameters.D22 = D(4:6,4:6);

    % === Restoring forces ===
    phi = eta(4); theta = eta(5);
    g = m * 9.81;
%     B = g * 0.90;  % postive buoyancy
%     B = g * 1.10;  % negative buoyancy
    B = g * 1.00;  % negative buoyancy
    
    xb = 0; yb = 0; zb = -0.001;

    g1 = [   (g-B) * sin(theta);
            -(g-B) * cos(theta) * sin(phi);
            -(g-B) * cos(theta) * cos(phi)  ];

    g2 = [ (yb*B) * cos(theta) * cos(phi) - (zb*B) * cos(theta)*sin(phi);
          -(zb*B) * sin(theta) - (xb*B) * cos(theta) * cos(phi);
           (xb*B) * cos(theta) * sin(phi) + (yb*B) * sin(theta)         ];

    g_vector = [g1; g2];
         

    modelParameters.g1 = g1;
    modelParameters.g2 = g2;
end
