function MP = BlueROV2ModelMatrices(nu_local, eta_local, params_local)
    % Renombrar componentes para evitar colisión de nombres con 'u' (entradas)
    u_sg = nu_local(1); v_sw = nu_local(2); w_hv = nu_local(3);
    p_rl = nu_local(4); q_pt = nu_local(5); r_yw = nu_local(6);
    nu1  = nu_local(1:3);
    nu2  = nu_local(4:6);

    S = @(v)[  0   -v(3)  v(2);
              v(3)   0   -v(1);
             -v(2)  v(1)   0 ];

    % Inercia rígida
    mass = params_local.m;
    r_g  = [params_local.xg; params_local.yg; params_local.zg];
    I_CG = [ params_local.Ixx, -params_local.Ixy, -params_local.Ixz; ...
            -params_local.Ixy,  params_local.Iyy, -params_local.Iyz; ...
            -params_local.Ixz, -params_local.Iyz,  params_local.Izz ];
    Srg  = [  0, -r_g(3),  r_g(2);
            r_g(3),   0,  -r_g(1);
           -r_g(2), r_g(1),  0    ];
    MRB = [ mass*eye(3), -mass*Srg;
            mass*Srg,     I_CG ];

    % Inercia añadida
    MA  = diag([params_local.X_u_dot, params_local.Y_v_dot, params_local.Z_w_dot, ...
                params_local.K_p_dot, params_local.M_q_dot, params_local.N_r_dot]);
    M   = MRB + MA;

    % Coriolis rígido
    CRB = [ zeros(3),    -mass*S(nu2);
           -mass*S(nu2), -S(I_CG*nu2) ];

    % Coriolis añadido (alfas/betas)
    MA11 = MA(1:3,1:3); MA12 = MA(1:3,4:6);
    MA21 = MA(4:6,1:3); MA22 = MA(4:6,4:6);
    alpha = MA11*nu1 + MA12*nu2;
    beta  = MA21*nu1 + MA22*nu2;
    CA = [ zeros(3),     -S(alpha);
           -S(alpha),    -S(beta) ];
    C = CRB + CA;

    % Amortiguamiento
    Dlin = diag([params_local.X_u, params_local.Y_v, params_local.Z_w, ...
                 params_local.K_p, params_local.M_q, params_local.N_r]);
    Dquad= diag([params_local.X_uu*abs(u_sg), params_local.Y_vv*abs(v_sw), params_local.Z_ww*abs(w_hv), ...
                 params_local.K_pp*abs(p_rl), params_local.M_qq*abs(q_pt), params_local.N_rr*abs(r_yw)]);
    D = Dlin + Dquad;

    % Restaurador
    phi = eta_local(4); theta = eta_local(5);
    gN  = mass * 9.81; B = gN * 1.0;
    xb = 0; yb = 0; zb = -0.01;

    g1 = [  (gN - B) * sin(theta);
           -(gN - B) * cos(theta) * sin(phi);
           -(gN - B) * cos(theta) * cos(phi) ];

    g2 = [ (yb*B)*cos(theta)*cos(phi) - (zb*B)*cos(theta)*sin(phi);
          -(zb*B)*sin(theta) - (xb*B)*cos(theta)*cos(phi);
           (xb*B)*cos(theta)*sin(phi) + (yb*B)*sin(theta) ];

    MP.M  = M;   MP.C  = C;   MP.D  = D;
    MP.g1 = g1;  MP.g2 = g2;
    MP.M11 = M(1:3, 1:3); MP.M12 = M(1:3, 4:6);
    MP.M21 = M(4:6, 1:3); MP.M22 = M(4:6, 4:6);
    MP.C11 = C(1:3, 1:3); MP.C12 = C(1:3, 4:6);
    MP.C21 = C(4:6, 1:3); MP.C22 = C(4:6, 4:6);
    MP.D11 = D(1:3, 1:3); MP.D12 = D(1:3, 4:6);
    MP.D21 = D(4:6, 1:3); MP.D22 = D(4:6, 4:6);
    MP.g1 = g1; MP.g2 = g2;
end