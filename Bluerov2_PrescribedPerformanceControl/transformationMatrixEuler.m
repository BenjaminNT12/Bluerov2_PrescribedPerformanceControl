function T = transformationMatrixEuler(eta2)
    phi = eta2(1); th = eta2(2);
    cth = cos(th); sth = sin(th);
    cth = sign(cth) * max(abs(cth), 1e-3);      % evita singularidad
    tanth = min(max(sth/cth, -10), 10);
    sech  = 1/cth;
    cph = cos(phi); sph = sin(phi);
    T = [ 1,       sph*tanth,  cph*tanth;
          0,       cph,        -sph;
          0,       sph*sech,   cph*sech ];
end