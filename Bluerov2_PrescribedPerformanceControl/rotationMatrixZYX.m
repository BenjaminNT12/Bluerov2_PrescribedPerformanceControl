function R = rotationMatrixZYX(eta2)
    phi = eta2(1);  cph = cos(phi);  sph = sin(phi);
    th  = eta2(2);  cth = cos(th);   sth = sin(th);
    ps  = eta2(3);  cps = cos(ps);   sps = sin(ps);
    R = [ cps*cth,  -sps*cph + cps*sth*sph,   sps*sph + cps*sth*cph;
          sps*cth,   cps*cph + sps*sth*sph,  -cps*sph + sps*sth*cph;
          -sth,      cth*sph,                 cth*cph ];
end