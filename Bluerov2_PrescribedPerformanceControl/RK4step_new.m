function x_next = RK4step_new(f, t, x, h, u, m, n, modelParameters)
    k1 = f(t, x, u, m, n, modelParameters);
    k2 = f(t + h/2, x + h/2 * k1, u, m, n, modelParameters);
    k3 = f(t + h/2, x + h/2 * k2, u, m, n, modelParameters);    
    k4 = f(t + h, x + h * k3, u, m, n, modelParameters);
    x_next = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
end