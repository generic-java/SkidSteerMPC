function xdot = get_state_derivs_linear(t, x, u, A, B)
    xdot = A * x + B * u;
end