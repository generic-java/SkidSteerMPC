function xdot = get_state_derivs_nl(t, x, u, params)
    a = params.a;
    b = params.b;
    c = params.c;
    k = params.k;
    mu_sc = params.mu_sci;
    mu_lc = params.mu_lci;
    m = params.m;
    I = params.I;
    g = params.g;
    r = params.wheel_rad;
    cd = params.cd;
    bd = params.bd;

    theta = x(3);
    Xdot = x(4);
    Ydot = x(5);
    thetadot = x(6);
    tau_L = u(1);
    tau_R = u(2);

    xdot = zeros(6, 1);

    xdot(1) = Xdot;
    xdot(2) = Ydot;
    xdot(3) = thetadot;
    xdot(4) = -(pi*Xdot*cd*r - pi*tau_R*cos(theta) - pi*tau_L*cos(theta) + 4*g*m*mu_sc*r*cos(theta)*(atan(k*(c*thetadot + Xdot*cos(theta) + Ydot*sin(theta))) + atan(k*(Xdot*cos(theta) - c*thetadot + Ydot*sin(theta)))) + 4*g*m*mu_lc*r*sin(theta)*(atan(k*(a*thetadot - Ydot*cos(theta) + Xdot*sin(theta))) - atan(k*(b*thetadot + Ydot*cos(theta) - Xdot*sin(theta)))))/(m*r*pi);
    xdot(5) = (pi*tau_L*sin(theta) + pi*tau_R*sin(theta) - pi*Ydot*cd*r - 4*g*m*mu_sc*r*sin(theta)*(atan(k*(c*thetadot + Xdot*cos(theta) + Ydot*sin(theta))) + atan(k*(Xdot*cos(theta) - c*thetadot + Ydot*sin(theta)))) + 4*g*m*mu_lc*r*cos(theta)*(atan(k*(a*thetadot - Ydot*cos(theta) + Xdot*sin(theta))) - atan(k*(b*thetadot + Ydot*cos(theta) - Xdot*sin(theta)))))/(m*r*pi);
    xdot(6) = -(pi*c*tau_L - pi*c*tau_R + pi*bd*r*thetadot + 4*c*g*m*mu_sc*r*(atan(k*(c*thetadot + Xdot*cos(theta) + Ydot*sin(theta))) - atan(k*(Xdot*cos(theta) - c*thetadot + Ydot*sin(theta)))) + 4*a*g*m*mu_lc*r*atan(k*(a*thetadot - Ydot*cos(theta) + Xdot*sin(theta))) + 4*b*g*m*mu_lc*r*atan(k*(b*thetadot + Ydot*cos(theta) - Xdot*sin(theta))))/(I*r*pi);
end