function xdot = get_state_derivs(t, x, tau, params)
    q = x(1:3);
    qdot = x(4:6);

    q_ddot = params.M \ (B(q, params.wheel_rad, params.c) * tau - R(q, qdot, params));
    xdot = [qdot; q_ddot];
end

function rotation_matrix = rot(theta)
    rotation_matrix = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end

function R_matrix = R(q, qdot, params) % Checked, good
    theta = q(3);
    %F = zeros(2, 4); % 4 Column vectors containing the friction forces for each wheel
    for i=1:4
        v_i = vel_at_wheel(q, qdot, params.r_vecs(:, i));
        F_i = params.m * params.g * diag([params.mu_sci params.mu_lci]) * sgn(v_i);
        F(:, i) = F_i;
    end
    M_r = -params.a * (F(2, 1) + F(2, 4)) + params.b * (F(2, 2) + F(2, 3)) + params.c * (-F(1, 1) - F(1, 2) + F(1, 3) + F(1, 4)); % Resistive moment about COM
    F_r = rot(theta) * sum(F, 2); % Perform a column-wise sum and then rotate by theta
    R_matrix = [F_r; M_r];
    if params.add_vel_damping
        R_matrix = R_matrix + diag(params.damping) * qdot;
    end
end

function y = sgn(x)
    k = 1e4;
    y = 2 / pi * atan(k * x);
end

function vel = vel_at_wheel(q, qdot, r) % Checked, good
    Xdot = qdot(1);
    Ydot = qdot(2);
    omega = qdot(3);
    t = q(3);
    vel = rot(-t) * [Xdot; Ydot] + omega * [-r(2); r(1)];
end

function B_matrix = B(q, r, c)
    theta = q(3);
    B_matrix = 1 / r * [cos(theta) cos(theta); sin(theta) sin(theta); -c c];
end