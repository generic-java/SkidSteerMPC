clear variables
close all
clc

syms a b c r m I g X Y theta Xdot Ydot thetadot Xddot Yddot thetaddot bd cd mu_sc mu_lc k tau_L tau_R

r1 = [-a; c];
r2 = [b; c];
r3 = [b; -c];
r4 = [-a; -c];

Rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];

M = diag([m m I]);
B = 1 / r * [cos(theta) cos(theta); sin(theta) sin(theta); -c c];
Tau = [tau_L; tau_R];

R1 = Rot * r1;
R2 = Rot * r2;
R3 = Rot * r3;
R4 = Rot * r4;

v1 = inv(Rot) * [Xdot - thetadot * R1(2); Ydot + thetadot * R1(1)];
v2 = inv(Rot) * [Xdot - thetadot * R2(2); Ydot + thetadot * R2(1)];
v3 = inv(Rot) * [Xdot - thetadot * R3(2); Ydot + thetadot * R3(1)];
v4 = inv(Rot) * [Xdot - thetadot * R4(2); Ydot + thetadot * R4(1)];

Fr = Rot * [2 * mu_sc * m * g / pi * (atan(k * v1(1)) + atan(k * v2(1)) + atan(k * v3(1)) + atan(k * v4(1)));
            2 * mu_lc * m * g / pi * (atan(k * v1(2)) + atan(k * v2(2)) + atan(k * v3(2)) + atan(k * v4(2)))] + diag([cd cd]) * [Xdot; Ydot];

Mr = 2 * m * g / pi * (mu_lc * (-a * (atan(k * v1(2)) + atan(k * v4(2))) + b * (atan(k * v2(2)) + atan(k * v3(2)))) + mu_sc * c * (atan(k * v3(1)) + atan(k * v4(1)) - atan(k * v1(1)) - atan(k * v2(1)))) + bd * thetadot;

R = [Fr; Mr];

qddot = simplify(inv(M) * (B * Tau - R));

J = jacobian(qddot, [X, Y, theta, Xdot, Ydot, thetadot]);

B = jacobian(qddot, [tau_L, tau_R]);





