clear variables
close all
clc

syms a b c r m I g X Y T Xdot Ydot Tdot Xddot Yddot Tddot mu_sc mu_lc k tau_L tau_R

r1 = [-a; c];
r2 = [b; c];
r3 = [b; -c];
r4 = [-a; -c];

Rot = [cos(T) -sin(T); sin(T) cos(T)];

M = diag([m m I]);
B = 1 / r * [cos(T) cos(T); sin(T) sin(T); -c c];
Tau = [tau_L; tau_R];


v1 = inv(Rot) * [Xdot + Tdot * r1(2); Ydot - Tdot * r1(1)];
v2 = inv(Rot) * [Xdot + Tdot * r2(2); Ydot - Tdot * r2(1)];
v3 = inv(Rot) * [Xdot + Tdot * r3(2); Ydot - Tdot * r3(1)];
v4 = inv(Rot) * [Xdot + Tdot * r4(2); Ydot - Tdot * r4(1)];

Fr = Rot * [2 * mu_sc * m * g / pi * (atan(k * v1(1)) + atan(k * v2(1)) + atan(k * v3(1)) + atan(k * v4(1)));
           2 * mu_lc * m * g / pi * (atan(k * v1(2)) + atan(k * v2(2)) + atan(k * v3(2)) + atan(k * v4(2)))];

Mr = 2 * m * g / pi * (mu_lc * (-a * (atan(k * v1(2)) + atan(k * v4(2))) + b * (atan(k * v2(2)) + atan(k * v3(2)))) + mu_sc * c * (atan(k * v3(1)) + atan(k * v4(1)) - atan(k * v1(1)) - atan(k * v2(1))));

R = [Fr; Mr];

qddot = inv(M) * (B * Tau - R);

J = jacobian(qddot, [X, Y, T, Xdot, Ydot, Tdot]);





