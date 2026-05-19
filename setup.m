params.m = 0.5;
params.I = 0.1;
params.mu_sci = 0.5;
params.mu_lci = 0.5;
params.damping = [200 200 1]; % vel x, vel y, angular vel damping

params.a = 0.05;
params.b = 0.05;
params.c = 0.03;

r1 = [-params.a; params.c];
r2 = [params.b; params.c];
r3 = [params.b; -params.c];
r4 = [-params.a; -params.c];

params.wheel_rad = 0.025;

params.r_vecs = [r1 r2 r3 r4];

params.M = diag([params.m params.m params.I]); % Inertial matrix
params.g = 9.81;

params.add_vel_damping = 1; % Flag to add a damping term to the frictional damping

params.U_max = 1000;