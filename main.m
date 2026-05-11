clear variables
close all
clc

addpath(genpath('C:\Users\geelhose\OneDrive - Rose-Hulman Institute of Technology\Desktop\Research\Model\YALMIP'));
addpath(genpath('C:\Users\geelhose\OneDrive - Rose-Hulman Institute of Technology\Desktop\Research\Model\OPTI-master'));


%% Define Physical Parameters
setup;

t_s = 10e-3; % ms
t_f = 5;
x = zeros(6, 1);

Np = 5;
Nc = 3;
Q = diag([1000 1000 0 0 0 0]);
R = diag([0.01 0.01]);
S = zeros(6);

% Define disturbance and reference trajectories
t_gen = 0:t_s:t_f;
ref_traj.Time = t_gen;
ref_traj.Ref = [0.1 * t_gen; zeros(5, length(t_gen))];

dist_traj.Time = t_gen;
dist_traj.Dist = zeros(6, length(t_gen));

u_traj = 0 * ones(2, Nc);

x_sim = [];
t_sim = [];
inputs = [];
disturbances = [];

for k = 1:t_f/t_s  
   
    [u, u_traj] = solve_MPC(Q, R, S, Np, Nc, t_gen(k), k, x, ref_traj, dist_traj, t_s, params, u_traj);

    t_step = [t_gen(k), t_gen(k+1)];
    options = [];
    [t_sim_local, x_sim_local] = ode23s(@(t, x) get_state_derivs(t, x, u, params), t_step, x, options);
    
    % Store all the state values from the time step integration
    x_sim = [x_sim; x_sim_local];
    x  = x_sim(end, :)'; % Keep track of the current state to pass back into the MPC solver
    t_sim = [t_sim; t_sim_local];

    % Our input was held "constant" over that entire time step
    inputs = [inputs; (u*ones(length(t_sim_local), 1)')']; 
    %disturbances = [disturbances; d*ones(numel(t_sim_local), 1)];

    fprintf("Iteration %.0f \n", k);

end

%%
x = x_sim(:, 1);
y = x_sim(:, 2);
heading = x_sim(:, 3);

velx = x_sim(:, 4);
vely = x_sim(:, 5);
omega = x_sim(:, 6);

set(0, 'DefaultLineLineWidth', 2);

figure
plot(x, y)
xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 14)
grid on;

figure
plot(t_sim, heading * 180 / pi)
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('Heading (deg)', 'Interpreter', 'latex', 'FontSize', 14)
grid on;

% figure
% plot(t_sim, velx)
% xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
% ylabel('$V_x$', 'Interpreter', 'latex', 'FontSize', 14)
% grid on;

%anim_car_traj(t_sim(2) - t_sim(1), x_sim, params.a, params.b, params.c)