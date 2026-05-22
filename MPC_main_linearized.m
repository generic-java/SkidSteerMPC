clear variables
close all
clc

addpath(genpath('C:\Users\geelhose\OneDrive - Rose-Hulman Institute of Technology\Desktop\Research\Model\YALMIP'));
addpath('.\Functions');
addpath('.\MPC');


%% Define Physical Parameters
setup;

%% Configure simulation

% Initialize integration parameters
t_s = 10e-3; % ms
t_f = 10;

linearize_reset = 3;
linearize_counter = 0;

t_gen = 0:t_s:t_f;
reference = zeros(6, length(t_gen));
reference(4, :) = tanh(t_gen);

% Initialize simulation variables
curr_t = 0;
curr_x = zeros(6, 1);
curr_u = zeros(2, 1);

% Linearize
[A, B] = linearize(curr_t, curr_x, curr_u, params);
x_bar = curr_x;
u_bar = curr_u;

Ad = eye(7) + A * t_s;
Bd = B * t_s;

% Initialize storage variables
t_sim = [];
x_sim = [];

% Configure MPC and acquire solver
Q = diag([0 0 0 1000 0 0]);
R = diag([0.0 0.0]);
Np = 25;
Nc = 20;
solver = get_MPC_solver(Q, R, Np, Nc);

for i = 1:t_f/t_s

    if linearize_counter == linearize_reset
        [A, B] = linearize(curr_t, curr_x, curr_u, params);
        x_bar = curr_x;
        u_bar = curr_u;

        linearize_counter = 0;

        Ad = eye(7) + A * t_s;
        Bd = B * t_s;
    end
    linearize_counter = linearize_counter + 1;

    % Acquire optimal control input from MPC
    query_t = curr_t * ones(1, Np) + linspace(0, t_s * (Np - 1), Np);
    reference_preview = interp1(t_gen, reference', query_t)';

    MPC_parameters = {Ad, Bd, x_bar, u_bar, reference_preview, curr_x};

    try
        [solved_sys, exit_flag, diagnostics] = solver(MPC_parameters);
        u_optimal = solved_sys(:, 1);
    
        if exit_flag == 0
            curr_u = u_optimal;
        end
    catch
        % Do nothing - keep current u
        fprintf("Optimization failed.\n");
    end

    del_x_aug = [curr_x - x_bar; 1];
    del_u = curr_u - u_bar;    
   
    % Solve the ODE numerically over the subinterval
    t_step = [t_gen(i), t_gen(i+1)];
    options = [];
    [t_sim_local, x_bar_sim_local] = ode23s(@(t, x) get_state_derivs_linear(t, x, del_u, A, B), t_step, del_x_aug, options);

    x_sim = [x_sim; x_bar_sim_local(:, 1:6) + x_bar'];
    curr_x  = x_sim(end, :)';

    t_sim = [t_sim; t_sim_local];
    curr_t = t_sim(end);
end

%% Plot results
x = x_sim(:, 1);
y = x_sim(:, 2);
heading = x_sim(:, 3);

velx = x_sim(:, 4);
vely = x_sim(:, 5);
omega = x_sim(:, 6);

set(0, 'DefaultLineLineWidth', 2);

figure
plot(x, y);
xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 14)
grid on;
axis equal;

figure
plot(t_sim, heading * 180 / pi);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('Heading (deg)', 'Interpreter', 'latex', 'FontSize', 14)
grid on;

figure
plot(t_sim, velx);
hold on;
plot(t_gen, reference(4, :));
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$V_x$ $(\frac{m}{s})$', 'Interpreter', 'latex', 'FontSize', 14)
grid on;

anim_car_traj(0.01, t_sim, x_sim, params.a, params.b, params.c)