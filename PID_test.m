clear variables
close all
clc

addpath(genpath('C:\Users\geelhose\OneDrive - Rose-Hulman Institute of Technology\Desktop\Research\Model\YALMIP'));
addpath(genpath('C:\Users\geelhose\OneDrive - Rose-Hulman Institute of Technology\Desktop\Research\Model\OPTI-master'));


%% Setup
setup;

t_s = 10e-3; % ms
t_f = 5;

t_gen = 0:t_s:t_f;

pid = PID(10, 0, 0);
ref_x = 3;


x_sim = [];
t_sim = [];
inputs = [];

x0 = zeros(6, 1);
x_curr = x0;

for k = 1:t_f/t_s  
   
    input = pid.calculate(x_curr(1), ref_x);
    u = input * ones(2, 1);

    t_step = [t_gen(k), t_gen(k+1)];
    options = [];
    [t_sim_local, x_sim_local] = ode23s(@(t, x) get_state_derivs(t, x, u, params), t_step, x_curr, options);
    
    % Store all the state values from the time step integration
    x_sim = [x_sim; x_sim_local];
    x_curr  = x_sim(end, :)'; % Keep track of the current state to pass back into the MPC solver
    t_sim = [t_sim; t_sim_local];

    % Our input was held "constant" over that entire time step
    inputs = [inputs; (u*ones(length(t_sim_local), 1)')'];

end

%% Plot
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

figure
plot(t_sim, velx)
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$V_x$ $(\frac{m}{s})$', 'Interpreter', 'latex', 'FontSize', 14)
grid on;

anim_car_traj(t_sim(2) - t_sim(1), x_sim, params.a, params.b, params.c)