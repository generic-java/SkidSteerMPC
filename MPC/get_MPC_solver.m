function problem = get_MPC_solver(Q, R, Np, Nc)

    % n_u is the number of control inputs; n is the number of states
    n_u = 2;
    n = 6;
    
    % Define sdpvar matrix U for optimal input at each sample    
    U = sdpvar(n_u, Nc, 'full'); % 'full' means the matrix is not symmetric.  We want a full set of decision vars
    X = sdpvar(n + 1, Np, 'full');

    % Define optimizer parameters
    Ad = sdpvar(n + 1, n + 1, 'full');
    Bd = sdpvar(n + 1, n_u, 'full');
    x_bar = sdpvar(n, 1, 'full');
    u_bar = sdpvar(n_u, 1, 'full');
    reference_preview = sdpvar(n, Np, 'full');
    x0 = sdpvar(n, 1, 'full');

    % Define cells for optimizer variables and parameters
    variables = {U};
    parameters = {Ad, Bd, x_bar, u_bar, reference_preview, x0};
    
    % Initialize the objective function at 0
    J = 0;
    
    % Initialize an empty constraint set
    constr = X(:, 1) == [x0; 1];    
    
    for j = 1:Np
    
        % Define the current reference vector
        R_curr = reference_preview(:, j);
    
        % Pull out current states and input from decision variable set
        if j < Nc
            U_curr = U(:, j);
        else
            U_curr = U(:, Nc);
        end
        X_curr = X(:, j);
    
        % Define the error
        E_curr = R_curr - X_curr(1:n);
    
        % Update the objection function J
        J = J + E_curr' * Q * E_curr + U_curr' * R * U_curr;
    
        % Apply all necessary constraints
        % constr = [constr, U_curr(1) <= params.U_max];
        % constr = [constr, U_curr(1) >= -params.U_max];
        % constr = [constr, U_curr(2) <= params.U_max];
        % constr = [constr, U_curr(2) >= -params.U_max];
        %constr = [constr, U_curr(1) == U_curr(2)];
        
    
        % Update state variables for next prediction step
        del_x_aug = X_curr - [x_bar; 0];
        del_u = U_curr - u_bar;

        if j < Np
            constr = [constr, X(:, j + 1) == X(:, j) + Ad * del_x_aug + Bd * del_u + [x_bar; 0]];
        end
    end
    
    % Set options and create the optimizer
    options = sdpsettings('solver', 'quadprog', 'verbose', 2);
    problem = optimizer(constr, J, options, parameters, variables); % Create the optimizer object

end