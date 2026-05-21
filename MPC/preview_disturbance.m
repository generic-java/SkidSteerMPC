function reference = preview_disturbance(t, Np, t_s, dist_traj)
    
    t_f = dist_traj.Time(end);
    reference = zeros(6, Np); % TODO: don't hardcode 6
    for j = 1:Np
        t = min([t; t_f]);
        reference(:, j) = interp1(dist_traj.Time, dist_traj.Dist', t)';
        t = t + t_s;
    end

end