function reference = preview_reference(t, Np, t_s, ref_traj)
    
    t_f = ref_traj.Time(end);
    reference = zeros(6, Np); % TODO: don't hardcode the 6
    for j = 1:Np
        t = min([t; t_f]);
        reference(:, j) = interp1(ref_traj.Time, ref_traj.Ref', t)';
        t = t + t_s;
    end

end