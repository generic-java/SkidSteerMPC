function anim_car_traj(dt, t, states, a, b, c)

    function rect = get_car_rect(x, y, heading, a, b, c)
        rect = [[-a; c], [b; c], [b; -c], [-a; -c]]; % Define the car rect
        rect = [cos(heading) -sin(heading); sin(heading) cos(heading)] * rect + [x; y]; % Rotate about COM and translate to actual position
    end

    x = states(:, 1);
    y = states(:, 2);
    heading = states(:, 3);

    figure
    axis equal;
    xlim([min(x) - 0.1 max(x) + 0.1]);
    ylim([min(y) - 0.1 max(y) + 0.1]);
    grid on;

    rect = get_car_rect(0, 0, 0, a, b, c);
    handle = patch(rect(1, :), rect(2, :), 'blue', 'FaceColor', 'none', 'EdgeColor', 'black', 'LineWidth', 2); % Get the handle to the graphics object

    for curr_t=t(1):dt:t(end)
        [t_sample, indices] = unique(t);
        x_sample = x(indices);
        y_sample = y(indices);
        heading_sample = heading(indices);
        
        x_curr = interp1(t_sample, x_sample, curr_t);
        y_curr = interp1(t_sample, y_sample, curr_t);
        heading_curr = interp1(t_sample, heading_sample, curr_t);
        rect = get_car_rect(x_curr, y_curr, heading_curr, a, b, c);
        handle.Vertices = rect';
        drawnow limitrate nocallbacks;
        pause(dt);
    end

end