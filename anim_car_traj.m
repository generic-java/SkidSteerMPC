function anim_car_traj(ts, states, a, b, c)

    function rect = get_car_rect(x, y, heading, a, b, c)
        rect = [[-a; c], [b; c], [b; -c], [-a; -c]]; % Define the car rect
        rect = [cos(heading) -sin(heading); sin(heading) cos(heading)] * rect + [x; y]; % Rotate about COM and translate to actual position
    end

    x = states(:, 1);
    min(x)
    max(x)
    y = states(:, 2);
    heading = states(:, 3);

    figure
    axis equal;
    xlim([min(x) - 0.1 max(x) + 0.1]);
    ylim([min(y) - 0.1 max(y) + 0.1]);
    grid on;

    rect = get_car_rect(0, 0, 0, a, b, c);
    handle = patch(rect(1, :), rect(2, :), 'Green'); % Get the handle to the graphics object

    for i = 1:size(states, 1)
        rect = get_car_rect(x(i), y(i), heading(i), a, b, c);
        handle.Vertices = rect';
        drawnow limitrate nocallbacks;
        pause(ts);
    end

end