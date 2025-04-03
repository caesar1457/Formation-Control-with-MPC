function [x_start, x_end] = traj_obs_pos_act(y_cart, obstacles, distance_over)

obs_min = obstacles(1,2) - distance_over;
obs_max = obstacles(1,4) + distance_over;

del_y_min = abs(y_cart - obs_min);
del_y_max = abs(y_cart - obs_max);

% go up
if del_y_max <= del_y_min

    y_move = del_y_max;
    x_buffer = 2*y_move;

    x_start = obstacles(1,1) - x_buffer;
    x_end = obstacles(1,3) + x_buffer;

% go down
elseif del_y_max > del_y_min

    y_move = del_y_min;
    x_buffer = 2*y_move;

    x_start = obstacles(1,1) - x_buffer;
    x_end = obstacles(1,3) + x_buffer;
end

end


