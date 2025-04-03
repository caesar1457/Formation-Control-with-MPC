function [traj_matrix] = traj_create(obstacles, y_cart, v_cart, distance_over, step, x_max)

% Remove non-complaint barrier blocks
[filtered_obstacles] = random_obs_choice(y_cart, obstacles, distance_over);

% Initially state
x_full = x_max + 50;

x_pos = [0];
y_pos = [y_cart];

% The end position of the previous obstacle, initially set as the starting point
last_obstacle_end_x = 0;
obstacles_single_x = 0;

% Create trajectory
for i = 1:size(filtered_obstacles,1)

    obstacles_single = filtered_obstacles(i,:);
        
    if y_cart <= obstacles_single(1,4) + distance_over ...
    && y_cart >= obstacles_single(1,2) - distance_over % Need for obstacle avoidance
    
        % Get the start and end points of the obstacle avoidance process
        [x_start, x_end] = traj_obs_pos_act(y_cart, obstacles_single, distance_over);
    
        % Draw the path from the previous end point to the current start point
        x_between = last_obstacle_end_x:step:x_start;
        y_between = repmat(y_cart, size(x_between));
    
        x_pos = [x_pos, x_between];
        y_pos = [y_pos, y_between];
    
        % Adding a path for obstacle avoidance
        [x_pos_new, y_pos_new] = traj_obs_pass(y_cart, v_cart, ...
                                                 obstacles_single, ...
                                                 distance_over, step);
    
        x_pos = [x_pos, x_pos_new];
        y_pos = [y_pos, y_pos_new];
    
        last_obstacle_end_x = x_end;
    
        obstacles_single_x = obstacles_single(1,1);

    else % No need for obstacle avoidance
    
        % Draw the path from the end of the actionto the end of the current obstacle
        x_between = last_obstacle_end_x:step:obstacles_single(1,3);
        y_between = repmat(y_cart, size(x_between));
        
        x_pos = [x_pos, x_between];
        y_pos = [y_pos, y_between];
    
        % Reset the next starting position
        last_obstacle_end_x = obstacles_single(1,3);
        obstacles_single_x = obstacles_single(1,1);

    end

end

if last_obstacle_end_x < x_full

    x_end = last_obstacle_end_x:step:x_full;
    y_end = repmat(y_cart, size(x_end));

    x_pos = [x_pos, x_end];
    y_pos = [y_pos, y_end];

end

%% Return array with x position, y position and angular velocity

dt = step/v_cart;
num_points = length(x_pos);
theta = zeros(1, num_points);
omega = zeros(1, num_points);

% Calculate the angle (theta) for each point
for i = 1:(num_points - 1) % Subtract 1 to avoid going out of index range

    delta_x = x_pos(i+1) - x_pos(i);
    delta_y = y_pos(i+1) - y_pos(i);

    theta(i) = atan2(delta_y, delta_x);
    v_t(i) = sqrt(delta_x^2 + delta_y^2)/dt;

end

% The theta of the last point can be set to be the same as the previous one
theta(num_points) = theta(num_points-1);
v_t(num_points) = v_cart;

% Calculate angular velocity
for i = 1:(num_points - 1)
    omega(i) = (theta(i+1) - theta(i)) / dt;
end

% For the last point
omega(num_points) = omega(num_points-1);

% Merge all the data into a matrix
traj_matrix = [x_pos; y_pos; omega; v_t];

end

