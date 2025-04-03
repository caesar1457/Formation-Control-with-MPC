function [x_pos, y_pos] = traj_obs_pass(y_cart, v_cart, obstacles, distance_over, step)

obs_min = obstacles(1,2) - distance_over;
obs_max = obstacles(1,4) + distance_over;

del_down = abs(y_cart - obs_min);
del_up = abs(y_cart - obs_max);

width_obs = abs(obstacles(1,1) - obstacles(1,3));
local_x = obstacles(1,1);
local_x_2 = obstacles(1,3); 

v_x = v_cart; % Set cart speed

if del_up <= del_down

    % Desired path geometry data
    y_move = del_up; % Distance the y-axis needs to be moved
    y_move_part = y_move / 2; % This process has two parts
    del_x_cto = 2*y_move; % The distance of cart starts avoiding obstacles set to be 2 times the y-axis travelling distance

    distance_x_start = local_x - del_x_cto;
    distance_x_end = local_x_2 + del_x_cto;

    % Coordinate
    y_peak = y_cart + y_move;
    x_act_1 = local_x - del_x_cto;


    % Accelerated motion data
    t_full = del_x_cto / v_x; % Total running time for obstacle avoidance
    t_part = t_full / 2; % This process has up and down, two parts
    a_y = ((y_move/2) * 2) / (t_part^2); % Acceleration in the y-axis, same acceleration and deceleration


    % Key position in the acceleration phase
    v_y_max = a_y * t_part; % Maximum speed
    x_move_part = del_x_cto/2; % x-axis displacement to reach maximum velocity

    % Coordinate
    pos_y_vy_max = y_cart + y_move_part; % Maximum speed y position
    pos_x_vy_max = local_x - del_x_cto/2; % Maximum speed x position


% Obstacle avoidance path setting
    distance = distance_x_start:step:distance_x_end;
    
    x_pos = zeros;
    y_pos = zeros;

    for i = 1:length(distance)
        
        % Not before response position
        if distance(i) <= local_x - del_x_cto
    
            x_pos(i) = distance(i);
            y_pos(i) = y_cart;
    
        % Accelerated phase of response initiated
        elseif distance(i) <= local_x - del_x_cto/2
    
            x_pos(i) = distance(i);
            del_t_in = (x_pos(i) - x_act_1) / v_x;
            y_pos(i) = y_cart + 0.5 * a_y * (del_t_in) ^ 2;
    
        % Deceleration phase of response
        elseif distance(i) <= local_x
    
            x_pos(i) = distance(i);
            del_t_de = (x_pos(i) - pos_x_vy_max) / v_x;
            y_pos(i) = pos_y_vy_max ...
                     + v_y_max * del_t_de - 0.5 * a_y * (del_t_de^2);
    
        % Straight through phase
        elseif distance(i) <= local_x + width_obs
    
            x_pos(i) = distance(i);
            y_pos(i) = y_peak;
    
        % Return to initial acceleration phase
        elseif distance(i) <= local_x + width_obs + del_x_cto/2
    
            x_pos(i) = distance(i);
            del_t_in = (x_pos(i) - local_x_2) / v_x;
            y_pos(i) = y_peak - 0.5 * a_y * (del_t_in) ^ 2;
    
        % Return to initial deceleration phase
        elseif distance(i) <= distance_x_end
    
            x_pos(i) = distance(i);
            del_t_de = (x_pos(i) - local_x_2 - x_move_part) / v_x;
            y_pos(i) = pos_y_vy_max ...
                     - v_y_max * del_t_de + 0.5 * a_y * (del_t_de^2);

        end
    end

elseif del_up > del_down

    % Desired path geometry data
    y_move = del_down; % Distance the y-axis needs to be moved
    y_move_part = y_move / 2; % This process has two parts
    del_x_cto = 2*y_move;

    distance_x_start = local_x - del_x_cto;
    distance_x_end = local_x_2 + del_x_cto;

    % Coordinate
    y_peak = y_cart - y_move;
    x_act_1 = local_x - del_x_cto;


    % Accelerated motion data
    t_full = del_x_cto / v_x; 
    t_part = t_full / 2; 
    a_y = ((y_move/2) * 2) / (t_part^2); 


    % Key position in the acceleration phase
    v_y_max = a_y * t_part; 
    x_move_part = del_x_cto/2;

    % Coordinate
    pos_y_vy_max = y_cart - y_move_part; 
    pos_x_vy_max = local_x - del_x_cto/2; 


    distance = distance_x_start:step:distance_x_end;
    
    x_pos = zeros;
    y_pos = zeros;

    for i = 1:length(distance)
        
        if distance(i) <= local_x - del_x_cto
    
            x_pos(i) = distance(i);
            y_pos(i) = y_cart;
    
        elseif distance(i) <= local_x - del_x_cto/2
    
            x_pos(i) = distance(i);
            del_t_in = (x_pos(i) - x_act_1) / v_x;
            y_pos(i) = y_cart - (0.5 * a_y * (del_t_in) ^ 2);
    
        elseif distance(i) <= local_x
    
            x_pos(i) = distance(i);
            del_t_de = (x_pos(i) - pos_x_vy_max) / v_x;
            y_pos(i) = pos_y_vy_max ...
                     - v_y_max * del_t_de + 0.5 * a_y * (del_t_de^2);
    
        elseif distance(i) <= local_x + width_obs
    
            x_pos(i) = distance(i);
            y_pos(i) = y_peak;
    
        elseif distance(i) <= local_x + width_obs + del_x_cto/2
    
            x_pos(i) = distance(i);
            del_t_in = (x_pos(i) - local_x_2) / v_x;
            y_pos(i) = y_peak + 0.5 * a_y * (del_t_in) ^ 2;
    
        elseif distance(i) <= distance_x_end
    
            x_pos(i) = distance(i);
            del_t_de = (x_pos(i) - local_x_2 - x_move_part) / v_x;
            y_pos(i) = pos_y_vy_max ...
                     + v_y_max * del_t_de - 0.5 * a_y * (del_t_de^2);
        
        end
    end

end

end

