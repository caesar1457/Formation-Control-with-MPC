function [obstacles, x_max, y_max, y_min] = random_obs_map()

    % Parameter initialisation
    x_max = 500; % Maximum width of the map
    y_max = 30; % Maximum height of the map
    y_min = -30; % Minimum height of the map
    obstacle_gap_range = [30, 30]; % Range of gaps between obstacles
    obstacle_length_range = [10, 15]; % obstacle length range
    obstacle_width_range = [10, 30];  % obstacle width range
    
    % Initialising an array of obstacles
    obstacles = [];
    
    x_current = 0; % current x-axis position
    
    
    while x_current < x_max
        % Generate starting x-axis of obstacle, add random gap
        x_start = x_current + obstacle_gap_range(1) + (obstacle_gap_range(2) - obstacle_gap_range(1)) * rand();
    
        % Out of bounds check
        if x_start > x_max
            break;
        end
    
        % Randomly generated lengths of obstacles
        obstacle_length = obstacle_length_range(1) + (obstacle_length_range(2) - obstacle_length_range(1)) * rand();
    
        % Double obstacle logic, triggered only when the obstacle length 
        % is less than a specific value
        if obstacle_length < 45/2
            double_obs = randi([1, 2]); % Randomly decide whether to generate a double obstacle
    
            if double_obs == 1
                
                direction = randi([0, 1]) * 2 - 1; 
                y_start = direction * randi([0, y_max/2]); 
                y_end = y_start + direction * obstacle_length; 
    
                % Mirror the location of the obstacle
                y_start_mirror = -y_end;
                y_end_mirror = -y_start;
    
                % Restriction of obstacle locations within the map
                y_start = max(y_min, min(y_max, y_start));
                y_end = max(y_min, min(y_max, y_end));
                y_start_mirror = max(y_min, min(y_max, y_start_mirror));
                y_end_mirror = max(y_min, min(y_max, y_end_mirror));
    
                % Adding obstacles to an array (original and mirrored)
                obstacles = [obstacles; x_start, min(y_start, y_end), x_start + obstacle_length, max(y_start, y_end)];
                obstacles = [obstacles; x_start, min(y_start_mirror, y_end_mirror), x_start + obstacle_length, max(y_start_mirror, y_end_mirror)];
    
                x_current = x_start + obstacle_length; 
                continue; 
            end
        end
    
        % If no double obstacle is generated
        obstacle_width = randi(obstacle_width_range);
        direction = randi([0, 1]) * 2 - 1;
        y_base = direction * randi([0, y_max]);
        y_start = y_base;
        y_end = y_base + direction * obstacle_width;
    
        % Ensure that y_start is the lower value
        if y_start > y_end
            temp = y_start;
            y_start = y_end;
            y_end = temp;
        end
    
        % End of obstacle x-coordinate
        x_end = x_start + obstacle_length;
    
        % Add data
        obstacles = [obstacles; x_start, y_start, x_end, y_end];
    
        x_current = x_end;
    
    end
end

