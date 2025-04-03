function [filtered_obstacles] = random_obs_choice(y_cart, obstacles, distance_over)

filtered_obstacles = [];

i = 1;

while i <= size(obstacles, 1)
    
    % Check if the current obstacle is mirrored
    if i < size(obstacles, 1) && obstacles(i, 1) == obstacles(i+1, 1)

        % Mirroring exists to determine which obstacle needs to be preserved
        if y_cart <= obstacles(i+1, 4) + distance_over ...
                && y_cart >= obstacles(i+1, 2) - distance_over

            filtered_obstacles = [filtered_obstacles; obstacles(i+1, :)];

        else

            filtered_obstacles = [filtered_obstacles; obstacles(i, :)];

        end
        
        % Skip the next item in the Mirror Barrier
        i = i + 1;
    else
        % No mirrors, just add
        filtered_obstacles = [filtered_obstacles; obstacles(i, :)];
    end
    
    % Move to the next element
    i = i + 1;
end

end
