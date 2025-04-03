clc;
clear;
close all;

y_cart_1 = 10;
y_cart_2 = -10;
v_cart = 1;
distance_over = 2;
step = 0.2;

[obstacles, x_max, y_max, y_min] = random_obs_map();
x_full = x_max + 50;

[traj_matrix_1] = traj_create(obstacles, y_cart_1, v_cart, distance_over, step, x_max);
x_pos_1 = traj_matrix_1(1,:);
y_pos_1 = traj_matrix_1(2,:);
omega_L_1 = traj_matrix_1(3,:);
v_L_1 = traj_matrix_1(4,:);


[traj_matrix_2] = traj_create(obstacles, y_cart_2, v_cart, distance_over, step, x_max);
x_pos_2 = traj_matrix_2(1,:);
y_pos_2 = traj_matrix_2(2,:);
omega_L_2 = traj_matrix_2(3,:);
v_L_2 = traj_matrix_2(4,:);

len1 = length(x_pos_1);
len2 = length(x_pos_2);

if length(x_pos_1) > length(x_pos_2)

    extra_elements = len1 - len2;
    fill_values1 = repmat(x_full, 1, extra_elements);
    fill_values2 = repmat(v_cart, 1, extra_elements);

    x_pos_2(end+1:end+extra_elements) = fill_values1;
    y_pos_2(len1) = 0;
    omega_L_2(len1) = 0;
    v_L_2(end+1:end+extra_elements) = fill_values2;

elseif len2 > len1
  
    extra_elements = len2 - len1;
    fill_values1 = repmat(x_full, 1, extra_elements);
    fill_values2 = repmat(v_cart, 1, extra_elements);

    x_pos_1(end+1:end+extra_elements) = fill_values1;
    y_pos_1(len1) = 0;
    omega_L_1(len1) = 0;
    v_L_1(end+1:end+extra_elements) = fill_values2;

end



% Defining parameters
v_L = v_cart; % leader's speed

% L 1&2
lambda_LF_d = - 1.5; 
phi_LF_d = -pi/20; 

% L 3&4
lambda_LF_d_a2 = - 3; 
phi_LF_d_a2 = pi/20; 

% initialisation state
leader_pos_1 = [0; 10; 0];
leader_pos_2 = [0; -10; 0];

leader_pos_3 = [0; 10; 0]; % same as leader 1
leader_pos_4 = [0; -10; 0]; % same as leader 2

omega_L = 0;

follower_pos_1 = [0; 3; 0]; % follower的状态误差
follower_pos_2 = [0; 3; 0];
follower_pos_3 = [0; 3; 0];
follower_pos_4 = [0; 3; 0];
% e = [1; 1; 0];

% time parameter
dt = step/v_cart;
numSteps = length(omega_L_1);

% MPC parameter
N = 5; % Projected time step
Q = diag([20, 25, 10]); % state weight
R = diag([0.01, 0.01]); % control weighting


%% ********************* 111 *********************
for k = 1:numSteps


    follower_history_1(:, k) = follower_pos_1;
    leader_history_1(:, k) = leader_pos_1;


    u0 = [0.9; 0.1]; % initial guess
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');

    u_opt = fmincon(@(u) ...
        objectiveFunc_dis(u, follower_pos_1, leader_pos_1,...
                          v_L_1(1,k), omega_L_1(1,k), ...
                          lambda_LF_d, phi_LF_d, Q, R, dt, N), ...
                          u0, [], [], [], [], [], [], [], options);


    % Using optimised control inputs
    v_F = u_opt(1);
    omega_F = u_opt(2);

    % Update follower status
    follower_pos_1 = follower_pos_1 + [v_F*cos(follower_pos_1(3)); v_F*sin(follower_pos_1(3)); omega_F] * dt;

    % e_dot = [v_L_1(1,k)*cos(e(3)) - v_F + omega_F*e(2) - omega_L_1(1,k)*lambda_LF_d*sin(phi_LF_d + e(3));
    %          v_L_1(1,k)*sin(e(3)) - omega_F*e(1) + omega_L_1(1,k)*lambda_LF_d*cos(phi_LF_d + e(3));
    %          omega_L_1(1,k) - omega_F];
    % e = e + e_dot * dt;

    % Update leaders status
    leader_pos_1 = [x_pos_1(1,k); y_pos_1(1,k); omega_L_1(1,k)];
end



%% ********************* 222 *********************
for k = 1:numSteps

    follower_history_2(:, k) = follower_pos_2;
    leader_history_2(:, k) = leader_pos_2;

    u0 = [0.5; 0.1]; % initial guess
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');

        u_opt = fmincon(@(u) ...
        objectiveFunc_dis(u, follower_pos_2, leader_pos_2,...
                          v_L_2(1,k), omega_L_2(1,k), ...
                          lambda_LF_d, phi_LF_d, Q, R, dt, N), ...
                          u0, [], [], [], [], [], [], [], options); 


    % Using optimised control inputs
    v_F = u_opt(1);
    omega_F = u_opt(2);

    % Update follower status
    follower_pos_2 = follower_pos_2 + [v_F*cos(follower_pos_2(3)); v_F*sin(follower_pos_2(3)); omega_F] * dt;

    % e_dot = [v_L_2(1,k)*cos(e(3)) - v_F + omega_F*e(2) - omega_L_2(1,k)*lambda_LF_d*sin(phi_LF_d + e(3));
    %          v_L_2(1,k)*sin(e(3)) - omega_F*e(1) + omega_L_2(1,k)*lambda_LF_d*cos(phi_LF_d + e(3));
    %          omega_L_2(1,k) - omega_F];
    % e = e + e_dot * dt;

    % Update leader status
    leader_pos_2 = [x_pos_2(1,k); y_pos_2(1,k); omega_L_2(1,k)];
end


%% ********************* 333 *********************
for k = 1:numSteps

    follower_history_3(:, k) = follower_pos_3;
    leader_history_3(:, k) = leader_pos_3; % same as leader 1

    u0 = [0.5; 0.1]; % initial guess
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');

        u_opt = fmincon(@(u) ...
        objectiveFunc_dis(u, follower_pos_3, leader_pos_3,...
                          v_L_1(1,k), omega_L_1(1,k), ...
                          lambda_LF_d_a2, phi_LF_d_a2, Q, R, dt, N), ...
                          u0, [], [], [], [], [], [], [], options); 


    % Using optimised control inputs
    v_F = u_opt(1);
    omega_F = u_opt(2);

    % Update follower status
    follower_pos_3 = follower_pos_3 + [v_F*cos(follower_pos_3(3)); v_F*sin(follower_pos_3(3)); omega_F] * dt;

    % Update leader status
    leader_pos_3 = [x_pos_1(1,k); y_pos_1(1,k); omega_L_1(1,k)];
end

%% ********************* 444 *********************
for k = 1:numSteps

    follower_history_4(:, k) = follower_pos_4;
    leader_history_4(:, k) = leader_pos_4; % same as leader 2

    u0 = [0.5; 0.1]; % initial guess
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');

        u_opt = fmincon(@(u) ...
        objectiveFunc_dis(u, follower_pos_4, leader_pos_4,...
                          v_L_2(1,k), omega_L_2(1,k), ...
                          lambda_LF_d_a2, phi_LF_d_a2, Q, R, dt, N), ...
                          u0, [], [], [], [], [], [], [], options); 


    % Using optimised control inputs
    v_F = u_opt(1);
    omega_F = u_opt(2);

    % Update follower status
    follower_pos_4 = follower_pos_4 + [v_F*cos(follower_pos_4(3)); v_F*sin(follower_pos_4(3)); omega_F] * dt;

    % Update leader status
    leader_pos_4 = [x_pos_2(1,k); y_pos_2(1,k); omega_L_2(1,k)];
end

%% output picture
x_pos_e1 = follower_history_1(1, :);
y_pos_e1 = follower_history_1(2, :);

x_pos_e2 = follower_history_2(1, :);
y_pos_e2 = follower_history_2(2, :);

x_pos_e3 = follower_history_3(1, :);
y_pos_e3 = follower_history_3(2, :);

x_pos_e4 = follower_history_4(1, :);
y_pos_e4 = follower_history_4(2, :);

% ****************** Animation Follow ******************
% Creating a new drawing window
figure;

% Setting the drawing range and initial view
axis([0 50 y_min y_max]); % Initial display of the range 0 to 30

% Keep image on
hold on;

% Mapping the barriers
for i = 1:size(obstacles, 1)

    rectangle('Position', [obstacles(i, 1), obstacles(i, 2), ...
               obstacles(i, 3) - obstacles(i, 1), ...
               obstacles(i, 4) - obstacles(i, 2)],...
               'FaceColor', 'k');
end


% Initialise an empty handle for the cart's position
h_car_1 = plot(NaN, NaN, 'ro');  % 'bo' for blue dot
h_car_2 = plot(NaN, NaN, 'ro');

h_car_1_f = plot(NaN, NaN, 'bo');
h_car_2_f = plot(NaN, NaN, 'bo');

h_car_1_f1 = plot(NaN, NaN, 'go');
h_car_2_f1 = plot(NaN, NaN, 'go');

% Initialise an empty trace
h_path_1 = plot(NaN, NaN, 'r--'); % 'b--' indicates a blue dotted line
h_path_2 = plot(NaN, NaN, 'r--');

h_path_1_f = plot(NaN, NaN, 'b--'); 
h_path_2_f = plot(NaN, NaN, 'b--'); 

h_path_1_f1 = plot(NaN, NaN, 'g--'); 
h_path_2_f1 = plot(NaN, NaN, 'g--'); 

% For each point in the path for cart 1
for i = 1:length(x_pos_1)

    % Update the position of the trolley
    set(h_car_1, 'XData', x_pos_1(i), 'YData', y_pos_1(i));
    set(h_car_2, 'XData', x_pos_2(i), 'YData', y_pos_2(i));

    set(h_car_1_f, 'XData', x_pos_e1(i),...
                   'YData', y_pos_e1(i));
    set(h_car_2_f, 'XData', x_pos_e2(i),...
                   'YData', y_pos_e2(i));

    set(h_car_1_f1, 'XData', x_pos_e3(i),...
                   'YData', y_pos_e3(i));
    set(h_car_2_f1, 'XData', x_pos_e4(i),...
                   'YData', y_pos_e4(i));

    % Update the path so far
    set(h_path_1, 'XData', x_pos_1(1:i), 'YData', y_pos_1(1:i));
    set(h_path_2, 'XData', x_pos_2(1:i), 'YData', y_pos_2(1:i));

    set(h_path_1_f, 'XData', x_pos_e1(1:i), ...
                    'YData', y_pos_e1(1:i));

    set(h_path_2_f, 'XData', x_pos_e2(1:i), ...
                    'YData', y_pos_e2(1:i));

    set(h_path_1_f1, 'XData', x_pos_e3(1:i), ...
                    'YData', y_pos_e3(1:i));

    set(h_path_2_f1, 'XData', x_pos_e4(1:i), ...
                    'YData', y_pos_e4(1:i));


    % set(h_path_1_f, 'XData', x_pos_e1(1:i), 'YData', y_pos_e1(1:i));
    % set(h_path_2_f, 'XData', x_pos_e2(1:i), 'YData', y_pos_e2(1:i));

    % Update the x-axis according to the position of the car
    if x_pos_1(i) > 25 && x_pos_1(i) < (x_full - 25)

        % Move the x-axis view to keep the car in the centre
        axis([x_pos_1(i) - 25 x_pos_1(i) + 25 y_min y_max]);
        % axis([x_pos_2(i) - 25 x_pos_2(i) + 25 y_min y_max]);

    elseif x_pos_1(i) >= (x_full - 25)

        % Fixed x-axis shows the last 50 metres in the end
        axis([x_full - 50 x_full y_min y_max]);
        % axis([x_full - 50 x_full y_min y_max]);

    end

    % Force MATLAB to plot updates immediately
    drawnow;

    % Pause for a moment to observe
    pause(0.01);
end

hold off;

% For each point in the path for cart 2
for i = 1:length(x_pos_2)

    Update the position of the trolley


    Update the path so far


    Update the x-axis according to the position of the car
    if x_pos_1(i) > 25 && x_pos_2(i) < (x_full - 25)

        Move the x-axis view to keep the car in the centre

    elseif x_pos_2(i) >= (x_full - 25)
        Fixed x-axis shows the last 50 metres in the end

    end

    Force MATLAB to plot updates immediately
    drawnow;

    Pause for a moment to observe
    pause(0.01);
end

Release Drawing

% 
% % ****************** 固定显示 ******************
% % 创建一个新的绘图窗口
% figure;
% 
% % 设置绘图范围（根据你的实际场景）
% axis([0 x_full y_min y_max]);
% 
% % 保持图像开启，这样我们就可以连续绘制
% hold on;
% 
% % 绘制障碍
% axis([0 x_max y_min y_max]); % 设置坐标轴范围
% for i = 1:size(obstacles, 1)
%     rectangle('Position', [obstacles(i, 1), obstacles(i, 2), obstacles(i, 3) - obstacles(i, 1), obstacles(i, 4) - obstacles(i, 2)], 'FaceColor', 'k');
% end
% 
% % 绘制路径
% plot(x_pos_1, y_pos_1, '-r', 'LineWidth', 1.5); % 路径以蓝色线绘制
% plot(x_pos_2, y_pos_2, '-r', 'LineWidth', 1.5); % 路径以蓝色线绘制
% 
% plot(x_pos_e1, y_pos_e1, '-b', 'LineWidth', 1.5); % 路径以蓝色线绘制
% plot(x_pos_e2, y_pos_e2, '-b', 'LineWidth', 1.5); % 路径以蓝色线绘制
% 
% plot(x_pos_e3, y_pos_e4, '-g', 'LineWidth', 1.5); % 路径以蓝色线绘制
% plot(x_pos_e4, y_pos_e4, '-g', 'LineWidth', 1.5); % 路径以蓝色线绘制
% 
% 
% hold off;
