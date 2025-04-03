function cost = objectiveFunc_dis(u, follower_pos, leader_pos, v_L, omega_L, lambda_LF_d, phi_LF_d, Q, R, dt, N)

    v_F = u(1);

    omega_F = u(2);

    

    cost = 0;

    for i = 1:N

        % 计算follower在leader相对位置下的理想位置

        ideal_follower_pos = [leader_pos(1) + lambda_LF_d * cos(leader_pos(3) + phi_LF_d);

                              leader_pos(2) + lambda_LF_d * sin(leader_pos(3) + phi_LF_d);

                              leader_pos(3)];

 

        % 计算位置误差

        e = ideal_follower_pos - follower_pos;

 

        e_dot = [v_L*cos(e(3)) - v_F;

                 v_L*sin(e(3)) - v_F;

                 omega_L - omega_F];

 

        follower_pos = follower_pos + [v_F*cos(follower_pos(3)); v_F*sin(follower_pos(3)); omega_F] * dt;

        leader_pos = leader_pos + [v_L*cos(leader_pos(3)); v_L*sin(leader_pos(3)); omega_L] * dt;

 

        cost = cost + e' * Q * e + u' * R * u;

    end

end