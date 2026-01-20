function trajectory()
    % 初始条件
    q_init = [pi/4; pi/6];  % 初始关节角度 (rad)
    q_dot_init = [0.1; 0.2];  % 初始关节角速度 (rad/s)
    q_final = [pi/3; pi/4];  % 最终关节角度 (rad)
    q_dot_final = [0; 0];    % 最终关节角速度 (rad/s)
    q_ddot_init = [0; 0];    % 初始关节加速度 (rad/s^2)
    q_ddot_final = [0; 0];   % 最终关节加速度 (rad/s^2)
    t0 = 0; tf = 10;         % 时间范围

    % 动力学参数
    m1 = 2; m2 = 1;  % 连杆质量 (kg)
    L1 = 0.4; L2 = 0.3;  % 连杆长度 (m)
    r1 = L1 / 2; r2 = L2 / 2;  % 连杆质心位置
    I1 = (1/12) * m1 * L1^2; % 惯性矩
    I2 = (1/12) * m2 * L2^2;
    g = 9.81; % 重力加速度

    % 五次多项式轨迹规划
    a = quintic_trajectory(t0, tf, q_init, q_dot_init, q_ddot_init, q_final, q_dot_final, q_ddot_final);

    % 期望轨迹函数
    q_desired = @(t) a(:, 1) + a(:, 2).*t + a(:, 3).*t.^2 + a(:, 4).*t.^3 + a(:, 5).*t.^4 + a(:, 6).*t.^5;
    q_dot_desired = @(t) a(:, 2) + 2*a(:, 3).*t + 3*a(:, 4).*t.^2 + 4*a(:, 5).*t.^3 + 5*a(:, 6).*t.^4;
    q_ddot_desired = @(t) 2*a(:, 3) + 6*a(:, 4).*t + 12*a(:, 5).*t.^2 + 20*a(:, 6).*t.^3;

    % 初始状态
    init_conditions = [q_init; q_dot_init];

    % 数值积分设置
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [T, Q] = ode45(@(t, q) robot_dynamics_with_control(t, q, m1, m2, L1, L2, r1, r2, I1, I2, g, ...
                                                       q_desired(t), q_dot_desired(t), q_ddot_desired(t)), ...
               [t0 tf], init_conditions, options);

    % 提取关节状态
    q1 = Q(:, 1); q2 = Q(:, 2);
    q1_dot = Q(:, 3); q2_dot = Q(:, 4);

    % 计算期望轨迹
    q_desired_eval = arrayfun(@(t) q_desired(t), T, 'UniformOutput', false);
    q_desired_eval = cell2mat(q_desired_eval')';

    % 绘制结果
    figure;

    subplot(3, 1, 1);
    plot(T, q1, T, q2);
    title('关节位置随时间变化');
    xlabel('时间 (s)');
    ylabel('关节角度 (rad)');
    legend('q1', 'q2');

    subplot(3, 1, 2);
    plot(T, q1_dot, T, q2_dot);
    title('关节速度随时间变化');
    xlabel('时间 (s)');
    ylabel('关节角速度 (rad/s)');
    legend('q1\_dot', 'q2\_dot');

    subplot(3, 1, 3);
    plot(T, q_desired_eval(:, 1) - q1, T, q_desired_eval(:, 2) - q2);
    title('轨迹跟踪误差');
    xlabel('时间 (s)');
    ylabel('误差 (rad)');
    legend('误差_q1', '误差_q2');
end


function a = quintic_trajectory(t0, tf, q0, v0, a0, qf, vf, af)
    % 解五次多项式系数
    A = [1 t0 t0^2 t0^3 t0^4 t0^5;
         0 1  2*t0 3*t0^2 4*t0^3 5*t0^4;
         0 0  2     6*t0   12*t0^2 20*t0^3;
         1 tf tf^2 tf^3 tf^4 tf^5;
         0 1  2*tf 3*tf^2 4*tf^3 5*tf^4;
         0 0  2     6*tf   12*tf^2 20*tf^3];
    b1 = [q0(1); v0(1); a0(1); qf(1); vf(1); af(1)];
    b2 = [q0(2); v0(2); a0(2); qf(2); vf(2); af(2)];
    coeffs1 = A \ b1;
    coeffs2 = A \ b2;
    a = [coeffs1'; coeffs2'];
end

function dqdt = robot_dynamics_with_control(t, q, m1, m2, L1, L2, r1, r2, I1, I2, g, q_desired, q_dot_desired, q_ddot_desired)
    % 当前状态
    q1 = q(1); q2 = q(2);
    q1_dot = q(3); q2_dot = q(4);

    % 动力学矩阵
    M = [I1 + m1*r1^2 + m2*(L1^2 + r2^2 + 2*L1*r2*cos(q2)), m2*(r2^2 + L1*r2*cos(q2));
         m2*(r2^2 + L1*r2*cos(q2)), I2 + m2*r2^2];
    C = [-m2*L1*r2*sin(q2)*q2_dot, -m2*L1*r2*sin(q2)*(q1_dot + q2_dot);
          m2*L1*r2*sin(q2)*q1_dot, 0];
    G = [m1*g*r1*cos(q1) + m2*g*(L1*cos(q1) + r2*cos(q1 + q2));
         m2*g*r2*cos(q1 + q2)];

    % 控制器增益
    Kp = diag([50, 50]);
    Kd = diag([5, 5]);

    % 误差
    error = [q1; q2] - q_desired;
    error_dot = [q1_dot; q2_dot] - q_dot_desired;

    % 控制力矩
    tau = M * q_ddot_desired + C * q_dot_desired + G + Kp * error + Kd * error_dot;

    % 加速度计算
    q_ddot = M \ (tau - C * [q1_dot; q2_dot] - G);

    % 状态导数
    dqdt = [q1_dot; q2_dot; q_ddot];
end
