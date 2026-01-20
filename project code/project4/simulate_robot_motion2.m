% 主函数
function simulate_robot_motion2()
    % 初始条件（确保为双精度）
    q_init = [pi/4; pi/6];  % 初始关节角度 (radians)
    q_dot_init = [0.1; 0.2];  % 初始关节角速度 (rad/s)
    m1 = 2; % 第一个连杆质量 (kg)
    m2 = 1; % 第二个连杆质量 (kg)
    L1 = 0.4; % 第一个连杆长度 (m)
    L2 = 0.3; % 第二个连杆长度 (m)
    r1 = L1 / 2; % 第一个连杆质心位置 (m)
    r2 = L2 / 2; % 第二个连杆质心位置 (m)
    I1 = (1/12) * m1 * L1^2; % 第一个连杆惯性矩 (kg·m²)
    I2 = (1/12) * m2 * L2^2; % 第二个连杆惯性矩 (kg·m²)
    g = 9.81; % 重力加速度 (m/s^2)

    % 定义力矩输入（可改为动态输入函数）
    tau_input = @(t) [0.1; 0.2];  % 力矩输入函数 (Nm)

    % 创建初始状态向量 [q1, q2, q1_dot, q2_dot] （确保为双精度）
    init_conditions = double([q_init; q_dot_init]);

    % 使用 ode45 求解机器人运动方程
    [T, Q] = ode45(@(t, q) robot_dynamics(t, q, m1, m2, L1, L2, r1, r2, I1, I2, g, tau_input(t)), [0 10], init_conditions);

    % 提取角度和角速度
    q1 = Q(:, 1);
    q2 = Q(:, 2);
    q1_dot = Q(:, 3);
    q2_dot = Q(:, 4);

    % 绘制结果
    figure;
    subplot(2, 1, 1);
    plot(T, q1, T, q2);
    title('关节位置随时间的变化');
    xlabel('时间 (s)');
    ylabel('关节角度 (rad)');
    legend('q1', 'q2');

    subplot(2, 1, 2);
    plot(T, q1_dot, T, q2_dot);
    title('关节速度随时间的变化');
    xlabel('时间 (s)');
    ylabel('关节角速度 (rad/s)');
    legend('q1_dot', 'q2_dot');
end

% 机器人动力学方程
function dqdt = robot_dynamics(t, q, m1, m2, L1, L2, r1, r2, I1, I2, g, tau)
    % 确保所有输入都为双精度
    q = double(q); % 关节角度和角速度
    tau = double(tau); % 力矩输入

    % 提取当前的关节角度和速度
    q1 = q(1);
    q2 = q(2);
    q1_dot = q(3);
    q2_dot = q(4);

    % 动力学方程的惯性矩阵
    M = [I1 + m1*r1^2 + m2*(L1^2 + r2^2 + 2*L1*r2*cos(q2)), m2*(r2^2 + L1*r2*cos(q2));
         m2*(r2^2 + L1*r2*cos(q2)), I2 + m2*r2^2];
    
    % 科里奥利和离心力矩
    C = [-m2*L1*r2*sin(q2)*q2_dot, -m2*L1*r2*sin(q2)*(q1_dot + q2_dot);
         m2*L1*r2*sin(q2)*q1_dot, 0];
    
    % 重力矩
    G = [m1*g*r1*cos(q1) + m2*g*(L1*cos(q1) + r2*cos(q1 + q2));
         m2*g*r2*cos(q1 + q2)];
    
    % 关节加速度
    q_ddot = M \ (tau - C*[q1_dot; q2_dot] - G);

    % 返回状态导数
    dqdt = [q1_dot; q2_dot; q_ddot];
end
