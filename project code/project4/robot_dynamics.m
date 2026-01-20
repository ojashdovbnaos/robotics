function dqdt = robot_dynamics(t, q, m1, m2, L1, L2, r1, r2, I1, I2, g, tau)
    % 确保输入参数为双精度类型
    q = double(q); % 关节角度和角速度
    tau = double(tau); % 力矩输入

    % 提取当前的关节角度和速度
    q1 = q(1);
    q2 = q(2);
    q1_dot = q(3);
    q2_dot = q(4);

    % 计算当前的力矩（这里使用给定的力矩输入，实际应基于动态方程计算）
    tau1 = tau(1);
    tau2 = tau(2);

    % 计算动态方程
    tau_dyn = dynamic_equations([q1; q2], [q1_dot; q2_dot], m1, m2, L1, L2, r1, r2, I1, I2, g);

    % 计算关节加速度
    q1_ddot = (tau1 - tau_dyn(1)) / I1; % 简单的模型假设
    q2_ddot = (tau2 - tau_dyn(2)) / I2;

    % 返回状态导数
    dqdt = [q1_dot; q2_dot; q1_ddot; q2_ddot];
end
