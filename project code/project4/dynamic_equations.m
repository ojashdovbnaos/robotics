function tau = dynamic_equations(q, q_dot, m1, m2, L1, L2, r1, r2, I1, I2, g)
    % 声明符号变量
    syms q1 q2 q1_dot q2_dot real
    
    % 关节角度和角速度
    q = [q1; q2];
    q_dot = [q1_dot; q2_dot];
    
    % 计算关节1的动能
    T1 = (1/2) * m1 * (r1^2 * q1_dot^2) + (1/2) * I1 * q1_dot^2;

    % 计算关节2的动能
    T2 = (1/2) * m2 * ((r1 * q1_dot)^2 + (r2 * q2_dot)^2 + 2 * r1 * r2 * q1_dot * q2_dot * cos(q2)) + (1/2) * I2 * q2_dot^2;

    % 计算势能
    V1 = m1 * g * r1 * cos(q1);
    V2 = m2 * g * (r1 * cos(q1) + r2 * cos(q1 + q2));

    % 总动能和总势能
    T = T1 + T2;
    V = V1 + V2;

    % 拉格朗日量 L = T - V
    L = T - V;

    % 计算对q_k的偏导数
    tau1 = diff(L, q1);
    tau2 = diff(L, q2);

    % 返回关节1和关节2的力矩
    tau = [tau1; tau2];
end
