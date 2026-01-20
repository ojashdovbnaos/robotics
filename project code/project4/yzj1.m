% 参数值
m1 = 2;  % 关节1的质量 (kg)
m2 = 1;  % 关节2的质量 (kg)
L1 = 1;  % 链接1的长度 (m)
L2 = 0.4;  % 链接2的长度 (m)
r1 = 0.3;  % 链接1的质心 (m)
r2 = 0.3;  % 链接2的质心 (m)
I1 = 0.05;  % 链接1的转动惯量 (kg·m²)
I2 = 0.02;  % 链接2的转动惯量 (kg·m²)
g = 9.81;  % 重力加速度 (m/s²)

% 调用函数
syms q1 q2 q1_dot q2_dot real
q = [q1; q2];
q_dot = [q1_dot; q2_dot];

tau = dynamic_equations(q, q_dot, m1, m2, L1, L2, r1, r2, I1, I2, g);

% 显示结果
disp('joint torque：');
disp(tau);
q1_val = pi/4;  % 45度
q2_val = pi/6;  % 30度
q1_dot_val = 1;  % 1 rad/s
q2_dot_val = 0.5;  % 0.5 rad/s

tau_numeric = subs(tau, {q1, q2, q1_dot, q2_dot}, {q1_val, q2_val, q1_dot_val, q2_dot_val});
disp('numerical torque：');
disp(tau_numeric);
