% 定义符号变量 q1, q2, q1_dot, q2_dot
syms q1 q2 q1_dot q2_dot real

% 关节角度和角速度
q = [pi/4; pi/6]; % 关节角度 (radians)
q_dot = [0.1; 0.2]; % 关节角速度 (rad/s)

% 机械臂的物理参数
m1 = 2; % 第一个连杆质量 (kg)
m2 = 1; % 第二个连杆质量 (kg)
L1 = 40; % 第一个连杆长度 (cm)
L2 = 30; % 第二个连杆长度 (cm)
r1 = L1 / 2; % 第一个连杆质心位置 (cm)
r2 = L2 / 2; % 第二个连杆质心位置 (cm)
I1 = (1/12) * m1 * L1^2; % 第一个连杆惯性矩 (kg.cm^2)
I2 = (1/12) * m2 * L2^2; % 第二个连杆惯性矩 (kg.cm^2)
g = 9.81; % 重力加速度 (m/s^2)

% 调用动态方程函数
tau = dynamic_equations(q, q_dot, m1, m2, L1, L2, r1, r2, I1, I2, g);

% 显示力矩
disp('关节力矩:');
disp(tau);
