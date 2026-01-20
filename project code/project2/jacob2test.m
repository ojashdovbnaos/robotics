q1 = [10, 20, 30];


J1 = Jacob(q1);
% 定义关节角度和关节角速度       % 当前关节角度（可以是任意值）
qdot = [0.1; 0.2; 0.3];   % 各关节角速度


% 计算末端执行器的速度
v = J1* qdot;

% 输出末端速度
disp('末端执行器的速度向量:');
disp(v);

function J = Jacob(q)
    % 已知常量
    a1 = 50; % 连杆长度 a1
    a2 = 40; % 连杆长度 a2
    a3 = 30; % 连杆长度 a3

    % 关节角度
    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);

    % 使用正向运动学计算各关节的位姿
    % 末端执行器位置
    x = a2 * cosd(theta1) * cosd(theta2) + a3 * cosd(theta1) * cosd(theta2 + theta3);
    y = a2 * sind(theta1) * cosd(theta2) + a3 * sind(theta1) * cosd(theta2 + theta3);
    z = a1 + a2 * sind(theta2) + a3 * sind(theta2 + theta3);

    % 计算雅可比矩阵的每一列
    % 对 theta1 的偏导数
    J1 = [-a2 * sind(theta1) * cosd(theta2) - a3 * sind(theta1) * cosd(theta2 + theta3);
           a2 * cosd(theta1) * cosd(theta2) + a3 * cosd(theta1) * cosd(theta2 + theta3);
           0];

    % 对 theta2 的偏导数
    J2 = [-a2 * cosd(theta1) * sind(theta2) - a3 * cosd(theta1) * sind(theta2 + theta3);
          -a2 * sind(theta1) * sind(theta2) - a3 * sind(theta1) * sind(theta2 + theta3);
           a2 * cosd(theta2) + a3 * cosd(theta2 + theta3)];

    % 对 theta3 的偏导数
    J3 = [-a3 * cosd(theta1) * sind(theta2 + theta3);
          -a3 * sind(theta1) * sind(theta2 + theta3);
           a3 * cosd(theta2 + theta3)];

    % 构建雅可比矩阵
    J = [J1, J2, J3];
end
