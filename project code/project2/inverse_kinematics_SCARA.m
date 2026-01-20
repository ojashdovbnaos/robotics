clear all; close all; clc;

T1=[0.633022221559489	-0.754406506735489	0.173648177666930	56.0073297827176;
0.111618897048950	-0.133022221559489	-0.984807753012208	9.87560335812988;
0.766044443118978	0.642787609686539	0	86.6621390265961;
0	0	0	1]

[theta1, theta2, theta3] = invkin(T1)


function [theta1, theta2, theta3] = invkin(T)
    % 已知常量
    a1 = 50; % 连杆长度 a1
    a2 = 40; % 连杆长度 a2
    a3 = 30; % 连杆长度 a3

    % 提取目标位置
    px = T(1, 4);
    py = T(2, 4);
    pz = T(3, 4);

    % 计算 theta1
    theta1 = atan2d(py, px);

    % 计算 theta2 和 theta3
    r = sqrt(px^2 + py^2); % 末端在 x-y 平面的投影距离
    s = pz - a1;           % 末端在 z 方向上的位移

    % 使用余弦定理计算 theta3
    D = (r^2 + s^2 - a2^2 - a3^2) / (2 * a2 * a3);
    theta3 = atan2d(sqrt(1 - D^2), D); % 两个解之一，您可以根据需要选择不同解

    % 计算 theta2
    theta2 = atan2d(s, r) - atan2d(a3 * sind(theta3), a2 + a3 * cosd(theta3));
end
