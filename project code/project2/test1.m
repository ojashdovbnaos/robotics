
theta1=10
theta2=20
theta3=30

T1 = forwardkin(theta1, theta2, theta3);
% q1_calculated = invkin(T1);
J1 = Jacob(q1);
% qdot = [0.1; 0.2]
% v1 = J1 * qdot;
disp(T1)

function T = forwardkin(theta1, theta2, theta3)
    % 定义 D-H 参数中的扭转角 α 直接使用度数
    alpha1 = 90;
    alpha2 = 0;
    alpha3 = 0;

    % 定义 D-H 参数变换矩阵，使用 sind 和 cosd 处理角度
    A1 = [cosd(theta1), -sind(theta1) * cosd(alpha1), sind(theta1) * sind(alpha1), 0;
          sind(theta1), cosd(theta1) * cosd(alpha1), -cosd(theta1) * sind(alpha1), 0;
          0, sind(alpha1), cosd(alpha1), 50;
          0, 0, 0, 1];

    A2 = [cosd(theta2), -sind(theta2) * cosd(alpha2), sind(theta2) * sind(alpha2), 40 * cosd(theta2);
          sind(theta2), cosd(theta2) * cosd(alpha2), -cosd(theta2) * sind(alpha2), 40 * sind(theta2);
          0, sind(alpha2), cosd(alpha2), 0;
          0, 0, 0, 1];

    A3 = [cosd(theta3), -sind(theta3) * cosd(alpha3), sind(theta3) * sind(alpha3), 30 * cosd(theta3);
          sind(theta3), cosd(theta3) * cosd(alpha3), -cosd(theta3) * sind(alpha3), 30 * sind(theta3);
          0, sind(alpha3), cosd(alpha3), 0;
          0, 0, 0, 1];

    % 计算总变换矩阵
    T = A1 * A2 * A3;
end


