function T = forwardkin(theta1, theta2, theta3, a1, a2, a3)
    % 定义 D-H 参数变换矩阵
    A1 = [cos(theta1), -sin(theta1), 0, a1 * cos(theta1);
          sin(theta1), cos(theta1), 0, a1 * sin(theta1);
          0, 0, 1, 0;
          0, 0, 0, 1];

    A2 = [cos(theta2), -sin(theta2), 0, a2 * cos(theta2);
          sin(theta2), cos(theta2), 0, a2 * sin(theta2);
          0, 0, 1, 0;
          0, 0, 0, 1];

    A3 = [cos(theta3), -sin(theta3), 0, a3 * cos(theta3);
          sin(theta3), cos(theta3), 0, a3 * sin(theta3);
          0, 0, 1, 0;
          0, 0, 0, 1];

    % 计算总变换矩阵
    T = A1 * A2 * A3;
end
