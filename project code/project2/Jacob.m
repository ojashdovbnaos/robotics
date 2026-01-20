function J = Jacob11(q)
    % 参数定义
    L1 = 40;
    L2 = 30;
    
    % 关节角度
    theta1 = q(1);
    theta2 = q(2);

    % 雅可比矩阵
    J11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
    J12 = -L2 * sin(theta1 + theta2);
    J21 = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    J22 = L2 * cos(theta1 + theta2);

    J = [J11, J12; J21, J22];
end
