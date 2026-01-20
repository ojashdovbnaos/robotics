function q = invkin(x, y, z, a1, a2, a3)
    % 计算 theta3
    cos_theta3 = (x^2 + y^2 - a1^2 - a2^2 - a3^2) / (2 * a2 * a3);
    theta3 = atan2(sqrt(1 - cos_theta3^2), cos_theta3);  % 假设正解

    % 计算 theta1 和 theta2
    theta1 = atan2(y, x) - atan2(a3 * sin(theta3), a1 + a2 * cos(theta3));
    theta2 = atan2(y, x) - theta1;

    % 返回关节角度
    q = [theta1; theta2; theta3];
end
