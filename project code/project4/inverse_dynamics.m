function inverse_dynamics()
    % Manipulator parameters
    m1 = 2; m2 = 1; % Link masses (kg)
    l1 = 0.4; l2 = 0.3; % Link lengths (m)
    lc1 = l1 / 2; lc2 = l2 / 2; % Centers of mass (m)
    I1 = (1/12) * m1 * l1^2; I2 = (1/12) * m2 * l2^2; % Inertia
    g = 9.81; % Gravity (m/s^2)

    % PD gains
    Kp = diag([5, 3]);
    Kd = diag([2, 1]);

    % Initial conditions [q1, q2, q1_dot, q2_dot]
    q_init = [pi/5; pi/5];
    q_dot_init = [0.2; 0.1];
    init_conditions = [q_init; q_dot_init];

    % Simulation time
    tspan = [0 10];

    % Solve using ODE45
    [T, Q] = ode45(@(t, q) dynamics(t, q, m1, m2, l1, l2, lc1, lc2, I1, I2, g, Kp, Kd), tspan, init_conditions);

    % Plot results
    figure;
    subplot(2, 1, 1);
    plot(T, Q(:, 1), T, Q(:, 2));
    title('Joint Positions');
    xlabel('Time (s)');
    ylabel('Joint Angles (rad)');
    legend('q1', 'q2');

    subplot(2, 1, 2);
    plot(T, Q(:, 3), T, Q(:, 4));
    title('Joint Velocities');
    xlabel('Time (s)');
    ylabel('Joint Velocities (rad/s)');
    legend('q1\_dot', 'q2\_dot');
end

function dqdt = dynamics(t, q, m1, m2, l1, l2, lc1, lc2, I1, I2, g, Kp, Kd)
    % Extract states
    q1 = q(1); q2 = q(2);
    q1_dot = q(3); q2_dot = q(4);

    % Desired trajectory
    q1d = (pi/4) * cos(t);
    q2d = (pi/6) * cos(2*t + pi/4);
    q1d_dot = -(pi/4) * sin(t);
    q2d_dot = -(pi/3) * sin(2*t + pi/4);
    q1d_ddot = -(pi/4) * cos(t);
    q2d_ddot = -(2*pi/3) * cos(2*t + pi/4);

    % Errors
    e = [q1 - q1d; q2 - q2d];
    e_dot = [q1_dot - q1d_dot; q2_dot - q2d_dot];

    % Inertia matrix D(q)
    D = [I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)), I2 + m2*(lc2^2 + l1*lc2*cos(q2));
         I2 + m2*(lc2^2 + l1*lc2*cos(q2)), I2 + m2*lc2^2];

    % Coriolis and centrifugal matrix C(q, q_dot)
    C = [-m2*l1*lc2*sin(q2)*q2_dot, -m2*l1*lc2*sin(q2)*(q1_dot + q2_dot);
          m2*l1*lc2*sin(q2)*q1_dot, 0];

    % Gravity vector G(q)
    G = [m1*g*lc1*cos(q1) + m2*g*(l1*cos(q1) + lc2*cos(q1 + q2));
         m2*g*lc2*cos(q1 + q2)];

    % Control input (inverse dynamics)
    qd_ddot = [q1d_ddot; q2d_ddot];
    tau = D * (qd_ddot - Kd * e_dot - Kp * e) + C * [q1_dot; q2_dot] + G;

    % Joint accelerations
    q_ddot = D \ (tau - C * [q1_dot; q2_dot] - G);

    % State derivatives
    dqdt = [q1_dot; q2_dot; q_ddot];
end
