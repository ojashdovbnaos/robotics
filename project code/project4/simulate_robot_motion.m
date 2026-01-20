function simulate_robot_motion()
    % Initial conditions (ensure double precision)
    q_init = [pi/5; pi/5];  % Initial joint angles (radians)
    q_dot_init = [0.1; 0.2];  % Initial joint angular velocities (rad/s)
    L1 = 0.4; % Length of the first link (m)
    L2 = 0.3; % Length of the second link (m)

    % Define target trajectory (desired angles and velocities increasing linearly)
    q_desired = @(t) [pi/6 + 0.05 * t; pi/8 + 0.03 * t]; % Desired joint angles
    q_dot_desired = @(t) [0.05; 0.03]; % Desired joint angular velocities
    q_ddot_desired = @(t) [0; 0]; % Desired joint accelerations

    % Initial state vector [q1, q2, q1_dot, q2_dot]
    init_conditions = double([q_init; q_dot_init]);

    % Solve robot motion equations using ode45
    [T, Q] = ode45(@(t, q) robot_dynamics_with_control(t, q, L1, L2, q_desired(t), q_dot_desired(t), q_ddot_desired(t)), [0 10], init_conditions);

    % Extract angles and angular velocities
    q1 = Q(:, 1);
    q2 = Q(:, 2);
    q1_dot = Q(:, 3);
    q2_dot = Q(:, 4);

    % Compute end-effector position and velocity
    x = L1 * cos(q1) + L2 * cos(q1 + q2);
    y = L1 * sin(q1) + L2 * sin(q1 + q2);

    x_dot = -L1 * sin(q1) .* q1_dot - L2 * sin(q1 + q2) .* (q1_dot + q2_dot);
    y_dot = L1 * cos(q1) .* q1_dot + L2 * cos(q1 + q2) .* (q1_dot + q2_dot);

    % Plot results
    figure;
    subplot(2, 1, 1);
    plot(T, x, T, y);
    title('End-Effector Position vs Time');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('x', 'y');

    subplot(2, 1, 2);
    plot(T, x_dot, T, y_dot);
    title('End-Effector Velocity vs Time');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('x\_dot', 'y\_dot');
end

% Robot dynamics equation including controller
function dqdt = robot_dynamics_with_control(t, q, L1, L2, q_desired, q_dot_desired, q_ddot_desired)
    % Current state
    q1 = q(1);
    q2 = q(2);
    q1_dot = q(3);
    q2_dot = q(4);

    % Define PD controller gains
    Kp = diag([50, 50]); % Proportional gain
    Kd = diag([5, 5]); % Derivative gain

    % Error computation
    error = [q1; q2] - q_desired;
    error_dot = [q1_dot; q2_dot] - q_dot_desired;

    % Control torque (PD Controller)
    tau = -Kp * error - Kd * error_dot;

    % Inertia matrix for dynamics (simplified assumptions)
    M = [1, 0; 0, 1]; % Simplified inertia matrix
    C = [0, 0; 0, 0]; % Coriolis forces neglected
    G = [0; 0]; % Gravity effects neglected

    % Acceleration calculation
    q_ddot = M \ (tau - C * [q1_dot; q2_dot] - G);

    % Return state derivatives
    dqdt = [q1_dot; q2_dot; q_ddot];
end
