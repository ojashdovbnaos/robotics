function trajectory2() 
    % Initial conditions (ensure double precision)
    q_init = [pi/5 ; pi/5];  % Initial joint angles (radians)
    q_dot_init = [0.2; 0.1];  % Initial joint angular velocities (rad/s)
    m1 = 2; % Mass of the first link (kg)
    m2 = 1; % Mass of the second link (kg)
    L1 = 0.4; % Length of the first link (m)
    L2 = 0.3; % Length of the second link (m)
    r1 = L1 / 2; % Center of mass position of the first link (m)
    r2 = L2 / 2; % Center of mass position of the second link (m)
    I1 = (1/12) * m1 * L1^2; % Inertia of the first link (kg·m²)
    I2 = (1/12) * m2 * L2^2; % Inertia of the second link (kg·m²)
    g = 9.81; % Gravitational acceleration (m/s^2)

    % Define the target trajectory (linearly increasing desired angles and velocities)
    q_desired = @(t) [pi/6 + 0.05 * t; pi/8 + 0.03 * t]; % Desired joint angles
    q_dot_desired = @(t) [0.05; 0.03]; % Desired joint angular velocities
    q_ddot_desired = @(t) [0; 0]; % Desired joint accelerations

    % Initial state vector [q1, q2, q1_dot, q2_dot]
    init_conditions = double([q_init; q_dot_init]);

    % Solve robot motion equations using ode45
    [T, Q] = ode45(@(t, q) robot_dynamics_with_control(t, q, m1, m2, L1, L2, r1, r2, I1, I2, g, q_desired(t), q_dot_desired(t), q_ddot_desired(t)), [0 10], init_conditions);

    % Extract angles and angular velocities
    q1 = Q(:, 1);
    q2 = Q(:, 2);
    q1_dot = Q(:, 3);
    q2_dot = Q(:, 4);

    % Plot joint positions and velocities
  

    % Compute and plot joint torques
    tau_values = zeros(length(T), 2);
    for i = 1:length(T)
        q_current = Q(i, 1:2)';
        q_dot_current = Q(i, 3:4)';
        error = q_current - q_desired(T(i));
        error_dot = q_dot_current - q_dot_desired(T(i));
        tau_values(i, :) = (-diag([50, 50]) * error - diag([5, 5]) * error_dot)'; % PD Controller
    end

    figure;
    subplot(3, 1, 1);
    plot(T, q1, T, q2);
    title('Joint Positions vs Time');
    xlabel('Time (s)');
    ylabel('Joint Angles (rad)');
    legend('q1', 'q2');

    subplot(3, 1, 2);
    plot(T, q1_dot, T, q2_dot);
    title('Joint Velocities vs Time');
    xlabel('Time (s)');
    ylabel('Joint Angular Velocities (rad/s)');
    legend('q1\_dot', 'q2\_dot');
    subplot(3, 1, 3);
    plot(T, tau_values(:, 1), T, tau_values(:, 2));
    title('Joint Torques vs Time');
    xlabel('Time (s)');
    ylabel('Joint Torques (N·m)');
    legend('τ1', 'τ2');
end

% Robot dynamics equation, including controller
function dqdt = robot_dynamics_with_control(t, q, m1, m2, L1, L2, r1, r2, I1, I2, g, q_desired, q_dot_desired, q_ddot_desired)
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

    % Inertia matrix of the dynamics
    M = [I1 + m1*r1^2 + m2*(L1^2 + r2^2 + 2*L1*r2*cos(q2)), m2*(r2^2 + L1*r2*cos(q2));
         m2*(r2^2 + L1*r2*cos(q2)), I2 + m2*r2^2];
    
    % Coriolis and centrifugal forces
    C = [-m2*L1*r2*sin(q2)*q2_dot, -m2*L1*r2*sin(q2)*(q1_dot + q2_dot);
         m2*L1*r2*sin(q2)*q1_dot, 0];
    
    % Gravitational torques
    G = [m1*g*r1*cos(q1) + m2*g*(L1*cos(q1) + r2*cos(q1 + q2));
         m2*g*r2*cos(q1 + q2)];
    
    % Acceleration calculation
    q_ddot = M \ (tau - C * [q1_dot; q2_dot] - G);

    % Return state derivatives
    dqdt = [q1_dot; q2_dot; q_ddot];
end
