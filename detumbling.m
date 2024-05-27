% THIS IS A SCRIPT FOR PERFORMING DETUMBLING SIMULATIONS
% AUTHOR: K. DRURY

% Initialize PRESET inertia tensor and detumble gain constant
J = [0.13614614, 0, 0; 0, 0.13608852, 0.00303665; 0, 0.00303665, 0.00636670];
K = 3.5e-5;

% Define initial conditions 
q0 = [1; 0; 0; 0];
p0 = [0.5; 0.5; 0; 0];

% Combine initial conditions into a single vector
y0 = [q0; p0];

% Define timespan for simulation
tspan = [0, 2,628,288];   % one month in seconds

% Solve the differential equation using ode45
[t, y] = ode45(@(t, y) quaternion_second_order_ode(t, y), tspan, y0);

% Plot the quaternion components over time
figure;
plot(t, q);
legend('q_0', 'q_1', 'q_2', 'q_3');
xlabel('Time (s)');
ylabel('Quaternion components');
title('Quaternion Evolution');

% Plot the first derivative components over time
figure;
plot(t, p);
legend('p_0', 'p_1', 'p_2', 'p_3');
xlabel('Time (s)');
ylabel('First Derivative Components');
title('First Derivative of Quaternion Evolution');

%% Second order derivative of attitude quaternion q, in the case of Bdot control

function dydt = quaternion_second_order_ode(t, y)
    % y(1:4) is q (quaternion)
    % y(5:8) is p (first derivative of quaternion)

    % Extract q and p from y
    q = y(1:4);
    p = y(5:8);

    E = [q(0), q(1), q(2), q(3); -q(1), q(0), q(3), -q(2); -q(2), -q(3), q(0), q(1); -q(3), q(2), -q(1), q(0)];
    E1 = [-q(1), q(0), q(3), -q(2); -u(q), -u(q), u(q), u(q); -u(q), u(q), -u(q), u(q)];

    omega = 2*E*p;
    omega_twiddle = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
    DCM = quat2dcm(q);
    DCM_dot = DCM * omega_twiddle;

    % Define the second derivative of q (ddq)
    Gamma = -K * cross( ( DCM_dot * ECI_field(t) + DCM * ECI_field_derivative(t) ), DCM * ECI_field(t) ); % Bdot torque   
    ddq = -0.5 * E1' \ J * omega_twiddle * J * omega - (p(1)^2 + p(2)^2 + p(3)^2 + p(4)^2) * q + 0.5 * E1' \ J * Gamma;

    % The first-order system
    dqdt = p;
    dpdt = ddq;

    % Combine dqdt and dpdt into a single column vector
    dydt = [dqdt; dpdt];
end

