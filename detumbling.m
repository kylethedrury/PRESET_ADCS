% THIS IS A SCRIPT FOR PERFORMING DETUMBLING SIMULATIONS
% AUTHOR: K. DRURY

time = 10000;    % specifiy time window in seconds
LTDN = 3;        % specifiy LTDN
rotation = pi;   % specifiy the norm of the initial rotation
K = 3.5e-5;      % detumble gain constant

[J, q_initial, omega_initial, ECI_pos, ECI_vel, ECEF_pos, ECEF_vel, LLA_pos, LLA_vel, B_ECF, B_ECI, t] = setup(time, LTDN, rotation);

R = quat2dcm(q_initial);  % initial DCM matrix for BF to ECI

B_BF = zeroes(3, t);               % allocate array for storing the magnetic field in BF 
B_BF(1) = R \ ECI_field(LTDN, 1);  % initial body field vector 

x_array = zeroes(7, t);     % allocate array for state vectors
xdot_array = zeroes(7, t);  % allocate array for state derivative vectors 

x_array(1) = [q_initial; omega_initial];       % initial state
mu = [0; 0; 0];                                % initial control vector
xdot_array(1) = rk4(x_array(1), mu, 1, LTDN);  % initial state derivative

for i = 2:t  % A loop to run the detumble algorithm (t in same units as CSVs)
    
    x_array(i) = x_array(i-1); % initially set the next state to the previous one; prepare for numerical integration

    for j = 0:99   % call rk4, advancing by 0.01s each time
        xdot_array(i) = rk4(x_array(i), mu, i-1 + 0.01*j, LTDN);
        x_array(i) = x_array(i) + (xdot_array(i) * 0.01);
    end

    state = x_array(i);                % get the new state
    q = state(1:4);                    % get the new attitude quaternion
    B_ECI = ECI_field(LTDN, i);        % get the new ECI field at current time
    R = quat2dcm(q);                   % get new rotation matrix
    B_BF(i) = R \ B_ECI;               % get new field in BF
    Bdot = B_BF(i-1) - B_BF(i);        % get Bdot; dt = 1s
    mu = -K * Bdot;                    % get control vector 
    total_mu = mu(1) + mu(2) + mu(3);  % get total moment (must ensure we don't exceed max moment)

    if total_mu > 0.4
        mu = mu * (0.4 / total_mu);    % normalize
    end

end 

% Open a new figure
figure;

% Hold the plot to overlay multiple plots on the same axes
hold on;

% Loop through each element
for i = 1:n
    % Extract the i-th 7x1 vector
    current_vector = x_array(:, i);
    
    % Plot each component of the current vector
    plot(1:7, current_vector, 'DisplayName', ['Vector ' num2str(i)]);
end

% Add labels and legend
xlabel('Component Index');
ylabel('Value');
title('Components of 7x1 Vectors');
legend show;

% Release the hold
hold off;

