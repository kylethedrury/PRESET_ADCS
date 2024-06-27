% THIS IS A SCRIPT FOR PERFORMING DETUMBLING SIMULATIONS-- be sure to add /Parameters and /Functions to path
% AUTHOR: K. DRURY

%% SETUP

B = 4.5e-5;        % average field
w = pi/4;          % expected tip-off rate
mu_max = 0.45;     % max moment with z-TQR
K = mu_max/(B*w);  % tuned detumble gain

endpoint = 86400*100;  % stop point in seconds (currently set to 1 day-- *100 to convert to 0.01s)
LTDN = 3;              % specifiy LTDN

omega_initial = [0; pi/4; 0]; % initial omega 
q_initial = [1; 0; 0; 0];     % initial q

[time, position, velocity, field] = get_data(LTDN);  % get data; it takes about 15-20 mins for data to be loaded in the 0.01s case

%% INITIALIZE AND ALLOCATE ARRAYS

timestamps = 0:1:endpoint-1;        % initialize an array of timestamps from 0 to 'endpoint' incrementing by 1s each time
B_BF = zeros(3, endpoint-1);        % allocate array for storing the magnetic field in BF; 3 rows, N columns
x_array = zeros(7, endpoint-1);     % allocate array for state vectors; 
xdot_array = zeros(7, endpoint-1);  % allocate array for state derivative vectors 

x_array(:,1) = [q_initial; omega_initial];       % initial state
u = [0; 0; 0];                                   % initial control vector
DCM = quat2dcm(q_initial');                      % initial DCM matrix for BF to ECI
B_BF(:,1) = DCM \ field(:,1);                    % initial body field vector 
xdot_array(:,1) = state_space(x_array(:,1), u);  % initial state derivative

%% SIMULATION

for i = 2:endpoint  % A loop to run the detumble algorithm (steps of 1s)
    
    x_array(:,i) = x_array(:,i-1) + (xdot_array(:,i-1)*0.01);  % get new state
    q = x_array(1:4,i) / norm(x_array(1:4,i));                 % get the new attitude quaternion and normalize it
    x_array(:,i) = [q; x_array(5:7,i)];                        % save the new state w normalized quaternion
    DCM = quat2dcm(q');                                        % get new DCM
    B_BF(:,i) = DCM \ field(:,i);                              % get new field in BF
    
    Bdot = (B_BF(:,i) - B_BF(:,i-1))/0.01;  % get Bdot; dt = 0.01s
    mu = -K * Bdot;                         % get control vector 
    total_mu = mu(1) + mu(2) + mu(3);       % get total moment (must ensure we don't exceed max moment)

    if total_mu > mu_max
        mu = mu * (mu_max / total_mu); % normalize
    end

    u = cross(mu, B_BF(:,i));  % get the control torque in BF

    xdot_array(:,i) = state_space(x_array(:,i), u); % get xdot for calculating the next state on next loop

    disp(i/endpoint) % Loading bar
    %disp(x_array(:,i)')
    %disp(xdot_array(:,i)') 
    %disp(mu') 

    if all(isnan(xdot_array(:,i)))
        disp('Vector xdot contains only NaN values. Stopping the script.');
        break; % Exit the loop if a NaN occurs
    end

end 

%% Plot

% Open a new figure
figure;

% Hold the plot to overlay multiple plots on the same axes
hold on;

% Extract omega for plotting
omega_x = x_array(5, :);
omega_y = x_array(6, :);
omega_z = x_array(7, :);

plot(timestamps, omega_x, 'r', 'DisplayName', 'wx'); 
plot(timestamps, omega_y, 'g', 'DisplayName', 'wy');
plot(timestamps, omega_z, 'b', 'DisplayName', 'wz');

% Add labels and legend
xlabel('Time');
ylabel('Omega');
title('Detumbling');
legend show;

% Display grid
grid on;

