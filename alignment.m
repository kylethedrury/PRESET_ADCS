% THIS IS A SCRIPT FOR PERFORMING ALIGNMENT SIMULATIONS
% AUTHOR: K. DRURY

time = 10000; % specify time frame of simulation 
LTDN = 3;     % speciify LTDN
K = 0.4 / 2;  % gain is at max when q is antiparallel to desired q

[J, q_initial, omega_initial, ECI_pos, ECI_vel, ECEF_pos, ECEF_vel, LLA_pos, LLA_vel, B_ECF, B_ECI, t] = setup(time, LTDN, 0);

R = quat2dcm(q_initial);         % initial DCM matrix for body to ECI 
x = [q_initial; omega_initial];  % initial state 

B_err = zeros(1, t);             % initialize array for plotting B_err

for i = 1:t                      % loop to run the detumble algorithm

    B_desired = get_desired(LTDN, i);       % get the desired field 
    B_body = R \ ECI_field(LTDN, i);        % get the current field 
    B_err(i) = norm(B_desired - B_body);    % get the error in body field                   
    mu = B_desired * K * abs(B_err(i));     % get the control moment
    mu_total = mu(1) + mu(2) + mu(3);       % get total moment

    if total_u > 0.4
        mu = mu * (0.4 / total_mu);         % normalize
    end
    
    for j = 0:99                            % call rk4, advancing by 0.1s each time
        xdot = rk4(x, mu, i+0.01*j, LTDN);
        x = x + (xdot * 0.01);
    end

    q = x(1:4);       % get attitude quaternion
    R = quat2dcm(q);  % get DCM from quaternion

end

figure; % Open a new figure

hold on; % Hold the plot to overlay multiple plots on the same axes

plot(t, B_err); % plot B_err over time 

xlabel('Component Index');
ylabel('Value');
title('Body Field Error');

hold off; % Release the hold

