% PRESET ATTITUDE PROPOGATOR

% Must map the orbital frame to the rotated body frame using the above variables
% To do this, we first define the Euler axis and the rotation angle of initial attitude for easier visualization 

syms e1 e2 e3 theta % components of the Euler axis and rotation angle 
syms w1 w2 w3 % angular rate in the orbital frame for easier visualization
syms Ix Iy Iz 

euler_axis = [e1, e2, e3]; % euler axis in orbital frame MUST BE NORM 1

q1_O = e1*sin(theta/2); % attitude quaternion in orbital frame
q2_O = e2*sin(theta/2);
q3_O = e3*sin(theta/2);
q4_O = cos(theta/2);

%% calculate the DCM matrix in quaternion form for O to B frame

O_to_B = [q1_O^2-q2_O^2-q3_O^2+q4_O^2, 2*(q1_O*q2_O+q3_O*q4_O), 2*(q1_O*q3_O-q2_O*q4_O); ...
    2*(q1_O*q2_O-q3_O*q4_O), -q1_O^2+q2_O^2-q3_O^2+q4_O^2, 2*(q2_O*q3_O+q1_O*q4_O); ...
    2*(q3_O*q1_O+q2_O*q4_O), 2*(q3_O*q2_O-q1_O*q4_O), -q1_O^2-q2_O^2+q3_O^2+q4_O^2];

ECEF_to_B = ECEF_to_O * O_to_B; % ECEF to B matrix; we will have ECEF_to_O from the "ADCS_Frame_Switcher" script 
B_to_ECEF = inv(ECEF_to_B); % B to ECEF matrix
ECI_to_B = ECI_to_O * O_to_B; % ECI to B matrix; we will have ECI_to_O from the "ADCS_Frame_Switcher" script
B_to_ECI = inv(ECI_to_B); % B to ECI matrix

syms mu_x mu_y mu_z % the dipole moment components being produced in the body frame (TBD in a different script)

mu_B = [mu_x, mu_y, mu_z]; % total dipole moment in body frame
mu_ECI = B_to_ECI * mu_B;

algorithm_Torque_ECI = cross(mu_ECI, B_ECI); % B_ECI is known from "ADCS_Frame_Switcher"
disturbance_Torque_ECI = ______ % TBD in another script
Torque_ECI = algorithm_Torque_ECI + disturbance_torque_ECI; % total torque in ECI
Tx_ECI = Torque_ECI(1);
Ty_ECI = Torque_ECI(2); % Torque components in ECI
Tz_ECI = Torque_ECI(3);

%% now we need to get the attitude quaternion that represents the body frame in the ECI frame
% refer to eqn 2.135 in Markley and Crassidis

q1 = ECI_to_B(2,3) - ECI_to_B(3,2); % get components of the quaternion from the matrix 
q2 = ECI_to_B(3,1) - ECI_to_B(1,3);
q3 = ECI_to_B(1,2) - ECI_to_B(2,1);
q4 = 1 + trace(ECI_to_B);

q1_ECI = q1 / norm([q1, q2, q3, q4]); % normalize the components
q2_ECI = q2 / norm([q1, q2, q3, q4]);
q3_ECI = q3 / norm([q1, q2, q3, q4]);
q4_ECI = q4 / norm([q1, q2, q3, q4]);

q_ECI = [q1, q2, q3, q4]; % define the ECI quaternion

%% Solve the quaternion equations iteratively 

q_dot = mtimes(q_ECI, [0, wx, wy, wz]);
wx_dot = Tx_ECI - (I)