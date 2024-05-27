syms q0 q1 q2 q3 q0dot q1dot q2dot q3dot axx ayy azz ayz azy Ixx Iyy Izz Iyz Izy

% Define E matrix
E = [-q1, q0, q3, -q2; 
     -q2, q3, q0, q1; 
     -q3, q2, -q1, q0];

ET = [-q1, -q2, -q3; 
      q0, q3, q2; 
      q3, q0, q1; 
      -q2, q1, q0];

% Define A matrix
%A = [axx, 0, 0; 
%     0, ayy, ayz; 
%     0, azy, azz];

A = [axx, 0, 0; 
     0, ayy, 0; 
     0, 0, azz];

% Define I matrix
%I = [Ixx, 0, 0; 
%     0, Iyy, Iyz; 
%     0, Izy, Izz];

I = [Ixx, 0, 0; 
     0, Iyy, 0; 
     0, 0, Izz];

% Define omega vector
omega1 = -q1*q0dot + q0*q1dot + q3*q2dot - q2*q3dot;
omega2 = -q2*q0dot + q3*q1dot + q0*q2dot + q1*q3dot;
omega3 = -q3*q0dot + q2*q1dot - q1*q2dot - q0*q3dot;

omega = [omega1; omega2; omega3];

omega_skew = [ 0    -omega(3)  omega(2);
             omega(3)  0     -omega(1);
             -omega(2)  omega(1)  0  ];




disp(-0.5 * ET * A * omega_skew * I * omega);

