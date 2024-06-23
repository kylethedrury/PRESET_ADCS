% THIS IS A FUNCTION FOR USING RK4 TO SOLVE THE STATE SPACE MODEL 

function xdot = rk4(x, mu, t, LTDN)

    J = [0.13614614, 0, 0; 0, 0.13608852, 0.00303665; 0, 0.00303665, 0.00636670]; % inertia tensor
    A = inv(J);                                                                   % inverse of tensor 

    Jxx = J(1, 1);
    Jyy = J(2, 2);
    Jzz = J(3, 3);
    Jyz = J(2, 3);
    Jzy = J(3, 2);
    axx = A(1, 1);
    ayy = A(2, 2);
    azz = A(3, 3);
    ayz = A(2, 3);
    azy = A(3, 2);

    k = axx * (Jyy*x(7) - Jzy*x(6) - Jzz*x(7));
    l = axx * Jyz * x(7);
    m = ayy * (-Jxx*x(7) + Jzy*x(6) + Jzz*x(7)) + ayz * (Jxx*x(6) - Jyy*x(6) - Jyz*x(7));
    n = azy * (-Jxx*x(7) + Jzy*x(6) + Jzz*x(7)) + azz * (Jxx*x(6) - Jyy*x(6) - Jyz*x(7));

    A = [0,  -x(5),  -x(6), -x(7),  0,   0,   0;   % get the A control matrix
        x(5),     0,  x(7), -x(6),  0,   0,   0; 
        x(6), -x(7),     0,  x(5),  0,   0,   0;
        x(7),  x(6), -x(5),    0,   0,   0,   0;
        0,        0,     0,    0,   0,   k,   l;
        0,        0,     0,    0,   m,   0,   0;
        0,        0,     0,    0,   n,   0,   0];

    DCM = quat2dcm(x(1), x(2), x(3), x(4));      % get the DCM matrix from the current quaternion

    B_ECI = ECI_field(LTDN, t);                   % get the ECI field at this moment

    skew_B = [        0, -B_ECI(3),  B_ECI(2);    % get the skew symmetric matrix of B_ECI
               B_ECI(3),         0, -B_ECI(1); 
              -B_ECI(2),  B_ECI(1),         0];                                  

    B = [     zeros(4, 3);                        % get the B control matrix
         -A * DCM \ skew_B];

    xdot = A*x + B*mu;                            % solve the state space eqn

end

