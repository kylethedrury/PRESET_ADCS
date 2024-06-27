% THIS IS A FUNCTION FOR USING THE EULER METHOD TO SOLVE THE STATE SPACE MODEL 
% AUTHOR: K. DRURY

function xdot = state_space(x, u)   % x state; mu control; time, field, J are necessary params, t is the timestamp

    Jxx = 0.13547588;   % define inertia tensor elements
    Jyy = 0.13542766;
    Jzz = 0.00635730;
    Jyz = -0.00295781;
    Jzy = -0.00295781;
    axx = 7.3813877422313108429;
    ayy = 7.4598182704864598814;
    azz = 158.91202716794601612;
    ayz = 3.4707207012040406363;
    azy = 3.4707207012040406363;

    k = 2 * axx * (Jyy*x(7) - Jzy*x(6) - Jzz*x(7));
    l = 2 * axx * Jyz * x(7);
    m = 2 * ayy * (-Jxx*x(7) + Jzy*x(6) + Jzz*x(7)) + ayz * (Jxx*x(6) - Jyy*x(6) - Jyz*x(7));
    n = 2 * azy * (-Jxx*x(7) + Jzy*x(6) + Jzz*x(7)) + azz * (Jxx*x(6) - Jyy*x(6) - Jyz*x(7));

    A = 1/2* [0,  -x(5),  -x(6), -x(7),  0,   0,   0;   % get the A control matrix
           x(5),     0,  x(7), -x(6),  0,   0,   0; 
           x(6), -x(7),     0,  x(5),  0,   0,   0;
           x(7),  x(6), -x(5),    0,   0,   0,   0;
           0,        0,     0,    0,   0,   k,   l;
           0,        0,     0,    0,   m,   0,   0;
           0,        0,     0,    0,   n,   0,   0];

    B = [ 0,   0,   0;
          0,   0,   0;
          0,   0,   0;
          0,   0,   0;
        axx,   0,   0; 
          0,  ayy,  ayz; 
          0,  azy,  azz];

    xdot = A*x + B*u;                            % solve the state space eqn

end

