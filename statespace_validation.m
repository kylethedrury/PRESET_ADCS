%% DEFINE INITIAL CONDITIONS, INERTIA TENSOR

% Body axes are initially aligned with inertial axes 
q0 = 1;  
q1 = 0;    
q2 = 0;   
q3 = 0;   

% Spin initially about y-axis 
omega_x = 0;   
omega_y = 1;   
omega_z = 0;   

% Define inertia tensor
Ixx = 0.13614614;
Iyy = 0.13608852;
Izz = 0.00636670;
Iyz = -0.00303665;
Izy = -0.00303665;

% Define inertia tensor inverse
axx = 7.3450484898066151563;
ayy = 7.4272042201116417278;
ayz = 3.5424662219049141394;
azy = 3.5424662219049141394;
azz = 158.75684892530629015;

% Initial state vector
x0 = [q0; q1; q2; q3; omega_x; omega_y; omega_z];

% Control vector (no control)
u = @(t) [0*t; 1; 0*t];

%% IN HOUSE STATE SPACE MODEL

% Define state-dependant matrices A and B
A = @(x) [
 0  -0.5*x(5)  -0.5*x(6)  -0.5*x(7)  -0.5*x(2)  -0.5*x(3)  -0.5*x(4);
 0.5*x(5)  0  0.5*x(7)  -0.5*x(6)  0.5*x(1)  -0.5*x(4)  0.5*x(3); 
 0.5*x(6)  -0.5*x(7)  0  0.5*x(5)  0.5*x(4)  0.5*x(1)  -0.5*x(2);
 0.5*x(7)  0.5*x(6)  -0.5*x(5)  0  -0.5*x(3)  0.5*x(2)  0.5*x(1);
 0  0  0  0  0  axx*(Iyy*x(3) - 2*Izy*x(6) - Izz*x(7))  axx*(Iyy*x(6) + 2*Iyz*x(7) - Izz*x(6));
 0  0  0  0  ayy*(-Ixx*x(7) + Izy*x(6) + Izz*x(7)) + ayz*(Ixx*x(6) - Iyy*x(6) - Iyz*x(7))  ayy*Izy*x(5) + ayz*x(5)*(Ixx - Iyy)  ayy*x(5)*(Izz - Ixx) - ayz*Iyz*x(5);
 0  0  0  0  azy*(-Ixx*x(7) + Izy*x(6) + Izz*x(7)) + azz*(Ixx*x(6) - Iyy*x(6) - Iyz*x(7))  azy*Izy*x(5) + azz*x(5)*(Ixx - Iyy)  azy*x(5)*(Izz - Ixx) - azz*Iyz*x(5)
 ];

B = [0 0 0;
     0 0 0;
     0 0 0;
     0 0 0; 
     axx 0 0;
     0 ayy ayz;
     0 azy azz];

stateSpaceModel = @(t,x) A(x)*x + B*u(t);

% Set time span for simulation
tspan = [0 20];

% Solve the state-space model using ode45
[t, x] = ode45(stateSpaceModel, tspan, x0);

% Plot the results
figure;
plot(t, x);
xlabel('Time (s)');
ylabel('States');
legend('x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7');
title('State Evolution with State-Dependent Matrices');
grid on;

%% STATE SPACE MODEL FROM YAGUANG

A = @(x) 0.5*[
    0     -x(5)  -x(6)  -x(7)  0  0  0;
    x(5)  0      x(7)   -x(6)  0  0  0;
    x(6)  -x(7)  0      x(5)   0  0  0;
    x(7)  x(6)   -x(5)  0      0  0  0;
    0     0      0      0      0  0  0;
    0     0      0      0      0  0  0;
    0     0      0      0      0  0  0;
    ];

B = [
    0  0  0;
    0  0  0;
    0  0  0;
    0  0  0;
    axx  0  0;
    0  ayy  ayz;
    0  azy  azz
    ];

% Function to renormalize the quaternion
normalize = @(x) [x(1:4) / norm(x(1:4)); x(5:7)];

stateSpaceModel = @(t,x) A(normalize(x))*normalize(x) + B*u(t);

% Set time span for simulation
tspan = [0 20];

% Solve the state-space model using ode45
[t, x] = ode45(stateSpaceModel, tspan, x0);

% Plot the results
figure;
plot(t, x);
xlabel('Time (s)');
ylabel('States');
legend('x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7');
title('State Evolution with State-Dependent Matrices');
grid on;

%%
disp(x);

