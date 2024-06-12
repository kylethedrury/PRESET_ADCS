%% DEFINE INITIAL CONDITIONS, INERTIA TENSOR
clear;
close;
clc;


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
u = @(t) [0; 0; 0];

% IN HOUSE STATE SPACE MODEL

% Define state-dependant matrices A and B (Original Matrix)
% A = @(x) [
%  0  -0.5*x(5)  -0.5*x(6)  -0.5*x(7)  -0.5*x(2)  -0.5*x(3)  -0.5*x(4);
%  0.5*x(5)  0  0.5*x(7)  -0.5*x(6)  0.5*x(1)  -0.5*x(4)  0.5*x(3); 
%  0.5*x(6)  -0.5*x(7)  0  0.5*x(5)  0.5*x(4)  0.5*x(1)  -0.5*x(2);
%  0.5*x(7)  0.5*x(6)  -0.5*x(5)  0  -0.5*x(3)  0.5*x(2)  0.5*x(1);
%  0  0  0  0  0  axx*(Iyy*x(7) - 2*Izy*x(6) - Izz*x(7))  axx*(Iyy*x(6) + 2*Iyz*x(7) - Izz*x(6));
%  0  0  0  0  ayy*(-Ixx*x(7) + Izy*x(6) + Izz*x(7)) + ayz*(Ixx*x(6) - Iyy*x(6) - Iyz*x(7))  ayy*Izy*x(5) + ayz*x(5)*(Ixx - Iyy)  ayy*x(5)*(Izz - Ixx) - ayz*Iyz*x(5);
%  0  0  0  0  azy*(-Ixx*x(7) + Izy*x(6) + Izz*x(7)) + azz*(Ixx*x(6) - Iyy*x(6) - Iyz*x(7))  azy*Izy*x(5) + azz*x(5)*(Ixx - Iyy)  azy*x(5)*(Izz - Ixx) - azz*Iyz*x(5)
%  ];

%New A Matrix (no double counting)
A = @(x) [
 0,  -0.5*x(5),  -0.5*x(6),  -0.5*x(7),  0,  0,  0;
 0.5*x(5),  0,  0.5*x(7),  -0.5*x(6),  0,  0,  0; 
 0.5*x(6),  -0.5*x(7),  0,  0.5*x(5),  0,  0,  0;
 0.5*x(7),  0.5*x(6),  -0.5*x(5),  0,  0,  0,  0;
 0,  0,  0,  0,  0,  axx*(Iyy*x(7) - Izy*x(6) - Izz*x(7)),  axx*Iyz*x(7);
 0,  0,  0,  0,  ayy*(-Ixx*x(7) + Izy*x(6) + Izz*x(7)) + ayz*(Ixx*x(6) - Iyy*x(6) - Iyz*x(7)),  0, 0;
 0,  0,  0,  0,  azy*(-Ixx*x(7) + Izy*x(6) + Izz*x(7)) + azz*(Ixx*x(6) - Iyy*x(6) - Iyz*x(7)),  0, 0
 ];



B = [0 0 0;
     0 0 0;
     0 0 0;
     0 0 0; 
     axx 0 0;
     0 ayy ayz;
     0 azy azz];

stateSpaceModel = @(t,x) A(x)*x + B*u(t);



%Function to renormalize the quaternion
%normalize = @(x) [x(1:4) / norm(x(1:4)); x(5:7)];


%stateSpaceModel = @(t,x) A(normalize(x))*normalize(x) + B*u(t);


% Set time span for simulation
tspan = [0 40];

% Solve the state-space model using ode45
[t, x] = ode23tb(stateSpaceModel, tspan, x0);

% Plot the results
figure(1);
plot(t, x);
xlabel('Time (s)');
ylabel('States');
legend('q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz');
title('State Evolution with State-Dependent Matrices');
grid on;


rows = size(x,1);
for i=1:rows
    Etotal(i) = (1/2)*(Ixx*(x(i,5)^2) + Iyy*(x(i,6)^2) + Iyz*x(i,6)*x(i,7) + Izy*x(i,6)*x(i,7) + Izz*(x(i,7)^2));

    Lx = Ixx*x(i,5);
    Ly = Iyy*x(i,6) + Iyz*x(i,7);
    Lz = Izy*x(i,6) + Izz*x(i,7);
    Ltotal(i) = sqrt(Lx^2 + Ly^2 + Lz^2); %taking absolute value

    norm_check(i) = sqrt(x(i,1)^2 + x(i,2)^2 + x(i,3)^2 + x(i,4)^2);
end



figure(2)
plot(t, Etotal);
xlabel('Time (s)');
ylabel('Total Energy (J)');
title('Total Energy of States');
grid on;

figure(3)
plot(t, Ltotal);
xlabel('Time (s)');
ylabel('Total Angular Momentum');
title('Total Angular Momentum of States');
grid on;

figure(4)
plot(t, norm_check);
xlabel('Time (s)');
ylabel('Quaternion Norm');
title('Norm of Quaternion');
grid on;





%% STATE SPACE MODEL FROM YAGUANG - Kyle

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

%% STATE SPACE MODEL FROM YAGUANG - Arif

A = @(x) 0.5*[
    0  0  0  0  0  0  0;
    0  0  0  0  1  0  0;
    0  0  0  0  0  1  0;
    0  0  0  0  0  0  1;
    0  0  0  0  0  0  0;
    0  0  0  0  0  0  0;
    0  0  0  0  0  0  0;
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


%% RK4 Model
close;
clear;
clc;

% Body axes are initially aligned with inertial axes 
q0_0 = 1;  
q1_0 = 0;    
q2_0 = 0;   
q3_0 = 0;   

% Spin initially about y-axis 
wx_0 = 1;   
wy_0 = 0;   
wz_0 = 0;   

%Control inputs
ux = 0;
uy = 0;
uz = 0;

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

%Equations
f0 = @(q1, q2, q3, wx, wy, wz) 0.5*(-q1*wx - q2*wy - q3*wz);

fx = @(q0, q2, q3, wx, wy, wz) 0.5*(q0*wx - q3*wy + q2*wz);
gx = @(wy, wz) axx*ux + axx*(wz*(Iyy*wy + Iyz*wz) - wy*(Izy*wy + Izz*wz));

fy = @(q0, q1, q3, wx, wy, wz) 0.5*(q3*wx + q0*wy - q1*wz);
gy = @(wx, wy, wz) ayy*(-wz*wx*Ixx + wx*(Izy*wy + Izz*wz)) + ayz*(wy*wx*Ixx - wx*(Iyy*wy + Iyz*wz)) + ayy*uy + ayz*uz;

fz = @(q0, q1, q2, wx, wy, wz) 0.5*(-q2*wx + q1*wy + q0*wz);
gz = @(wx, wy, wz) azy*(-wz*wx*Ixx + wx*(Izy*wy + Izz*wz)) + azz*(wy*wx*Ixx - wx*(Iyy*wy + Iyz*wz)) + azy*uy + azz*uz;


%Parameters
h = 0.01; %time step
tf = 90*60*5; %final time
t = [0:h:tf];
n = length(t);

%Initializing lists
q0 = zeros(1,n);
q1 = zeros(1,n);
q2 = zeros(1,n);
q3 = zeros(1,n);
wx = zeros(1,n);
wy = zeros(1,n);
wz = zeros(1,n);
norm = zeros(1,n);
Etotal = zeros(1,n);
Ltotal = zeros(1,n);

q0(1) = q0_0;
q1(1) = q1_0;
q2(1) = q2_0;
q3(1) = q3_0;
wx(1) = wx_0;
wy(1) = wy_0;
wz(1) = wz_0;
norm(1) = 1;

Etotal(1) = (1/2)*(Ixx*(wx_0^2) + Iyy*(wy_0^2) + Iyz*wy_0*wz_0 + Izy*wy_0*wz_0 + Izz*(wz_0^2));

Lx0 = Ixx*wx_0;
Ly0 = Iyy*wy_0 + Iyz*wz_0;
Lz0 = Izy*wy_0 + Izz*wz_0;
Ltotal(1) = sqrt(Lx0^2 + Ly0^2 + Lz0^2);

for i = 1:n-1
    k1q0 = f0(q1(i), q2(i), q3(i), wx(i), wy(i), wz(i));
    k1q1 = fx(q0(i), q2(i), q3(i), wx(i), wy(i), wz(i));
    k1wx = gx(wy(i), wz(i));
    k1q2 = fy(q0(i), q1(i), q3(i), wx(i), wy(i), wz(i));
    k1wy = gy(wx(i), wy(i), wz(i));
    k1q3 = fz(q0(i), q1(i), q2(i), wx(i), wy(i), wz(i));
    k1wz = gz(wx(i), wy(i), wz(i));

    k2q0 = f0(q1(i) + 1/2*k1q1*h, q2(i) + 1/2*k1q2*h, q3(i) + 1/2*k1q3*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2q1 = fx(q0(i) + 1/2*k1q0*h, q2(i) + 1/2*k1q2*h, q3(i) + 1/2*k1q3*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2wx = gx(wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2q2 = fy(q0(i) + 1/2*k1q0*h, q1(i) + 1/2*k1q1*h, q3(i) + 1/2*k1q3*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2wy = gy(wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2q3 = fz(q0(i) + 1/2*k1q0*h, q1(i) + 1/2*k1q1*h, q2(i) + 1/2*k1q2*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2wz = gz(wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);

    k3q0 = f0(q1(i) + 1/2*k2q1*h, q2(i) + 1/2*k2q2*h, q3(i) + 1/2*k2q3*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3q1 = fx(q0(i) + 1/2*k2q0*h, q2(i) + 1/2*k2q2*h, q3(i) + 1/2*k2q3*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3wx = gx(wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3q2 = fy(q0(i) + 1/2*k2q0*h, q1(i) + 1/2*k2q1*h, q3(i) + 1/2*k2q3*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3wy = gy(wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3q3 = fz(q0(i) + 1/2*k2q0*h, q1(i) + 1/2*k2q1*h, q2(i) + 1/2*k2q2*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3wz = gz(wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);

    k4q0 = f0(q1(i) + k3q1*h, q2(i) + k3q2*h, q3(i) + k3q3*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4q1 = fx(q0(i) + k3q0*h, q2(i) + k3q2*h, q3(i) + k3q3*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4wx = gx(wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4q2 = fy(q0(i) + k3q0*h, q1(i) + k3q1*h, q3(i) + k3q3*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4wy = gy(wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4q3 = fz(q0(i) + k3q0*h, q1(i) + k3q1*h, q2(i) + k3q2*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4wz = gz(wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);

    q0(i+1) = q0(i) + 1/6*(k1q0 + 2*k2q0 + 2*k3q0 + k4q0)*h;
    q1(i+1) = q1(i) + 1/6*(k1q1 + 2*k2q1 + 2*k3q1 + k4q1)*h;
    wx(i+1) = wx(i) + 1/6*(k1wx + 2*k2wx + 2*k3wx + k4wx)*h;
    q2(i+1) = q2(i) + 1/6*(k1q2 + 2*k2q2 + 2*k3q2 + k4q2)*h;
    wy(i+1) = wy(i) + 1/6*(k1wy + 2*k2wy + 2*k3wy + k4wy)*h;
    q3(i+1) = q3(i) + 1/6*(k1q3 + 2*k2q3 + 2*k3q3 + k4q3)*h;
    wz(i+1) = wz(i) + 1/6*(k1wz + 2*k2wz + 2*k3wz + k4wz)*h;

     norm_new = sqrt(q0(i+1)^2 + q1(i+1)^2 + q2(i+1)^2 + q3(i+1)^2);
     q0(i+1) = q0(i+1)/norm_new;
     q1(i+1) = q1(i+1)/norm_new;
     q2(i+1) = q2(i+1)/norm_new;
     q3(i+1) = q3(i+1)/norm_new;

     norm(i+1) = sqrt(q0(i+1)^2 + q1(i+1)^2 + q2(i+1)^2 + q3(i+1)^2);

    Etotal(i+1) = (1/2)*(Ixx*(wx(i+1)^2) + Iyy*(wy(i+1)^2) + Iyz*wy(i+1)*wz(i+1) + Izy*wy(i+1)*wz(i+1) + Izz*(wz(i+1)^2));
    
    Lx = Ixx*wx(i+1);
    Ly = Iyy*wy(i+1) + Iyz*wz(i+1);
    Lz = Izy*wy(i+1) + Izz*wz(i+1);
    Ltotal(i+1) = sqrt(Lx^2 + Ly^2 + Lz^2); %taking absolute value



end

%Plotting
figure(1)
q0plot = plot(t, q0);
hold on
q1plot = plot(t, q1);
hold on
q2plot = plot(t, q2);
hold on
q3plot = plot(t, q3);
hold on
wxplot = plot(t, wx);
hold on
wyplot = plot(t, wy);
hold on
wzplot = plot(t, wz);
grid on
legend('q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz');
xlabel('Time (s)');
ylabel('Magnitude of States');
title('Evolution of State Variables');
ylim([-1.2,1.2])

figure(2)
normplot = plot(t, norm);
grid on
xlabel('Time (s)');
ylabel('Quaternion Norm');
title('Unit Quaternion Verification');

figure(3)
Eplot = plot(t, Etotal);
grid on
xlabel('Time (s)');
ylabel('Energy (J)');
title('Total Energy of States');

figure(4)
Lplot = plot(t, Ltotal);
grid on
xlabel('Time (s)');
ylabel('Angular Momentum (kg*m^2/s)');
title('Total Angular Momentum of States');

%% Testing function
close;
clear;
clc;

% Use function solve_RK4(x1, x2, x3, x4, x5, x6, x7, u_x, u_y, u_z, h, t)
% Inputs of x1-x7 correspond to initial conditions of state variables
% x1, x2, x3, x4 = q0, q1, q2, q3
% x5, x6, x7 = wx, wy, wz
% u_x, u_y, u_z correspond to control inputs
% h is the time step used 
% t is the amount of time to be solved for in seconds


solve_RK4(1,0,0,0,0,1,0,0,0,0,0.001,40)


%% RK4 function

function RK4 = solve_RK4(x1, x2, x3, x4, x5, x6, x7, u_x, u_y, u_z, h, t)
 
q0_0 = x1;  
q1_0 = x2;    
q2_0 = x3;   
q3_0 = x4;   

% Spin initially about y-axis 
wx_0 = x5;   
wy_0 = x6;   
wz_0 = x7;   

%Control inputs
ux = u_x;
uy = u_y;
uz = u_z;

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

%Equations
f0 = @(q1, q2, q3, wx, wy, wz) 0.5*(-q1*wx - q2*wy - q3*wz);

fx = @(q0, q2, q3, wx, wy, wz) 0.5*(q0*wx - q3*wy + q2*wz);
gx = @(wy, wz) axx*ux + axx*(wz*(Iyy*wy + Iyz*wz) - wy*(Izy*wy + Izz*wz));

fy = @(q0, q1, q3, wx, wy, wz) 0.5*(q3*wx + q0*wy - q1*wz);
gy = @(wx, wy, wz) ayy*(-wz*wx*Ixx + wx*(Izy*wy + Izz*wz)) + ayz*(wy*wx*Ixx - wx*(Iyy*wy + Iyz*wz)) + ayy*uy + ayz*uz;

fz = @(q0, q1, q2, wx, wy, wz) 0.5*(-q2*wx + q1*wy + q0*wz);
gz = @(wx, wy, wz) azy*(-wz*wx*Ixx + wx*(Izy*wy + Izz*wz)) + azz*(wy*wx*Ixx - wx*(Iyy*wy + Iyz*wz)) + azy*uy + azz*uz;


%Parameters
t_list = [0:h:t];
n = length(t_list);

%Initializing lists
q0 = zeros(1,n);
q1 = zeros(1,n);
q2 = zeros(1,n);
q3 = zeros(1,n);
wx = zeros(1,n);
wy = zeros(1,n);
wz = zeros(1,n);
norm = zeros(1,n);
Etotal = zeros(1,n);
Ltotal = zeros(1,n);

q0(1) = q0_0;
q1(1) = q1_0;
q2(1) = q2_0;
q3(1) = q3_0;
wx(1) = wx_0;
wy(1) = wy_0;
wz(1) = wz_0;
norm(1) = 1;

Etotal(1) = (1/2)*(Ixx*(wx_0^2) + Iyy*(wy_0^2) + Iyz*wy_0*wz_0 + Izy*wy_0*wz_0 + Izz*(wz_0^2));

Lx0 = Ixx*wx_0;
Ly0 = Iyy*wy_0 + Iyz*wz_0;
Lz0 = Izy*wy_0 + Izz*wz_0;
Ltotal(1) = sqrt(Lx0^2 + Ly0^2 + Lz0^2);

for i = 1:n-1
    k1q0 = f0(q1(i), q2(i), q3(i), wx(i), wy(i), wz(i));
    k1q1 = fx(q0(i), q2(i), q3(i), wx(i), wy(i), wz(i));
    k1wx = gx(wy(i), wz(i));
    k1q2 = fy(q0(i), q1(i), q3(i), wx(i), wy(i), wz(i));
    k1wy = gy(wx(i), wy(i), wz(i));
    k1q3 = fz(q0(i), q1(i), q2(i), wx(i), wy(i), wz(i));
    k1wz = gz(wx(i), wy(i), wz(i));

    k2q0 = f0(q1(i) + 1/2*k1q1*h, q2(i) + 1/2*k1q2*h, q3(i) + 1/2*k1q3*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2q1 = fx(q0(i) + 1/2*k1q0*h, q2(i) + 1/2*k1q2*h, q3(i) + 1/2*k1q3*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2wx = gx(wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2q2 = fy(q0(i) + 1/2*k1q0*h, q1(i) + 1/2*k1q1*h, q3(i) + 1/2*k1q3*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2wy = gy(wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2q3 = fz(q0(i) + 1/2*k1q0*h, q1(i) + 1/2*k1q1*h, q2(i) + 1/2*k1q2*h, wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);
    k2wz = gz(wx(i) + 1/2*k1wx*h, wy(i) + 1/2*k1wy*h, wz(i) + 1/2*k1wz*h);

    k3q0 = f0(q1(i) + 1/2*k2q1*h, q2(i) + 1/2*k2q2*h, q3(i) + 1/2*k2q3*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3q1 = fx(q0(i) + 1/2*k2q0*h, q2(i) + 1/2*k2q2*h, q3(i) + 1/2*k2q3*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3wx = gx(wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3q2 = fy(q0(i) + 1/2*k2q0*h, q1(i) + 1/2*k2q1*h, q3(i) + 1/2*k2q3*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3wy = gy(wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3q3 = fz(q0(i) + 1/2*k2q0*h, q1(i) + 1/2*k2q1*h, q2(i) + 1/2*k2q2*h, wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);
    k3wz = gz(wx(i) + 1/2*k2wx*h, wy(i) + 1/2*k2wy*h, wz(i) + 1/2*k2wz*h);

    k4q0 = f0(q1(i) + k3q1*h, q2(i) + k3q2*h, q3(i) + k3q3*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4q1 = fx(q0(i) + k3q0*h, q2(i) + k3q2*h, q3(i) + k3q3*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4wx = gx(wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4q2 = fy(q0(i) + k3q0*h, q1(i) + k3q1*h, q3(i) + k3q3*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4wy = gy(wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4q3 = fz(q0(i) + k3q0*h, q1(i) + k3q1*h, q2(i) + k3q2*h, wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);
    k4wz = gz(wx(i) + k3wx*h, wy(i) + k3wy*h, wz(i) + k3wz*h);

    q0(i+1) = q0(i) + 1/6*(k1q0 + 2*k2q0 + 2*k3q0 + k4q0)*h;
    q1(i+1) = q1(i) + 1/6*(k1q1 + 2*k2q1 + 2*k3q1 + k4q1)*h;
    wx(i+1) = wx(i) + 1/6*(k1wx + 2*k2wx + 2*k3wx + k4wx)*h;
    q2(i+1) = q2(i) + 1/6*(k1q2 + 2*k2q2 + 2*k3q2 + k4q2)*h;
    wy(i+1) = wy(i) + 1/6*(k1wy + 2*k2wy + 2*k3wy + k4wy)*h;
    q3(i+1) = q3(i) + 1/6*(k1q3 + 2*k2q3 + 2*k3q3 + k4q3)*h;
    wz(i+1) = wz(i) + 1/6*(k1wz + 2*k2wz + 2*k3wz + k4wz)*h;

     norm_new = sqrt(q0(i+1)^2 + q1(i+1)^2 + q2(i+1)^2 + q3(i+1)^2);
     q0(i+1) = q0(i+1)/norm_new;
     q1(i+1) = q1(i+1)/norm_new;
     q2(i+1) = q2(i+1)/norm_new;
     q3(i+1) = q3(i+1)/norm_new;

     norm(i+1) = sqrt(q0(i+1)^2 + q1(i+1)^2 + q2(i+1)^2 + q3(i+1)^2);

    Etotal(i+1) = (1/2)*(Ixx*(wx(i+1)^2) + Iyy*(wy(i+1)^2) + Iyz*wy(i+1)*wz(i+1) + Izy*wy(i+1)*wz(i+1) + Izz*(wz(i+1)^2));
    
    Lx = Ixx*wx(i+1);
    Ly = Iyy*wy(i+1) + Iyz*wz(i+1);
    Lz = Izy*wy(i+1) + Izz*wz(i+1);
    Ltotal(i+1) = sqrt(Lx^2 + Ly^2 + Lz^2); %taking absolute value

end

%Plotting
figure(1)
q0plot = plot(t_list, q0);
hold on
q1plot = plot(t_list, q1);
hold on
q2plot = plot(t_list, q2);
hold on
q3plot = plot(t_list, q3);
hold on
wxplot = plot(t_list, wx);
hold on
wyplot = plot(t_list, wy);
hold on
wzplot = plot(t_list , wz);
grid on
legend('q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz');
xlabel('Time (s)');
ylabel('Magnitude of States');
title('Evolution of State Variables');

figure(2)
normplot = plot(t_list, norm);
grid on
xlabel('Time (s)');
ylabel('Quaternion Norm');
title('Unit Quaternion Verification');

figure(3)
Eplot = plot(t_list, Etotal);
grid on
xlabel('Time (s)');
ylabel('Energy (J)');
title('Total Energy of States');

figure(4)
Lplot = plot(t_list, Ltotal);
grid on
xlabel('Time (s)');
ylabel('Angular Momentum (kg*m^2/s)');
title('Total Angular Momentum of States');

end
