% A SCRIPT FOR SWITCHING BETWEEN ECI, ECEF, and O FRAMES

syms r long lat; % akin to radius, radial angle and azimuthal angle in spherical coords (in ECEF frame)
syms vx vy vz; % the velocity vector components in the ECI frame (or the ECEF frame, we can convert to ECI after)
v = [vx, vy, vz]; % velocity 3-vector in ECEF
syms Bx By Bz; % B field vector components in ECEF 
B_ECEF = [Bx, By, Bz]; % B field in ECEF frame from IGRF model

r_lat_long_vector = [r long lat]; % initalize ECEF position vector 

x_ECEF = r*sin(lat)*cos(long); % convert to Euclidean coords in ECEF
y_ECEF = r*sin(lat)*sin(long);
z_ECEF = r*cos(long);

position_ECEF = [x_ECEF, y_ECEF, z_ECEF]; % GET POSITION IN ECEF in EUCLIDEAN COORDS

%% specify the year, month, day, hour, minute, and second to find the GMST angle 

syms Y M D h m s; % year (1901-2099 -> 0-197), month (0-11), day (0-364), hour (0-23), second (0-59) 

T0 = ( JD(Y, M, D, 0, 0, 0) - 2451545 )/36525; % some parameter we need from Markley & Crassidis textbook

greenwich_angle = 24110.54841 + 8640181.812866*T0 + 0.093104*T0^2 - (6.2e-6)*T0^3 + 1.002737909350795*(3600*h + 60*m + s); % get greenwich angle

ECI_to_ECEF = [cos(greenwich_angle), sin(greenwich_angle), 0; -sin(greenwich_angle), cos(greenwich_angle), 0; 0, 0, 1]; % ECI to ECEF matrix
ECEF_to_ECI = inv(ECI_to_ECEF); % ECEF to ECI matrix

position_ECI = ECI_to_ECEF \ position_ECEF; % GET POSITION IN ECI
B_ECI = ECI_to_ECEF \ B_ECEF; % GET B FIELD IN ECI

%% now we define the orbit frame axes

z_O = -1*position_ECI / norm(position_ECI); % orbital basis
y_O = -1*cross(position_ECI, v) / norm( cross(position_ECI, v) );
x_O = cross(y_O, z_O);

O_to_ECI = [x_O', y_O', z_0']; % O to ECI matrix
ECI_to_O = inv(O_to_ECI); % ECI to O matrix
ECEF_to_O = ECEF_to_ECI * ECI_to_O; % ECEF to O matrix
O_to_ECEF = inv(ECEF_to_O); % O to ECEF matrix

position_O = O_to_ECI \ position_ECI; % GET POSITION IN O
B_O = O_to_ECI \ B_ECEF; % GET B FIELD IN O

function result = JD(Y, M, D, h, m, s) % Julian Date function 

    if D == 180 || D == 364
        result = 1721013.5 + 367*Y - fix(7/4 * (Y + fix((m+9)/12))) + fix(275*M/12) + D + (60*h+m+s/61)/1440;

    else
        result = 1721013.5 + 367*Y - fix(7/4 * (Y + fix((m+9)/12))) + fix(275*M/12) + D + (60*h+m+s/60)/1440;

    end

end 