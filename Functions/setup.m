% This is a function for setting up initial variables

function [J, q_initial, omega_initial, ECI_pos, ECI_vel, ECEF_pos, ECEF_vel, LLA_pos, LLA_vel, B_ECF, B_ECI, t] = setup(time, LTDN, rotation)

    t = time;  % time to run the simulation
    J = [0.13614614, 0, 0; 0, 0.13608852, 0.00303665; 0, 0.00303665, 0.00636670]; % inertia tensor

    random_vector = randn(3, 1);
    omega_initial = (random_vector / norm(random_vector)) * rotation; % random omega vector of norm "rotation"

    random_vector = randn(4,1); 
    q_initial = random_vector / norm(random_vector); % random quaternion of norm 1

    % get data from STK files 
    if LTDN == 3
        ECEF = readtable('Simulink Parameters/3am_LTDN/PRESET_ECEF_Position_Velocity_3am_LTDN.csv');
        ECI = readtable('Simulink Parameters/3am_LTDN/PRESET_ECI_Position_Velocity_3am_LTDN.csv');
        LLA = readtable('Simulink Parameters/3am_LTDN/PRESET_LLA_Position_3am_LTDN.csv');
        IGRF = readtable('Simulink Parameters/3am_LTDN/PRESET_IGRF_3am_LTDN.csv');

    elseif LTDN == 6
        ECEF = readtable('Simulink Parameters/6am_LTDN/PRESET_ECEF_Position_Velocity_6am_LTDN.csv');
        ECI = readtable('Simulink Parameters/6am_LTDN/PRESET_ECI_Position_Velocity_6am_LTDN.csv');
        LLA = readtable('Simulink Parameters/6am_LTDN/PRESET_LLA_Position_6am_LTDN.csv');
        IGRF = readtable('Simulink Parameters/3am_LTDN/PRESET_IGRF_6am_LTDN.csv');

    else 
        ECEF = readtable('Simulink Parameters/12pm_LTDN/PRESET_ECEF_Position_Velocity_12pm_LTDN.csv');
        ECI = readtable('Simulink Parameters/12pm_LTDN/PRESET_ECI_Position_Velocity_12pm_LTDN.csv');
        LLA = readtable('Simulink Parameters/12pm_LTDN/PRESET_LLA_Position_12pm_LTDN.csv');
        IGRF = readtable('Simulink Parameters/3am_LTDN/PRESET_IGRF_12pm_LTDN.csv');

    end

    % Save each column in the table as a list for convenience 
    ECEF_x = ECEF.x;
    ECEF_y = ECEF.y;
    ECEF_z = ECEF.z;
    ECEF_vx = ECEF.vx;
    ECEF_vy = ECEF.vy;
    ECEF_vz = ECEF.vz;

    ECI_x = ECI.x;
    ECI_y = ECI.y;
    ECI_z = ECI.z;
    ECI_vx = ECI.vx;
    ECI_vy = ECI.vy;
    ECI_vz = ECI.vz;

    lat = LLA.Lat;
    lon = LLA.Lon;
    alt = LLA.Alt;
    lat_rate = LLA.Lat_Rate;
    lon_rate = LLA.Lon_Rate;
    alt_rate = LLA.Alt_Rate;

    B_ECF_x = IGRF.B_ECF_x;
    B_ECF_y = IGRF.B_ECF_y;
    B_ECF_z = IGRF.B_ECF_z;

    B_ECI_x = IGRF.B_ECI_x;
    B_ECI_y = IGRF.B_ECI_y;
    B_ECI_z = IGRF.B_ECI_z;

    % Turn the above data into vectors; that is, position and velocity vectors in the ECI and ECEF (will be used to get the ECI-O DCM matrix)

    ECI_pos = zeros(3, 1048575);  % Allocate arrays to save time
    ECI_vel = zeros(3, 1048575);
    ECEF_pos = zeros(3, 1048575);
    ECEF_vel = zeros(3, 1048575);
    LLA_pos = zeros(3, 1048575);
    LLA_vel = zeros(3, 1048575);
    B_ECF = zeros(3, 1048575);
    B_ECI = zeros(3, 1048575);


    for i = 1:length(ECI_x)
    
        % Create vectors 
        ECI_position_vector = [ECI_x(i); ECI_y(i); ECI_z(i)];         
        ECI_velocity_vector = [ECI_vx(i); ECI_vy(i); ECI_vz(i)];
        ECEF_position_vector = [ECEF_x(i); ECEF_y(i); ECEF_z(i)];
        ECEF_velocity_vector = [ECEF_vx(i); ECEF_vy(i); ECEF_vz(i)];
        LLA_position_vector = [lat(i); lon(i); alt(i)];
        LLA_rate_vector = [lat_rate(i); lon_rate(i); alt_rate(i)];
        B_ECF_vector = [B_ECF_x(i); B_ECF_y(i); B_ECF_z(i)];
        B_ECI_vector = [B_ECI_x(i); B_ECI_y(i); B_ECI_z(i)];

        % Append to arrays
        ECI_pos(:, i) = ECI_position_vector;                        
        ECI_vel(:, i) = ECI_velocity_vector;
        ECEF_pos(:, i) = ECEF_position_vector;
        ECEF_vel(:, i) = ECEF_velocity_vector;
        LLA_pos(:, i) = LLA_position_vector;
        LLA_vel(:, i) = LLA_rate_vector;
        B_ECF(:, i) = B_ECF_vector;
        B_ECI(:, i) = B_ECI_vector;

    end

end
