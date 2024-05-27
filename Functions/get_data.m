% THIS IS A FUNCTION FOR GETTING POSITION AND VELOCITY VECTORS OVER AN ORBIT IN ECI, ECEF, and LLA COORDINATES

% Import STK orbital trajectory data as tables

function [ECI_position, ECI_velocity, ECEF_position, ECEF_velocity, LLA_position, LLA_velocity] = get_data(LTDN) % Function depends only on LTDN
 
    if LTDN == 3
        ECEF = readtable('Simulink Parameters/3am_LTDN/PRESET_ECEF_Position_Velocity_3am_LTDN.csv');
        ECI = readtable('Simulink Parameters/3am_LTDN/PRESET_ECI_Position_Velocity_3am_LTDN.csv');
        LLA = readtable('Simulink Parameters/3am_LTDN/PRESET_LLA_Position_3am_LTDN.csv');

    elseif LTDN == 6
        ECEF = readtable('Simulink Parameters/6am_LTDN/PRESET_ECEF_Position_Velocity_6am_LTDN.csv');
        ECI = readtable('Simulink Parameters/6am_LTDN/PRESET_ECI_Position_Velocity_6am_LTDN.csv');
        LLA = readtable('Simulink Parameters/6am_LTDN/PRESET_LLA_Position_6am_LTDN.csv');

    else 
        ECEF = readtable('Simulink Parameters/12pm_LTDN/PRESET_ECEF_Position_Velocity_12pm_LTDN.csv');
        ECI = readtable('Simulink Parameters/12pm_LTDN/PRESET_ECI_Position_Velocity_12pm_LTDN.csv');
        LLA = readtable('Simulink Parameters/12pm_LTDN/PRESET_LLA_Position_12pm_LTDN.csv');
    end

    %% Save each column in the table as a list for convenience 

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

    %% Turn the above data into vectors; that is, position and velocity vectors in the ECI and ECEF (will be used to get the ECI-O DCM matrix)

    ECI_position = zeros(3, 1048575);  % Allocate arrays to save time
    ECI_velocity = zeros(3, 1048575);
    ECEF_position = zeros(3, 1048575);
    ECEF_velocity = zeros(3, 1048575);
    LLA_position = zeros(3, 1048575);
    LLA_velocity = zeros(3, 1048575);

    for i = 1:length(ECI_x)
    
        % Create vectors 
        ECI_position_vector = [ECI_x(i); ECI_y(i); ECI_z(i)];         
        ECI_velocity_vector = [ECI_vx(i); ECI_vy(i); ECI_vz(i)];
        ECEF_position_vector = [ECEF_x(i); ECEF_y(i); ECEF_z(i)];
        ECEF_velocity_vector = [ECEF_vx(i); ECEF_vy(i); ECEF_vz(i)];
        LLA_position_vector = [lat(i), lon(i), alt(i)];
        LLA_rate_vector = [lat_rate(i), lon_rate(i), alt_rate(i)];

        % Append to arrays
        ECI_position(:, i) = ECI_position_vector;                        
        ECI_velocity(:, i) = ECI_velocity_vector;
        ECEF_position(:, i) = ECEF_position_vector;
        ECEF_velocity(:, i) = ECEF_velocity_vector;
        LLA_position(:, i) = LLA_position_vector;
        LLA_velocity(:, i) = LLA_rate_vector;

    end

end
