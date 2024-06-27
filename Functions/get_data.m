% THIS IS A FUNCTION FOR GETTING STK PARAMETERS FROM A CSV

function [time, position, velocity, field] = get_data(LTDN)  % '3', '6' or '12' LTDN; '1' for thompson, '2' for yaw config

    function_dir = fileparts(mfilename('fullpath'));   % get the directory of the current function

    if LTDN == 3   % find the directory containing the data based on the LTDN
        csv_file_path = fullfile(function_dir, '..', 'Parameters/3am_LTDN/001.csv');

    elseif LTDN == 6
        csv_file_path = fullfile(function_dir, '..', 'Parameters/6am_LTDN1/001.csv');

    elseif LTDN == 12
        csv_file_path = fullfile(function_dir, '..', 'Parameters/12pm_LTDN/001.csv');
    
    else 
        disp('Invalid LTDN!')

    end

    data = readtable(csv_file_path);    % read in data

    x = (data.x)';        % save the position in ECI
    y = (data.y)';
    z = (data.z)';

    vx = (data.vx)';      % save the velocity in ECI
    vy = (data.vy)';
    vz = (data.vz)';

    Bx = (data.Bx)';      % save the field in ECI (convert from nT to T)
    By = (data.By)';
    Bz = (data.Bz)';

    time = (data.Time)';  % save the timesteps 

    position = [x; y; z];      % save all the data as 3xN matrices 
    velocity = [vx; vy; vz];
    field = [Bx; By; Bz] / 1e9;

end

