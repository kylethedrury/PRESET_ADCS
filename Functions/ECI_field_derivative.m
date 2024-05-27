% THIS IS A FUNCTION FOR GETTING THE MAGNETIC FIELD VECTOR DERIVATIVE IN ECI COORDS AS A FUNCTION OF T

% This function references a csv file located in Simulink Parameters/3am_LTDN for a 3am LTDN time, etc

function output = ECI_field_derivative(LTDN, t)

    if LTDN == 3
        filepath = fullfile('..', 'Simulink Parameters/3am_LTDN/', 'PRESET_ECI_Field_Derivative_3am_LTDN.csv');

    elseif LTDN == 6
        filepath = fullfile('..', 'Simulink Parameters/6am_LTDN/', 'PRESET_ECI_Field_Derivative_6am_LTDN.csv');

    else
        filepath = fullfile('..', 'Simulink Parameters/12pm_LTDN/', 'PRESET_ECI_Field_Derivative_12pm_LTDN.csv');

    end

    data = readtable(filepath);

    output = interp1(data.Time, data.Field_Derivative, t);

end