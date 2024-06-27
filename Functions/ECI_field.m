% THIS IS A FUNCTION FOR GETTING THE MAGNETIC FIELD VECTOR IN ECI COORDS AS A FUNCTION OF TIME

function output = ECI_field(time, field, t) % field is the field array; time is the time array; t is the time to interpolate at

    % Interpolate each component of the vectors
    vec_x = interp1(time, field(:, 1), t, 'linear');
    vec_y = interp1(time, field(:, 2), t, 'linear');
    vec_z = interp1(time, field(:, 3), t, 'linear');

    % Combine the interpolated components into a 3x1 vector
    output = [vec_x; vec_y; vec_z];

end