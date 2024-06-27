[time, position, velocity, field] = get_data(3);  

dcm = quat2dcm(q_initial');

disp(dcm)