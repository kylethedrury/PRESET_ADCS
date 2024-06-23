% Take a CSV full of normal time dates and turn into Julian dates with the
% JulianDate function

% Specify the input and output CSV file names
inputCSV = 'Simulink Parameters/dates.csv';
outputCSV = 'Simulink Parameters/julian_dates.csv';

% Read the input CSV file into a table
dateTable = readtable(inputCSV);

% Initialize an array to store the Julian dates
numRows = height(dateTable);
julianDates = zeros(numRows, 1);

% Loop through each row in the table to convert the date to Julian date

for i = 1:numRows
    % Extract the date components from the table
    year = dateTable.Year(i);
    month = dateTable.Month(i);
    day = dateTable.Day(i);
    hour = dateTable.Hour(i);
    minute = dateTable.Minute(i);
    second = dateTable.Second(i);
    
    % Create a datetime object
    dateTime = datetime(year, month, day, hour, minute, second);
    
    % Convert the datetime to Julian date
    julianDates(i) = juliandate(dateTime);
    disp(i);
end

% Write the Julian dates to a new CSV file
writetable(table(julianDates), outputCSV, 'WriteVariableNames', false);

% Display a message indicating completion
disp(['Julian dates have been written to ', outputCSV]);
