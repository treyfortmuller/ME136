
function struct_out = csv2strucT(filename, numberOfdataumn) 

% Reads from csv file and stores data in a struct as an output.
% put the csv file at the same directory as this function. 

if (~ischar(filename))
    error('the file name should be a string');
end

if (nargin < 2 || nargin > 2)
    error('you should specify file name and number of dataumns in your csv file');
end

fileID  = fopen(filename, 'rt');
format = [];
for i=1:numberOfdataumn
   format = [format '%s'];
end

M_cell = textscan(fileID, format, 'delimiter', ',');
cel_size = size(M_cell);

data = nan(numberOfdataumn, length(M_cell{1}));

for i=1:cel_size(2)
    len = length(M_cell{i});
    for j=2:len
        data(i,j) = str2double(M_cell{i}{j});   
    end
end
% data structure
struct_out.time            = data(1,:);
struct_out.accelerometer.x = data(2,:);
struct_out.accelerometer.y = data(3,:);
struct_out.accelerometer.z = data(4,:);
struct_out.rateGyro.x      = data(5,:);
struct_out.rateGyro.y      = data(6,:);
struct_out.rateGyro.z      = data(7,:);
struct_out.heightSensor    = data(8,:);
struct_out.opticalflow.x   = data(9,:);
struct_out.opticalflow.y   = data(10,:);
struct_out.battVoltage     = data(11,:);
struct_out.motorcmd1       = data(12,:);
struct_out.motorcmd2       = data(13,:);
struct_out.motorcmd3       = data(14,:);
struct_out.motorcmd4       = data(15,:);
struct_out.debugval0       = data(16,:);
struct_out.debugval1       = data(17,:);
struct_out.debugval2       = data(18,:);
struct_out.debugval3       = data(19,:);
struct_out.debugval4       = data(20,:);
struct_out.debugval5       = data(21,:);
struct_out.debugval6       = data(22,:);
struct_out.debugval7       = data(23,:);
struct_out.debugval8       = data(24,:);
struct_out.debugval9       = data(25,:);
struct_out.debugval10      = data(26,:);
struct_out.debugval11      = data(27,:);
struct_out.motorsOn        = data(28,:);
struct_out.debugChar       = data(29,:);

end
