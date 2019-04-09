function mat = trakSTARunits2SIunits(mat)

% This function converts the output from trakSTAR, which has units of
% inches (xyz) or deg (aer) to meters (xyz) and radians (aer)
%
% INPUT
% mat   Nx43 matrix, where N is the number of samples
%       Columns  1- 7 contain sensor #, xyz, and aer for sensor E
%       Columns  8-14 contain sensor #, xyz, and aer for sensor F
%       Columns 15-21 contain sensor #, xyz, and aer for sensor G
%       Columns 22-28 contain sensor #, xyz, and aer for sensor H
%       Columns 29-35 contain sensor #, xyz, and aer for sensor on the scapula
%       Columns 36-42 contain sensor #, xyz, and aer for sensor on the stylus
%       Column     43 contains time
%
% OUTPUT
% mat   Same as input, but with xyz data in meters and aer data in radians

% Designate columns containing xyz data and convert from inches to meters
xyz_col = [2:4, 9:11, 16:18, 23:25, 30:32, 37:39];
mat(:,xyz_col) = mat(:,xyz_col)*25.4/1000;

% The sensor on the scapula and the sensor on the stylus were both set to
% a scale of 72, while the other sensors appear to be set to a scale of 36.
% mat(:,[30:32, 37:39]) = mat(:,[30:32, 37:39])/2;
% mat(:,[2:4]) = mat(:,[2:4])/2;

% Designate columns containing aer data and convert from deg to rad
aer_col = xyz_col + 3;
mat(:,aer_col) = mat(:,aer_col)*pi/180;