function [E0, F0, G0, H0] = pre_P2R(mat)

% This function takes the output from trakSTAR and extracts the aer data
% for each of the sensors (using posture calibration method). 

% INPUTS
% mat   Nx43 matrix, where N is the number of samples
%       Columns  1- 7 contain sensor #, xyz, and aer for sensor E
%       Columns  8-14 contain sensor #, xyz, and aer for sensor F
%       Columns 15-21 contain sensor #, xyz, and aer for sensor G
%       Columns 22-28 contain sensor #, xyz, and aer for sensor H
%       Columns 29-35 contain sensor #, xyz, and aer for sensor on the scapula
%       Columns 36-42 contain sensor #, xyz, and aer for sensor on the stylus
%       Column     43 contains time

% OUTPUTS:
% E0 is a 1x3 matrix containing [a,e,r] of sensor E at calibration
% F0 is a 1x3 matrix containing [a,e,r] of sensor F at calibration
% G0 is a 1x3 matrix containing [a,e,r] of sensor G at calibration
% H0 is a 1x3 matrix containing [a,e,r] of sensor H at calibration


% Find rows containing NaN (denoting a MARK or START/STOP in trakSTAR data)
% NOTE: 
%   One single row of NaN implies a MARK in the data
%   Two subsequent rows of NaN imply a START/STOP in the data
v = find(isnan(mat(:,1)));

v(2:end+1) = v;
v(1) = 0;
v(end+1) = 0;

% Find MARK row and extract aer data. 
% Also check for bad data (multiple MARK points)
check = [];
for i = 2:length(v)-1
    if  v(i) ~= v(i+1)-1 && v(i) ~= v(i-1)+1
          E0 = mat(v(i)-1,5:7);
          F0 = mat(v(i)-1,12:14);
          G0 = mat(v(i)-1,19:21);
          H0 = mat(v(i)-1,26:28);
        
        check(end+1) = v(i);
    else
    end
end

% Throw error to user if there are multiple MARK rows.
if size(check,2) > 1
msg = 'Multiple MARK Rows';
error(msg);
else if size(check,2) < 1
        msg = 'No MARK Row';
        error(msg);
    end
end


