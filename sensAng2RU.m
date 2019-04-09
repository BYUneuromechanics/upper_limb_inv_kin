function [REUt, RFUt, RGUt, RHUt, t] = sensAng2RU(mat)

% This function takes the output from trakSTAR and returns rotation
% matricies for sensor coordinate systems relative to the Universal frame
% of the transmitter.
%
% INPUTS
% mat   Nx43 matrix, where N is the number of samples
%       Columns  1- 7 contain sensor #, xyz, and aer for sensor E
%       Columns  8-14 contain sensor #, xyz, and aer for sensor F
%       Columns 15-21 contain sensor #, xyz, and aer for sensor G
%       Columns 22-28 contain sensor #, xyz, and aer for sensor H
%       Columns 29-35 contain sensor #, xyz, and aer for sensor on the scapula
%       Columns 36-42 contain sensor #, xyz, and aer for sensor on the stylus
%       Column     43 contains time
%
% LABELS
% A is the BCS of the thorax
% B is the BCS of the humerus (upper arm)
% C is the BCS of the distal forearm (forearm)
% D is the BCS of the third metacarpal (hand)
% E is the SCS of the sensor on the thorax
% F is the SCS of the sensor on the humerus
% G is the SCS of the sensor on the forearm
% H is the SCS of the sensor on the hand
% U is the stationary frame of the transmitter (universal frame)
%
%OUTPUTS
% REUt  3x3xn matrix in which each 3x3 matrix is a rotation matrix
%       describing the orientation of E relative to U over time.
% RFUt  Similar to REU
% RGUt  Similar to REU
% RHUt  Similar to REU
% t     an nx1 matrix of corrected time steps

% extract SCS angles from the data and the time
SCSE = mat(:,5:7);
SCSF = mat(:,12:14);
SCSG = mat(:,19:21);
SCSH = mat(:,26:28);
t = mat(:,end);
t = t(:)-t(1);

% convert a,e,r sensor angles to rotation matricies for each SCS
REUt = aer2R(SCSE(:,1),SCSE(:,2),SCSE(:,3));
RFUt = aer2R(SCSF(:,1),SCSF(:,2),SCSF(:,3));
RGUt = aer2R(SCSG(:,1),SCSG(:,2),SCSG(:,3));
RHUt = aer2R(SCSH(:,1),SCSH(:,2),SCSH(:,3));
end