function [RAE, RBF, RCG, RDH] = CalibrationP(mat)

% This function takes in the raw trakSTAR data taken from Calibration using
% the POSTURE METHOD. It returns the rotation Matrices of the BCS relative to the SCS, packaged
% for use in the Main.

% INPUTS 
% mat   Nx43 matrix, where N is the number of samples
%       Columns  1- 7 contain sensor #, xyz, and aer for sensor E
%       Columns  8-14 contain sensor #, xyz, and aer for sensor F
%       Columns 15-21 contain sensor #, xyz, and aer for sensor G
%       Columns 22-28 contain sensor #, xyz, and aer for sensor H
%       Columns 29-35 contain sensor #, xyz, and aer for sensor on the scapula
%       Columns 36-42 contain sensor #, xyz, and aer for sensor on the stylus
%       Column     43 contains time

% OUTPUTS
% RAE: 3x3 rotation matrix of BCS (A) relative to SCS (E)
% RBF: Similar to RAE
% RCG: Similar to RAE
% RDH: Similar to RAE

% Extract aer data using pre_P2R
[E0, F0, G0, H0] = pre_P2R(mat);

% Extract each BCS relative to each correspinding SCS
[RBS] = P2R(E0,F0,G0,H0);

% Package for use in Main
RAE = RBS(:,:,1);
RBF = RBS(:,:,2);
RCG = RBS(:,:,3);
RDH = RBS(:,:,4);

