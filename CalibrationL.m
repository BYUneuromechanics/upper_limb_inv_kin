function [RAE, RBF, RCG, RDH] = CalibrationL(sGmat, sLmat, s_opt)

% This function takes in the raw trakSTAR data taken from Calibration using
% the POSTURE METHOD. It returns the rotation Matrices of the BCS relative to the SCS, packaged
% for use in the Main.

% INPUTS 
% sLmat   Nx43 matrix containing calibration data gathered using the LANDMARK
%         CALIBRATION method. N is the number of samples
%       Columns  1- 7 contain sensor #, xyz, and aer for sensor E
%       Columns  8-14 contain sensor #, xyz, and aer for sensor F
%       Columns 15-21 contain sensor #, xyz, and aer for sensor G
%       Columns 22-28 contain sensor #, xyz, and aer for sensor H
%       Columns 29-35 contain sensor #, xyz, and aer for sensor on the scapula
%       Columns 36-42 contain sensor #, xyz, and aer for sensor on the stylus
%       Column     43 contains time

% sGmat:   Nx43 matrix containing data describing movements of the 
%          glenohumeral joint. N is the number of samples
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

% Find the GH rotation center
[FpGHF, deff] = pre_ICR(sGmat);

% Extract aer data using pre_L2R
[UpL, UpS, REU, RFU, RGU, RHU] = pre_L2R(sLmat, s_opt);

% Extract each BCS relative to each correspinding SCS
[RBS, SpL] = L2R(UpL, UpS, REU, RFU, RGU, RHU, FpGHF);

% Package for use in main
RAE = RBS(:,:,1);
RBF = RBS(:,:,2);
RCG = RBS(:,:,3);
RDH = RBS(:,:,4);

