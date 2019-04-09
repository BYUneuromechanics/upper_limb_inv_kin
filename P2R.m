function [RBS] = P2R(E0,F0,G0,H0)

% This function takes in aer data of the sensors (with respect to the 
% transmitter) extracted by pre_P2R. It returns the rotation Matrices of 
% the BCS relative to the SCS.


% INPUTS
% E0 is a 1x3 matrix containing [a,e,r] of sensor E at calibration
% F0 is a 1x3 matrix containing [a,e,r] of sensor F at calibration
% G0 is a 1x3 matrix containing [a,e,r] of sensor G at calibration
% H0 is a 1x3 matrix containing [a,e,r] of sensor H at calibration

% OUTPUTS
% RBS   3x3x4 matrix in which each 3x3 matrix is a rotation matrix of a
%       BCS (B) relative to a SCS (S):
%       1. RAE   3x3 rotation matrix of A relative to E
%       2. RBF   3x3 rotation matrix of B relative to F
%       3. RCG   3x3 rotation matrix of C relative to G
%       4. RDH   3x3 rotation matrix of D relative to H



% CREATE ROTATION MATRICES FOR SCS AT CALIBRATION AND TIME t
% RAB0 and RABt are the rotation matrices of A relative to B (normally
% written with A as leading subscript and B as leading superscript) at
% calibration and at time t, respectively.
REU0 = aer2R(E0(1), E0(2), E0(3));
RFU0 = aer2R(F0(1), F0(2), F0(3));
RGU0 = aer2R(G0(1), G0(2), G0(3));
RHU0 = aer2R(H0(1), H0(2), H0(3));



% CREATE ROTATION MATRICES FOR BCS AT CALIBRATION
RAU0 = [-1 0 0; 0 0 -1; 0 -1 0];
RBU0 = RAU0;
RCU0 = [0 1 0; 1 0 0; 0 0 -1];
RDU0 = RCU0;

% Create rotation matrix of a BCS relative to corresponding SCS
RAE =  REU0' * RAU0;
RBF =  RFU0' * RBU0;
RCG =  RGU0' * RCU0;
RDH =  RHU0' * RDU0;


RBS(:,:,1) = RAE;
RBS(:,:,2) = RBF;
RBS(:,:,3) = RCG;
RBS(:,:,4) = RDH;