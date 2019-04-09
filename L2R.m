 function [RBS, SpL] = L2R(UpL, UpS, REU, RFU, RGU, RHU, FpGH)

% This function takes in landmark positions (expressed in the transmitter
% frame U) and rotation matrices of the SCS relative to the transmitter
% (measured at the same moment in time as the associated landmarks) and
% returns the rotation matrices of the BCS relative to the SCS.
%
% INPUTS
% UpLU  3x12 matrix in which each column describes the position of a
%       landmark (see below) relative to the transmitter
% UpSU  3x14 matrix in which each column describes the position of a sensor
%       relative to the transmitter at the moment when the position of a
%       landmark was measured (see below)
% REU   3x3x4 matrix in which each 3x3 matrix is a rotation matrix
%       describing the orientation of E relative to U at the moment when
%       the position of a landmark was measured (see below)
% RFU   Similar to REU
% RGU   Similar to REU
% RHU   Similar to REU
% FpGH  3x1 vector describing the position of the center of rotation of the
%       glenohumeral joint relative to sensor F
%
% OUTPUTS
% RBS   3x3x4 matrix in which each 3x3 matrix is a rotation matrix of a
%       BCS (B) relative to a SCS (S):
%       1. RAE   3x3 rotation matrix of A relative to E
%       2. RBF   3x3 rotation matrix of B relative to F
%       3. RCG   3x3 rotation matrix of C relative to G
%       4. RDH   3x3 rotation matrix of D relative to H
% SpLS  3x15 matrix in which each column describes the position of a
%       landmark relative to a sensor (see below)

% EXTRACT LANDMARK POSITIONS FROM UpLU
UpC7    = UpL(:, 1);
UpT8    = UpL(:, 2);
UpIJ    = UpL(:, 3);
UpPX    = UpL(:, 4);
UpEL    = UpL(:, 5);
UpEM    = UpL(:, 6);
UpRS    = UpL(:, 7);
UpUS    = UpL(:, 8);
UpMC2h  = UpL(:, 9);
UpMC3h  = UpL(:,10);
UpMC4h  = UpL(:,11);
UpMC3b  = UpL(:,12);

% EXTRACT SENSOR POSITIONS FROM UpSU
UpE_C7     = UpS(:, 1);
UpE_T8     = UpS(:, 2);
UpE_IJ     = UpS(:, 3);
UpE_PX     = UpS(:, 4);
UpF_EL     = UpS(:, 5);
UpF_EM     = UpS(:, 6);
UpG_EL     = UpS(:, 7);
UpG_EM     = UpS(:, 8);
UpG_RS     = UpS(:, 9);
UpG_US     = UpS(:,10);
UpH_MC2h   = UpS(:,11);
UpH_MC3h   = UpS(:,12);
UpH_MC4h   = UpS(:,13);
UpH_MC3b   = UpS(:,14);

% EXTRACT ROTATION MATRICES FROM INPUT MATRICES
REU_C7   = REU(:,:,1);   % REU at the moment C7 was measured
REU_T8   = REU(:,:,2);
REU_IJ   = REU(:,:,3);
REU_PX   = REU(:,:,4);
RFU_EL   = RFU(:,:,2);
RFU_EM   = RFU(:,:,3);
RFU_US   = RFU(:,:,4);
RGU_EL   = RGU(:,:,1);
RGU_EM   = RGU(:,:,2);
RGU_RS   = RGU(:,:,3);
RGU_US   = RGU(:,:,4);
RHU_MC2h = RHU(:,:,1);
RHU_MC3h = RHU(:,:,2);
RHU_MC4h = RHU(:,:,3);
RHU_MC3b = RHU(:,:,4);

% RAE
EpC7    = REU_C7'*(UpC7 - UpE_C7);
EpT8    = REU_T8'*(UpT8 - UpE_T8);
EpIJ    = REU_IJ'*(UpIJ - UpE_IJ);
EpPX    = REU_PX'*(UpPX - UpE_PX);

% EpC7    = REU_C7'*UpC7 - UpE_C7;
% EpT8    = REU_T8'*UpT8 - UpE_T8;
% EpIJ    = REU_IJ'*UpIJ - UpE_IJ;
% EpPX    = REU_PX'*UpPX - UpE_PX;

EyA = (EpIJ + EpC7)/2 - (EpPX + EpT8)/2;
EyA = EyA/norm(EyA);
EzA = cross(EyA, EpC7 - EpIJ);
EzA = EzA/norm(EzA);
ExA = cross(EyA,EzA);

RAE = [ExA, EyA, EzA];

% RCG 
GpEL    = RGU_EL'*(UpEL - UpG_EL);
GpEM    = RGU_EM'*(UpEM - UpG_EM);
GpRS    = RGU_RS'*(UpRS - UpG_RS);
GpUS    = RGU_US'*(UpUS - UpG_US);

% GpEL    = RGU_EL'*UpEL - UpG_EL;
% GpEM    = RGU_EM'*UpEM - UpG_EM;
% GpRS    = RGU_RS'*UpRS - UpG_RS;
% GpUS    = RGU_US'*UpUS - UpG_US;


GyC = (GpEL + GpEM)/2 - (GpRS+GpUS)/2;
% THIS LINE ABOVE WAS CHANGED FROM GyC = (GpEL + GpEM)/2 - GpUS;
% This change was done in order to make the LM Calibration more like the
% posture calibration method 10/20/16

GyC = GyC/norm(GyC);
GxC = cross(GyC, GpRS - GpUS);
GxC = GxC/norm(GxC);
GzC = cross(GxC,GyC);
%GzC = cross(GyC,GxC); %as of 8/12/16 in the code

RCG = [GxC, GyC, GzC];

% RBF
FpEL    = RFU_EL'*(UpEL - UpF_EL);
FpEM    = RFU_EM'*(UpEM - UpF_EM);

% FpEL    = RFU_EL'*UpEL - UpF_EL;
% FpEM    = RFU_EM'*UpEM - UpF_EM;


FyB = FpGH - (FpEL + FpEM)/2;
FyB = FyB/norm(FyB);
FyC = RFU_US'*RGU_US*GyC;
FzB = cross(FyB,FyC);
FzB = FzB/norm(FzB);
FxB = cross(FyB,FzB);

RBF = [FxB, FyB, FzB];

% RDH
HpMC2h = RHU_MC2h'*(UpMC2h - UpH_MC2h);
HpMC3h = RHU_MC3h'*(UpMC3h - UpH_MC3h);
HpMC4h = RHU_MC4h'*(UpMC4h - UpH_MC4h);
HpMC3b = RHU_MC3b'*(UpMC3b - UpH_MC3b);

% HpMC2h = RHU_MC2h'*UpMC2h - UpH_MC2h;
% HpMC3h = RHU_MC3h'*UpMC3h - UpH_MC3h;
% HpMC4h = RHU_MC4h'*UpMC4h - UpH_MC4h;
% HpMC3b = RHU_MC3b'*UpMC3b - UpH_MC3b;

HyD = HpMC3b - HpMC3h;
HyD = HyD/norm(HyD);
HxD = cross(HyD,(HpMC2h - HpMC4h));
%HxD = cross(HyD,(-HpMC4h + HpMC2h)); possible fix? 8/13/16
HxD = HxD/norm(HxD);
HzD = cross(HxD,HyD);

RDH = [HxD, HyD, HzD];

RBS(:,:,1) = RAE;
RBS(:,:,2) = RBF;
RBS(:,:,3) = RCG;
RBS(:,:,4) = RDH;

% Vector of landmark positions relative to SCS
SpL(:, 1) = EpC7;
SpL(:, 2) = EpT8;
SpL(:, 3) = EpIJ;
SpL(:, 4) = EpPX;
SpL(:, 5) = FpGH;
SpL(:, 6) = FpEL;
SpL(:, 7) = FpEM;
SpL(:, 8) = GpEL;
SpL(:, 9) = GpEM;
SpL(:,10) = GpRS;
SpL(:,11) = GpUS;
SpL(:,12) = HpMC2h;
SpL(:,13) = HpMC3h;
SpL(:,14) = HpMC4h;
SpL(:,15) = HpMC3b;