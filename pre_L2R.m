 function [UpL, UpS, REU, RFU, RGU, RHU] = pre_L2R(mat, s_opt)

% This function takes the output from trakSTAR and returns a matrix of
% landmark positions and matrices containing rotation matrices for each
% landmark
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
% OUTPUTS
% UpL   3x12 matrix in which each column describes the position of a
%       landmark (see below) relative to the transmitter
% UpS   3x14 matrix in which each column describes the position of a sensor
%       relative to the transmitter at the moment when the position of a
%       landmark was measured (see below)
% REU    3x3x4 matrix in which each 3x3 matrix is a rotation matrix
%       describing the orientation of E relative to U at the moment when
%       the position of a landmark was measured (see below)
% RFU   Similar to REU
% RGU   Similar to REU
% RHU   Similar to REU
%
% NOTATION
% Vectors:  UpE_C7 means the vector describing the position of sensor E relative to
%           frame U at the moment C7 was touched with the stylus
% Matrices: REU_C7 means the rotation matrix describing frame E relative to
%           frame U at the moment C7 was touched with the stylus
% In addition to the usual frames (U, A, B, C, D, E, F, G, and H), frame S
% stands for the frame of any generic sensor. Likewise, in addition to the
% usual landmarks described below, L represents any generic landmark.

% Pre-allocate memory
UpL    = zeros(3,12);
REU     = zeros(3,3,4);
RFU     = zeros(3,3,4);
RGU     = zeros(3,3,4);
RHU     = zeros(3,3,4);

% DETERMINE MOMENTS THAT LANDMARKS WERE MEASURED
% Each moment is marked by a row of NaN immediately following the moment
k = find(isnan(mat(:,1))) - 1;
%Checking for duplicate Mark inputs or start stop or pause inputs
for s = 1:length(k)
    for i = 1:length(k)-1
        if (k(i+1) - k(i)) == 1
            k(i+1) = [];
            k(i) = [];
            break;
        end
    end
end

if(length(k)~= 16)
    ['ERROR: THERE SHOULD BE 16 LANDMARKS, BUT THERE ARE ' num2str(length(k))]
end
% The order of landmark measurements is:
%  1. C7
%  2. T8
%  3. IJ
%  4. PX
%  5. EL
%  6. EM
%  7. RS
%  8. US
%  9. MC2h
% 10. MC3h
% 11. MC4h
% 12. MC3b

% EXTRACT LANDMARK POSITIONS AND CONVERT TO FRAME U
% Columns 37-39 contain the xyz-position of the sensor on the stylus, and
% Columns 40-42 contain the Euler angles (in deg) of the sensor on the stylus
for i = 1:12
    UpL(:,i) = stylus(mat(k(i),37:39)',mat(k(i),40:42), s_opt);
end

% EXTRACT SENSOR POSITION
UpE_C7     = mat(k( 1), 2: 4)';
UpE_T8     = mat(k( 2), 2: 4)';
UpE_IJ     = mat(k( 3), 2: 4)';
UpE_PX     = mat(k( 4), 2: 4)';
UpF_EL     = mat(k( 5), 9:11)';
UpF_EM     = mat(k( 6), 9:11)';
UpG_EL     = mat(k( 5),16:18)';
UpG_EM     = mat(k( 6),16:18)';
UpG_RS     = mat(k( 7),16:18)';
UpG_US     = mat(k( 8),16:18)';
UpH_MC2h   = mat(k( 9),23:25)';
UpH_MC3h   = mat(k(10),23:25)';
UpH_MC4h   = mat(k(11),23:25)';
UpH_MC3b   = mat(k(12),23:25)';

UpS = [UpE_C7, UpE_T8, UpE_IJ, UpE_PX, UpF_EL, UpF_EM, UpG_EL,...
    UpG_EM, UpG_RS, UpG_US, UpH_MC2h, UpH_MC3h, UpH_MC4h, UpH_MC3b];

% EXTRACT ROTATION MATRICES FROM INPUT MATRICES
% Columns  5- 7 contain a, e, and r for sensor E (in deg)
% Columns 12-14 contain a, e, and r for sensor F (in deg)
% Columns 19-21 contain a, e, and r for sensor G (in deg)
% Columns 26-28 contain a, e, and r for sensor H (in deg)
REU_C7  = aer2R(mat(k(1), 5), mat(k(1), 6), mat(k(1), 7));
REU_T8  = aer2R(mat(k(2), 5), mat(k(2), 6), mat(k(2), 7));
REU_IJ  = aer2R(mat(k(3), 5), mat(k(3), 6), mat(k(3), 7));
REU_PX  = aer2R(mat(k(4), 5), mat(k(4), 6), mat(k(4), 7));

RFU_EL  = aer2R(mat(k(5),12), mat(k(5),13), mat(k(5),14));
RFU_EM  = aer2R(mat(k(6),12), mat(k(6),13), mat(k(6),14));
RFU_US  = aer2R(mat(k(8),12), mat(k(8),13), mat(k(8),14));

RGU_EL  = aer2R(mat(k(5),19), mat(k(5),20), mat(k(5),21));
RGU_EM  = aer2R(mat(k(6),19), mat(k(6),20), mat(k(6),21));
RGU_RS  = aer2R(mat(k(7),19), mat(k(7),20), mat(k(7),21));
RGU_US  = aer2R(mat(k(8),19), mat(k(8),20), mat(k(8),21));

RHU_MC2h= aer2R(mat(k( 9),26), mat(k( 9),27), mat(k( 9),28));
RHU_MC3h= aer2R(mat(k(10),26), mat(k(10),27), mat(k(10),28));
RHU_MC4h= aer2R(mat(k(11),26), mat(k(11),27), mat(k(11),28));
RHU_MC3b= aer2R(mat(k(12),26), mat(k(12),27), mat(k(12),28));

% Package the data for efficient return
REU(:,:,1) = REU_C7;
REU(:,:,2) = REU_T8;
REU(:,:,3) = REU_IJ;

REU(:,:,4) = REU_PX;

RFU(:,:,2) = RFU_EL;
RFU(:,:,3) = RFU_EM;
RFU(:,:,4) = RFU_US;

RGU(:,:,1) = RGU_EL;
RGU(:,:,2) = RGU_EM;
RGU(:,:,3) = RGU_RS;
RGU(:,:,4) = RGU_US;

RHU(:,:,1) = RHU_MC2h;
RHU(:,:,2) = RHU_MC3h;
RHU(:,:,3) = RHU_MC4h;
RHU(:,:,4) = RHU_MC3b;