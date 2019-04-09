function [abg_s, abg_e, abg_w, t] = sens2joint(Mmat, CALmat, Gmat, s_opt, approx)

% This function takes in trakSTAR data and derives sets of joint angles 
% depending on the desired approximation and calibration method for the 
% shoulder, elbow, and wrist.
%
% IMPORTANT INFORMATION

% --- Each raw data file must first be imported as a numeric matrix and saved as
%     a .mat file before running any aspect of the program.
% --- The following files are required for all aspects of this program to 
%     function correctly:
%       1. trakSTARunits2SIunits.m
%       2. stylus.m
%       3. sensAng2RU.m
%       4. R2abg_7DOF.m
%       5. R2abg.m
%       6. pre_P2R.m
%       7. pre_L2R.m
%       8. pre_ICR.m
%       9. P2R.m
%       10. MatMult3.m
%       11. L2R.m
%       12. ICR.m
%       13. CalibrationL.m
%       14. CalibrationP.m
%       15. aer2R.m

% INPUTS:
% Mmat is an Nx43 matrix of trakSTAR data recorded as the subject made a number of
%       movements.
%
% CALmat is an Nx43 matrix that can contain either the Landmark calibration
%       data or Posture calibration data.
%           - Landmark data matrix contains the data describing the position of the
%           landmarks when they are touched with the stylus(N is the number of
%           samples).
%           - Posture data matrix contains the data taken at the moment of 
%           the posture calibration, i.e. when the BCS were aligned with the 
%           transmitter frame.
%
% Gmat is an  Nx43 matrix containing the data related to finding the center of 
%       rotation of the glenohumeral joint. This input should only be used
%       for the landmark calibration method. If the posture calibration
%       method is being implemented, the value for Gmat should be [].
%
% s_opt is a 3-element vector describing the position of the tip of the
%     stylus in the frame of the sensor. If no calibration was performed, 
%     the default input should be [0;0;0]. 
%     In order to obtain the proper s_opt vector follow these steps:
%       1. Open the file pre_ICR and read the information of CASE 1
%       2. The Nx8 callibration movement data matrix, which will be used as 
%       the mat input, should be run through the trakSTARunits2SIunits function.
%       3. Run the pre_ICR function with the mat input. The output of pCR
%       should be the desired s_opt vector.
%
% approx specifies the level of approximation:
% 0: Full 9-DOF model, i.e. no approximation
% 1: 7-DOF model (be = gw = 0)
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
% OUTPUTS
% abg_s is a nx3 vector of alpha, beta, and gamma for the shoulder joint
% abg_e is a nx3 vector of alpha, beta, and gamma for the elbow/forearm joint
% abg_w is a nx3 vector of alpha, beta, and gamma for the wrist joint
% t is a vector of time for the movement data held in sMmat
%
% ADJUSTMENTS
% If you are using a 7-DOF model with a non-zero value for beta_e or
% gamma_w, you need to specify those within this function (see comments
% below).
%
% Some applications favor a particular order for the Euler angles of the
% shoulder.  These can be selected in R2abg.m or R2abg_7DOF.m, depending on
% which approximation you are using.  The standard is ZXY, but the
% equations for YXY are provided. You just need to comment/uncomment the
% relevant sections.
%
% EXAMPLE of function sens2joint use:
%
%   Let's say I want the joint angles using a 7 DOF approximation using Landmark
%   calibration. Let's also assume I don't want to take the time to determine
%   the s_opt vector. The implementation is as follows:
%
%   [abg_s, abg_e, abg_w, t] = sens2joint(Mmat, CALmat, Gmat, [0,0,0], 1);
%       where CALmat is the landmark calibration data matrix
%   
%   This would return the desired joint angles.
%   
%   If I wanted to use Posture calibration for the same approximation
%   instead, then I would implement the function as follows:
%
%   [abg_s, abg_e, abg_w, t] = sens2joint(Mmat, CALmat, [], [0,0,0], 1);
%       where CALmat is the posture calibration matrix
%   
%   This would return the desired joint angles
%

% Written by Dr. Steven Charles, Taylor Dickinson, and Ryan Clark, 2016.



% Determine the calibration rotation matricies according to the inputs:
sMmat = trakSTARunits2SIunits(Mmat);
if isempty(Gmat)
    sPmat = trakSTARunits2SIunits(CALmat);
    [RAE,RBF,RCG,RDH] = CalibrationP(sPmat);
else
    sGmat = trakSTARunits2SIunits(Gmat);
    sLmat = trakSTARunits2SIunits(CALmat);
    [RAE,RBF,RCG,RDH] = CalibrationL(sGmat, sLmat, s_opt);
end

% Extraction of motion data:
[REUt, RFUt, RGUt, RHUt, t] = sensAng2RU(sMmat);

% Compute the JCS Rotation Matricies depending on approximation:
if approx == 0 || approx == 1
RBAt = MatMult3(RAE, RBF, REUt, RFUt);
RCBt = MatMult3(RBF, RCG, RFUt, RGUt);
RDCt = MatMult3(RCG, RDH, RGUt, RHUt);
else
    error('ERROR: NOT A VALID APPROXIMATION NUMBER, VALID NUMBERS ARE 0-1');
end

% Extract Joint Angles from the JCS rotation matricies depending on
% approximation:
if approx == 0
    [as,bs,gs] = R2abg(RBAt, 1);
    [ae,be,ge] = R2abg(RCBt, 2);
    [aw,bw,gw] = R2abg(RDCt, 3);
else
    be = 0;     % If you are using a 7-DOF model with a non-zero beta_e
                % value, specify here (in rad).
    gw = 0;     % If you are using a 7-DOF model with a non-zero gamma_w
                % value, specify here (in rad).
    [as,bs,gs] = R2abg_7DOF(RBAt, 1, be, gw);
    [ae,be,ge] = R2abg_7DOF(RCBt, 2, be, gw);
    [aw,bw,gw] = R2abg_7DOF(RDCt, 3, be, gw);
end

% Prepare output and convert from radians to degrees
abg_s = [as,bs,gs];
abg_e = [ae,be,ge];
abg_w = [aw,bw,gw];

abg_s = abg_s*180/pi;
abg_e = abg_e*180/pi;
abg_w = abg_w*180/pi;