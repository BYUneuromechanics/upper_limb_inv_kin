function UpT = stylus(xyz, aer, s_opt)

% This function takes in the position and orientation of the sensor
% attached to the stylus and returns the position of the tip of the stylus,
% expressed in the transmitter frame U.
%
% INPUTS
% xyz   3x1 vector describing the position of the sensor in frame U
% aer   Euler angles of the sensor relative to frame U
%
% OUTPUT
% UpT   3x1 vector describing the position of the tip of the stylus,
%       expressed in frame U

RSU = aer2R(aer(1),aer(2),aer(3));

UpO = xyz;
%load('s_opt.mat');    % s_opt is a 3-element vector describing the position of
                        % the tip of the stylus in the frame of the sensor
SpT = s_opt;

UpT = UpO + RSU*SpT;