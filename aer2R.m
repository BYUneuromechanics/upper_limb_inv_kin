function R = aer2R(a,e,r)
% This function takes sensor angles (radians) and creates their rotation matricies
% a is azimuth or yaw, e is elevation or pitch, and r is roll

% Preallocate memory
R = zeros(3,3,length(a));
    
for i = 1:length(a)
    
    Rz_a = [cos(a(i)) -sin(a(i)) 0; sin(a(i)) cos(a(i)) 0; 0 0 1];
    Ry_e = [cos(e(i)) 0 sin(e(i)); 0 1 0; -sin(e(i)) 0 cos(e(i))];
    Rx_r = [1 0 0; 0 cos(r(i)) -sin(r(i)); 0 sin(r(i)) cos(r(i))];
    % the i creates a 3D matix to keep track of each individual 2D
    % matrix created from multiplying the three 3x3 matricies together in
    % order to keep track of all euler angles
    
    % z*y*x is how the euler angles are defined for trakSTAR
    R(:,:,i) = Rz_a * Ry_e * Rx_r;
end