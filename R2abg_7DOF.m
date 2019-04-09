function [a,b,g] = R2abg_7DOF(R,joint,be,gw)

% This function takes in a rotation matrix and returns joint angles
% a is alpha, b is beta, and g is gamma; all angles
% Joint 1 is shoulder, 2 is elbow, and 3 is wrist

a = zeros(size(R,3),1);
b = a;
g = a;

for i = 1:size(R,3)
    
    if joint == 1
        % SHOULDER JOINT
        % YXY Euler angle sequence
%         b(i) = atan2(sqrt(R(2,1,i)^2 + R(2,3,i)^2), R(2,2,i));
%         a(i) = atan2(R(1,2,i)/sin(b(i)), R(3,2,i)/sin(b(i)));
%         g(i) = atan2(R(2,1,i)/sin(b(i)), -R(2,3,i)/sin(b(i)));

        % ZXY Euler angle sequence
        b(i) = atan2(R(3,2,i), sqrt(R(3,1,i)^2 + R(3,3,i)^2));
        a(i) = atan2(-R(1,2,i)/cos(b(i)), R(2,2,i)/cos(b(i)));
        g(i) = atan2(-R(3,1,i)/cos(b(i)), R(3,3,i)/cos(b(i)));
        
    elseif joint == 2
        % ELBOW/FOREARM JOINT
        % Extract the only possible set
        b(i) = be;
        a(i) = atan2(-R(1,2,i)/cos(b(i)), R(2,2,i)/cos(b(i)));
        g(i) = atan2(-R(3,1,i)/cos(b(i)), R(3,3,i)/cos(b(i)));
        
    elseif joint == 3
        % WRIST JOINT
        % Extract the only possible set
        g(i) = gw;
        b(i) = atan2(R(3,2,i), sqrt(R(3,1,i)^2 + R(3,3,i)^2));
        a(i) = atan2(-R(1,2,i)/cos(b(i)), R(2,2,i)/cos(b(i)));
    end
    
end