function [pCR, deff] = pre_ICR(mat)

% This function takes the output from trakSTAR and calls the ICR function
% to return the center of rotation relative to a frame. This function works
% for finding the tip of the stylus (Case 1) expressed in the frame of the stylus
% (assuming the stylus was rotated about a stationary point), and for
% finding the center of rotation of the glenohumeral joint expressed in
% the frame of SCS F (Case 2).
%
% INPUTS
%
% Case 1
% mat   Nx8 matrix, where N is the number of samples
%       Columns  1- 7 contain sensor #, xyz, and aer for sensor
%       Column      8 contains time
%
% Case 2
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
% pCR   3x1 vector from the origin of frame A to the optimal estimate of
%       the center of rotation, expressed in terms of frame A (also known
%       as the glenohumeral joint)
% deff  Scalar representing rms distance from s_opt to instaneous helical axes

close all
dt = 0.1;

if size(mat,2) == 8 % CASE 1
    
    mat(:,1)=[];
    t = mat(:,end);
    t = t(:)-t(1);
    mat(:,end)=[];
    aer = mat(:,4:6);
    R = aer2R(aer(:,1),aer(:,2),aer(:,3));

    for i = 1:size(R,3)
        R(:,:,i) = R(:,:,i)';
    end
    
    time = repmat(t',3,1)';
    aer_dot = diff(aer)./diff(time);
    aer_dot(end+1,:) = aer_dot(end,:);
    w_mag = sqrt(aer_dot(:,1).^2 + aer_dot(:,2).^2 + aer_dot(:,3).^2 - 2*aer_dot(:,1).*aer_dot(:,3).*sin(aer(:,2)));

    xyz = mat(:,1:3);
    p = xyz';
    for i = 1:size(p,2)
        p(:,i) = R(:,:,i)*(-p(:,i));
    end
    
%     PLOTS COMMENTED OUT FOR ANALYSIS CONVENIENCE IN ICR AND PRE-ICR 4/2/17

%     figure(1)
%     subplot(2,2,1)
%     plot3(xyz(:,1),xyz(:,2),xyz(:,3))
%     set(gca,'Ydir','reverse')
%     set(gca,'Zdir','reverse')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
%     axis equal
%     grid on
% 
%     subplot(2,2,2)
%     plot(t,xyz)
%     xlabel('Time [s]')
%     ylabel('Position [m]')
% 
%     subplot(2,2,3)
%     plot(t,aer*180/pi)
%     xlabel('Time [s]')
%     ylabel('Angle [deg]')
% 
%     subplot(2,2,4)
%     plot(t,[w_mag]*180/pi)
%     xlabel('Time [s]')
%     ylabel('Velocity [deg/s]')
%     
else % CASE 2
    
    % The relevant information is contained in the following columns of
    % mat:
    xyzFcol =  9:11;
    aerFcol = 12:14;
    xyzScol = 30:32;
    aerScol = 33:35;
    
    t = mat(:,end);
    t = t(:)-t(1);
    aerF = mat(:,aerFcol);
    aerS = mat(:,aerScol);
    RFU = aer2R(aerF(:,1),aerF(:,2),aerF(:,3));
    RSU = aer2R(aerS(:,1),aerS(:,2),aerS(:,3));
    
    for i = 1:size(RFU,3)
        RSF(:,:,i) = RFU(:,:,i)'*RSU(:,:,i);
    end

    xyzF = mat(:,xyzFcol);
    xyzS = mat(:,xyzScol);
    UpF = xyzF';
    UpS = xyzS';
    for i = 1:size(UpF,2)
        FpS(:,i) = RFU(:,:,i)'*(UpS(:,i) - UpF(:,i));
    end
    
    p = FpS;
    R = RSF;
    
%     figure(1)
%     subplot(2,2,1)
%     plot3(xyzF(:,1),xyzF(:,2),xyzF(:,3))
%     hold on
%     plot3(xyzS(:,1),xyzS(:,2),xyzS(:,3),':')
%     set(gca,'Xdir','reverse')
%     set(gca,'Zdir','reverse')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
%     axis equal
%     grid on
%     view(-37.5+75,30)
% 
%     subplot(2,2,2)
%     plot(t,xyzF)
%     hold on
%     plot(t,xyzS,':')
%     xlabel('Time [s]')
%     ylabel('Position [m]')
% 
%     subplot(2,2,3)
%     plot(t,aerF*180/pi)
%     hold on
%     plot(t,aerS*180/pi,':')
%     xlabel('Time [s]')
%     ylabel('Angle [deg]')
% 
%     subplot(2,2,4)
% %     plot(t,[w_mag]*180/pi)
%     xlabel('Time [s]')
%     ylabel('Velocity [deg/s]')
%     
end

[pCR, deff] = ICR(t,p,R);