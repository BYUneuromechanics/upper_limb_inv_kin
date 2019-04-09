function [s_opt, deff] = ICR(t,p,R)

% This function calculates the optimal estimate of the center of rotation
% and is based on "Woltring, H. (1990). Data Processing and error analysis.
% Biomechanics of Human Movement: Applications in Rehabilitation, Sport and
% Ergonomics. A. Capozza and P. Berme. Worthington, OH, Berlec Corporation:
% 203-237."
%
% INPUTS
% t     1xN time vector, where N is the number of samples
% p     3xN vector from the origin of frame A to the origin of frame B,
%       expressed in terms of frame A
% R     3x3xN matrix of rotation matrices of frame B relative to frame A,
%       i.e. R = [AXB AYB AZB], where AXB is the X-axis of B expressed in 
%       terms of frame A
%
% OUTPUTS
% s_opt 3x1 vector from the origin of frame A to the optimal estimate of
%       the center of rotation, expressed in terms of frame A
% deff  Scalar representing rms distance from s_opt to instaneous helical axes     

% Pre-allocate memory
p_dot = zeros(3,length(t));
R_dot = zeros(3,3,length(t));
Qi = zeros(3,3,length(t));
w = zeros(3,length(t));
w_mag = zeros(1,length(t));
n = zeros(3,length(t));
v = zeros(3,length(t));
s = zeros(3,length(t));
s_prime = zeros(3,length(t));
deff = zeros(size(t));

% Determine derivatives of p and R
for i = 1:length(t)
    if i == 1
        p_dot(:,i) = (p(:,i+1) - p(:,i)) / (t(i+1) - t(i));
        R_dot(:,:,i) = (R(:,:,i+1) - R(:,:,i)) / (t(i+1) - t(i));
    elseif i == length(t)
        p_dot(:,i) = (p(:,i) - p(:,i-1)) / (t(i) - t(i-1));
        R_dot(:,:,i) = (R(:,:,i) - R(:,:,i-1)) / (t(i) - t(i-1));
    else
        p_dot(:,i) = (p(:,i+1) - p(:,i-1)) / (t(i+1) - t(i-1));
        R_dot(:,:,i) = (R(:,:,i+1) - R(:,:,i-1)) / (t(i+1) - t(i-1));
    end
end

% Calculate instaneous helical axes
for i = 1:length(t)
    W = R_dot(:,:,i)*R(:,:,i)';
    w(:,i) = [W(3,2), W(1,3), W(2,1)]';
    
    w_mag(i) = norm(w(:,i));
    n(:,i) = w(:,i)/w_mag(i);
    v(:,i) = dot(p_dot(:,i),n(:,i));
    s(:,i) = p(:,i) + cross(w(:,i),p_dot(:,i))/(w_mag(i)^2);

    Qi(:,:,i) = eye(3) - n(:,i)*n(:,i)';
    s_prime(:,i) = Qi(:,:,i)*s(:,i);
end

% Before determining s_opt, remove instances where w_mag is too low
w_mag_min = 0.25;
k = find(w_mag < w_mag_min);
Qi(:,:,k) = [];
s_prime(:,k) = [];
s(:,k) = [];

% Determine s_opt
Q = mean(Qi,3);
s_opt = Q\mean(s_prime,2);

% Determine deff
for i = 1:size(s,2)
    deff(i) = (s_opt - s(:,i))'*Qi(:,:,i)*(s_opt - s(:,i));
end
deff = sqrt(mean(deff));

% % Plot the data and center of rotation
% figure(1)
% subplot(2,2,4)
% hold on
% plot(t,w_mag*180/pi,'k')
% 
% figure(2)
% hold on
% for i = 1:size(s,2)
% 
%     x(:,i) = p(:,i) + R(:,:,i)*[1 0 0]';
%     y(:,i) = p(:,i) + R(:,:,i)*[0 1 0]';
%     z(:,i) = p(:,i) + R(:,:,i)*[0 0 1]';
% 
%     line([p(1,i);x(1,i)],[p(2,i);x(2,i)],[p(3,i);x(3,i)],'Color',[0 0 1]);
%     line([p(1,i);y(1,i)],[p(2,i);y(2,i)],[p(3,i);y(3,i)],'Color',[0 0.5 0]);
%     line([p(1,i);z(1,i)],[p(2,i);z(2,i)],[p(3,i);z(3,i)],'Color',[1 0 0]);
%     
%     plot3(s(1,i),s(2,i),s(3,i),'ko')
%     line([s(1,i)-w(1,i);s(1,i)+w(1,i)],[s(2,i)-w(2,i);s(2,i)+w(2,i)],...
%         [s(3,i)-w(3,i);s(3,i)+w(3,i)],'Color',[0 0 0],'LineStyle',':');
% end
% 
% plot3(s_opt(1),s_opt(2),s_opt(3),'ro','MarkerFaceColor','r')
% set(gca,'Xdir','reverse')
% set(gca,'Zdir','reverse')
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% view(-37.5+75,30)