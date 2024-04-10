setenv('ROS_MASTER_URI','http://192.168.35.251:11311');
setenv('ROS_IP','192.168.114.38');
rosshutdown()
rosinit('http://192.168.114.251:11311','NodeHost','192.168.114.38')

%% "Laser scan" line fit example
clear


scansub = rossubscriber('/master/scan');

while(1)
% plot(scan)
% while(1)
% get data from scan
scan = receive(scansub);
cart = readCartesian(scan);
% figure()
hold on
xlim([-1 1])
ylim([-1 1])
plot(cart(:,2), cart(:,1), '.') % note - y before x..

x = cart(:,2); % x-pos
d = cart(:,1); % depth

%% Fitting one line..

% Define the coefficients of the line from the linear regression model
mdl = fitlm(x, d);
coef = mdl.Coefficients.Estimate;

% Extract intercept and slope
intercept = coef(1);
slope = coef(2);

plot(x, intercept + slope * x, 'r')

% Calculate the distance from the origin to the line
distance = abs(intercept) / sqrt(1 + slope^2);

fprintf('Closest distance from (0,0) to the line: %f\n', distance);





%% Multi-line method..

% N = 100; % grid size
% 
% xoffset = min(x);
% xscale = (max(x)-min(x));
% xi = (x - xoffset)/xscale; % scale to 0-1
% xi = round(xi*(N-1))+1; % scale and integer 1 to N
% di = (d - min(d))/(max(d)-min(d)); % scale to 0-1
% di = round(di*(N-1))+1;
% ind = sub2ind([N N], xi, di); % matrix indices
% 
% mat = zeros(N); % NxN matrix
% mat(ind) = 1; % with points..
% % figure()
% % imshow(mat), colorbar()
% 
% %% Hough transform
% [H,theta,rho] = hough(mat);
% 
% % Hough peaks
% Nlines = 5; % Note - Nlines peaks/lines !
% P  = houghpeaks(H,Nlines); 
% % figure()
% % imshow(H,[],'XData',theta,'YData',rho,'InitialMagnification','fit');
% % xlabel('\theta'), ylabel('\rho');
% % axis on, axis normal, hold on;
% % figure()
% plot(theta(P(:,2)),rho(P(:,1)),'s','color','white');
% 
% %% Hough lines
% lines = houghlines(mat,theta,rho,P,'FillGap',30,'MinLength',30);
% 
% % figure()
% imshow(mat), hold on,
% xlim([0 100])
% ylim([0 100])
% 
% % for k = 1:length(lines)
% try
%     xy = [lines(1).point1; lines(1).point2];
%     plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
% catch exception
% 
% end
% % end


end
