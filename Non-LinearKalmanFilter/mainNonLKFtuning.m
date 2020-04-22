clear all; 
%close all; 
clc;

% True track
% Sampling period
T = 0.1;
% Length of time sequence
K = 600;
% Allocate memory
omega = zeros(1,K+1);
% Turn rate
omega(200:400) = -pi/201/T;
% Initial state
x0 = [0 0 20 0 omega(1)]';
% Allocate memory
X = zeros(length(x0),K+1);
X(:,1) = x0;
% Create true track
for i=2:K+1
    % Simulate
    X(:,i) = coordinatedTurnMotion(X(:,i-1), T);
    % Set turn?rate
    X(5,i) = omega(i);
end

%Prior information
X_0 = [0 0 0 0 0]';
P_0 = diag([10 10 10 5*pi/180 pi/180].^2);
% Sensor positions
s1 = [280 -80]';
s2 = [280 -200]';

% measurement noise
R = diag([4*pi/180 4*pi/180].^2);
% Process Noise
sigma_V =200;
sigma_W = 200*pi/180 ;

Q = diag([0 0 T*sigma_V^2 0 T*sigma_W^2]);
% Motion model function handle. Note how sample time T is inserted into the function.
motionModel = @(x) coordinatedTurnMotion(x, T);

% This is the measurement model
measModel = @(X) dualBearingMeasurement(X, s1, s2)

% Generate measurements sequences
Y = genNonLinearMeasurementSequence(X, measModel, R);

% calcualte unfiltered position from sensors given angles
Xm(1,:) = ( s2(2)-s1(2) + tan(Y(1,:))*s1(1) - tan(Y(2,:))*s2(1) ) ./ ( tan(Y(1,:)) - tan(Y(2,:)) );
Xm(2,:) = s1(2) + tan(Y(1,:)) .* ( Xm(1,:) - s1(1) );

   % filter
[xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, X_0, P_0, motionModel, Q, measModel, R, 'UKF');

% figure('Color','white','Position',[758  175  603  429]);
figure('Color','white','Position',[520  180  654  417]);
grid on; hold on, axis equal;

for i=1:15:length(xf)
    ell_xy = sigmaEllipse2D(xf(1:2,i),Pf(1:2,1:2,i),3,50);
%     fill(ell_xy(1,:),ell_xy(2,:), '--', 'Color',cp(5,:), 'DisplayName','3-sigma level');
    p4 = fill(ell_xy(1,:),ell_xy(2,:), [0, 0.4470, 0.7410],'facealpha',.1, 'DisplayName','3-sigma level');   %,'edgecolor','none'
end

p1 = plot(X(1,:),X(2,:), 'Color', 'blue', 'LineWidth',2, 'DisplayName','True position');
    
p2 = plot(xf(1,:),xf(2,:), 'Color', 'red', 'LineWidth',2, 'DisplayName','estimated position');
    
sc1 = scatter(s1(1), s1(2), 100, 'o', 'MarkerFaceAlpha',0.8, 'MarkerFaceColor', [.5 0 .5], 'MarkerEdgeColor', [.5 0 .5],'DisplayName','sensor 1 location');
sc2 = scatter(s2(1), s2(2), 200, 'h', 'MarkerFaceAlpha',0.8, 'MarkerFaceColor', [.5 0 .5], 'MarkerEdgeColor', [.5 0 .5],'DisplayName','sensor 2 location');

axis manual
p3 = plot(Xm(1,:),Xm(2,:), 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',1, 'DisplayName','Measured position');
    
xlabel 'pos x', ylabel 'pos y'
%title(sprintf('Case %d, filter type: %s',case_i,filter_type{1}))
legend([p1 p2 p3 p4 sc1 sc2], 'Location','southwest')

% plot position error
figure('Color','white','Position',[428  692  930  207]);
grid on, hold on;
plot( (1:K)*T, vecnorm(xf(1:2,:)-X(1:2,2:end), 2, 1) , 'LineWidth',1)
ylabel('$|p_k - \hat{p_{k|k}}|_2$', 'Interpreter','Latex', 'FontSize',16), xlabel('Time [s]')
title 'Position error'