clc;
%close all
clear all;

% Number of data points for measurement generation
N = 50000;
% Define prior state and covariances for the two different state densities
state_densities = 2; % This can be 1 or 2

%Setting the state_densities

if state_densities == 2
    x_mu = [120,120];
    x_sigma = [5^2 0; 0 10^2];
else
    x_mu = [120,-20];
    x_sigma = [5^2 0; 0 10^2];
end

% sensor positions 
s1 = [0, 100]';
s2 = [100, 0]';

% Measurement noise covariance
sigR1 = 0.1*pi/180;
sigR2 = 0.1*pi/180;
R = diag([sigR1 sigR2].^2);

% Measurement model. This is found under Motion and Measurement Model
% directory
measModel = @(X) dualBearingMeasurement(X, s1, s2)

%Approximating the transformed gaussian dist as a gaussian using previously
%implemented function

fun_handle = @(x)genNonLinearMeasurementSequence(x,measModel,R);

[y_mu, y_sigma, ys,xs] = approxGaussianTransform( x_mu, x_sigma, fun_handle,N);

% Choosing the filter type

filter_type = 'CKF'
% Estimating the transformed mean and covariance using the different
% filters
if strcmp(filter_type,'UKF') || strcmp(filter_type,'CKF')
    [SP1,W1] = sigmaPoints(x_mu,x_sigma,filter_type)
    hSP1 = measModel(SP1)
    [ye_mu, ye_sigma]  = unscentedTransform(measModel, R, SP1, W1)
elseif strcmp(filter_type,'EKF')
    [hx, dhx] = measModel(x_mu);
    ye_mu = hx;
    ye_sigma = dhx * x_sigma * dhx' + R;
end


figure('Color','white','Position',[651  364  588  441]);
grid on; hold on %, axis equal

sc1 = scatter(ys(1,:), ys(2,:), 20, 'filled', 'MarkerFaceColor', 'b', 'MarkerFaceAlpha',0.1, 'DisplayName','y = h(x) + r');
[ xy ] = sigmaEllipse2D( y_mu, y_sigma, 3, 100 );
p1 = plot(xy(1,:),xy(2,:), 'Color', 'r', 'LineWidth',2, 'DisplayName','Sample 3-sigma ellipse');

sc2 = scatter(y_mu(1), y_mu(2), 100,'o','filled', 'MarkerFaceAlpha',0.5, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r','DisplayName','Sample mean');


if strcmp(filter_type,'UKF') || strcmp(filter_type,'CKF')
    sc3 = scatter(hSP1(1,:), hSP1(2,:), 100, 'h','filled', 'MarkerFaceAlpha',1, 'MarkerFaceColor', [.5 0 .5], 'MarkerEdgeColor', [.5 0 .5],'DisplayName','Sigma points');
end

sc4 = scatter(ye_mu(1,:), ye_mu(2,:), 100, 'o','filled', 'MarkerFaceAlpha',0.8, 'MarkerFaceColor', [.5 0 .5], 'MarkerEdgeColor', [.5 0 .5],'DisplayName','Approximated mean');

[ xy ] = sigmaEllipse2D( ye_mu, ye_sigma, 3, 100 );
p2 = plot(xy(1,:),xy(2,:), '--', 'Color', [.5 0 .5], 'LineWidth',3,'DisplayName','Approximated 3-sigma ellipse');






