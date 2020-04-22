clear
clc
close all
% Number of data points for measurement generation
N = 10;
% Define prior state and Covariance
x_0     = [0;0]; 
%Dimensions of the state
n       = length(x_0);
P_0     = diag(ones(n,1));
% Define process model
A  = [1 1; 0 1];
%Process covariance
Q  = 2*diag(ones(n,1));
% generate true state sequences. The purpose of this is to use it for
% generating the measurement data
s = rng;
X = genLinearStateSequence(x_0, P_0, A, Q, N)
plot(X(1,2:N+1));
hold on
% Generating the simulated measurements
% Defining Measurement Model
H = [1 0; 0 1];
% dimensions fo the measurements
m = size(H,1);
% Measurment covariance
R = 200*diag(ones(m,1));
% Generating the measurement data 
Y = genLinearMeasurementSequence(X, H, R)
% Plotting the measurements
plot(Y(1,:));
%legend({' True Data','Measurement Data'})

% Applying Kalman filtering
[X, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R)

%Comparing and plotting measurement, true and posterior data
plot(X(1,:));
legend({' True Data','Measurement Data','Posterior Data'})

