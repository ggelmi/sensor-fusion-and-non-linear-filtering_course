function [X, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R)
%KALMANFILTER Filters measurements sequence Y using a Kalman filter. 
%
%Input:
%   Y           [m x N] Measurement sequence
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   A           [n x n] State transition matrix
%   Q           [n x n] Process noise covariance
%   H           [m x n] Measurement model matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   x           [n x N] Estimated state vector sequence
%   P           [n x n x N] Filter error convariance
%

%% Parameters
N = size(Y,2);

n = length(x_0);
m = size(Y,1);

%% Data allocation
X = zeros(n,N);
P = zeros(n,n,N);
X(:,1) = x_0;
P(:,:,1) = P_0;

for i = 1:N
    %Predicting the next state
   [X(:,i+1), P(:,:,i+1)] = linearPrediction(X(:,i), P(:,:,i), A, Q)  
    %Updating The predicted state with the measurement
   [X(:,i+1), P(:,:,i+1)] = linearUpdate(X(:,i+1), P(:,:,i+1), Y(:,i), H, R)
    
end

X = X(:,2:end);
P = P(:,:,2:end);
end

