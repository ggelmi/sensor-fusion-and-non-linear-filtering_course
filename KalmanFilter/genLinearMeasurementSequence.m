function Y = genLinearMeasurementSequence(X, H, R)
%GENLINEARMEASUREMENTSEQUENCE generates a sequence of observations of the state 
% sequence X using a linear measurement model. Measurement noise is assumed to be 
% zero mean and Gaussian.
%
%Input:
%   X           [n x N+1] State vector sequence. The k:th state vector is X(:,k+1)
%   H           [m x n] Measurement matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   Y           [m x N] Measurement sequence
%

% your code here
m = size(H,1);
n = length(X(:,1)); 
N = length(X(1,:));
Y = zeros(m,N-1);
for i = 2:N
    Y(:,i-1) = H*X(:,i)+ mvnrnd(zeros(m,1),R)';
end
end