function [Xr, Wr, j] = resampl(X, W)
%RESAMPLE Resample particles and output new particles and weights.
% resampled particles. 
%
%   if old particle vector is x, new particles x_new is computed as x(:,j)
%
% Input:
%   X   [n x N] Particles, each column is a particle.
%   W   [1 x N] Weights, corresponding to the samples
%
% Output:
%   Xr  [n x N] Resampled particles, each corresponding to some particle 
%               from old weights.
%   Wr  [1 x N] New weights for the resampled particles.
%   j   [1 x N] vector of indices refering to vector of old particles

% Your code here!
% Number of samples
 N=size(X,2);
 % Generates the segmented numberline from 0 to 1
 segment = [0 cumsum(W)/sum(W)];
 % drawing samples from uniform distribution on [0,1]
    samples = rand([1 N]);
 % indices place holder
   j=zeros(1,N);
   for i=1:N
        j(i) = find(samples(i) >= segment,1,'last');
   end
    % Resampled weights
    Wr = 1/N*ones(1,N);
    % Resamples particles corresponding to some particle from old weights
    Xr = X(:,j);
end