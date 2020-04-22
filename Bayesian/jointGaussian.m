function [mu, Sigma] = jointGaussian(mu_x, sigma2_x, sigma2_r)
%jointGaussian calculates the joint Gaussian density as defined
%in problem 1.3a. 
%
%Input
%   MU_X        Expected value of x
%   SIGMA2_X    Covariance of x
%   SIGMA2_R    Covariance of the noise r
%
%Output
%   MU          Mean of joint density 
%   SIGMA       Covariance of joint density


%Your code here
A = [1,0;1,1];
b = [0,0]';

%defining the mean of [x;r]
MU_X = [mu_x,0]';

%SIGMA_X = diag([sigma2_x;sigma2_r]);

SIGMA_X = blkdiag(sigma2_x,sigma2_r);
[mu,Sigma] = affineGaussianTransform(MU_X, SIGMA_X, A, b)

end