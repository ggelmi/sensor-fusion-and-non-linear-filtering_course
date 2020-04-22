clear all;
clc;
%close all;
%{
% The first assesment
%Here we give parameters for a Gaussian density. The parameter mu is the mean, and P is the covariance matrix.
mu = [-2; 1];
P = [4, 2; 2 8];

%Call your function.
xy = sigmaEllipse2D(mu, P,2);

%Now plot the generated points. You should see an elongated ellipse stretching from the top left corner to the bottom right. 
figure(1);
h1 = plot(xy(1,:), xy(2,:));
%Set the scale of x and y axis to be the same. This should be done if the two variables are in the same domain, e.g. both are measured in meters.
axis equal
hold on
%Also plot a star where the mean is, and make it have the same color as the ellipse.
plot(mu(1), mu(2), '*', 'color', h1.Color);

% The second assessment

mu_x= [-2,1]';
sigma_x = [4, -2; -2 8];

A = [1,2;2,1];
b = [1,0]';

affineGaussianTransform(mu_x, sigma_x, A, b)

%} 
% test with a 2D Gaussian distribution

mu_x = [rand*2*pi; 5];
Sigma_x = diag([0.2; 0.5]);

f = @f_pol2cart;

[mu_y, Sigma_y, ys,xs] = approxGaussianTransform(mu_x, Sigma_x, f);

mu_x 
mu_y
Sigma_y


function xs = f_pol2cart(ys)
    [x,y] = pol2cart(ys(:,1), ys(:,1));
    xs(:,1) = x;
    xs(:,2) = y;
end
%{
mu_x = [7; 3]
Sigma_x = diag([0.2; 8])

f = @f_atan2;

[mu_y, Sigma_y, xs] = approxGaussianTransform(mu_x, Sigma_x, f);

mu_y
Sigma_y
x1 = 7; x2 = 3;
up = abs(sqrt(x1^2+x2^2))
utheta = atan2(x2,x1)

mu_x = 19;
sigma2_x = 5^2;
sigma2_r = 2^2;

[mu, Sigma] = jointGaussian(mu_x, sigma2_x, sigma2_r)

mu_x = 160;          % Mean of the 1st component
sigma2_x = 3600;  % Covariance of the 1st component
y = 195;        % Mean of the 2nd component
sigma2_r = 1600;  % Covariance of the 2nd component

[mu,sig] = posteriorGaussian(mu_x, sigma2_x, y, sigma2_r)
 sqrt(sig)
%}
