function [hx, Hx] = dualBearingMeasurement(x, s1, s2)
%DUOBEARINGMEASUREMENT calculates the bearings from two sensors, located in 
%s1 and s2, to the position given by the state vector x. Also returns the
%Jacobian of the model at x.
%
%Input:
%   x           [n x 1] State vector, the two first element are 2D position
%   s1          [2 x 1] Sensor position (2D) for sensor 1
%   s2          [2 x 1] Sensor position (2D) for sensor 2
%
%Output:
%   hx          [2 x 1] measurement vector
%   Hx          [2 x n] measurement model Jacobian
%
% NOTE: the measurement model assumes that in the state vector x, the first
% two states are X-position and Y-position.

[r,c] = size(x);
% This is intended to represent the state vector as a n by 1 rather than 1
% by n
if(r < c && c ==2)
    x = x';
end
% Your code here
n = size(x,1);  
N = size(x,2);
hx = zeros(2,N);
Hx = zeros(2,n);

angle_1_input = [x(2,:)-s1(2) x(1,:)-s1(1)];
angle_2_input = [x(2,:)-s2(2) x(1,:)-s2(1)];

if isreal(angle_1_input)
    angle1 =  atan2(x(2,:)-s1(2),x(1,:)-s1(1));
    angle2 =  atan2(x(2,:)-s2(2),x(1,:)-s2(1));
    
else
    
    angle1 =  atan2(real(x(2,:)-s1(2)),real(x(1,:)-s1(1)));
    angle2 =  atan2(real(x(2,:)-s2(2)),real(x(1,:)-s2(1)));
end


hx(1:2,:) = [angle1; angle2];

Hx(1:2,1:2) = [-(x(2)-s1(2)) / ( (x(1)-s1(1))^2 + (x(2)-s1(2))^2 ), (x(1)-s1(1)) / ((x(1)-s1(1))^2 + (x(2)-s1(2))^2);
                   -(x(2)-s2(2)) / ( (x(1)-s2(1))^2 + (x(2)-s2(2))^2 ), (x(1)-s2(1)) / ((x(1)-s2(1))^2 + (x(2)-s2(2))^2)];






end