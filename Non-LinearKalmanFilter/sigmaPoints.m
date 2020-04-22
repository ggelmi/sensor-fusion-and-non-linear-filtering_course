function [SP,W] = sigmaPoints(x, P, type)
% SIGMAPOINTS computes sigma points, either using unscented transform or
% using cubature.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%
%Output:
%   SP          [n x 2n+1] UKF, [n x 2n] CKF. Matrix with sigma points
%   W           [1 x 2n+1] UKF, [1 x 2n] UKF. Vector with sigma point weights 
%

[r,c] = size(x);

if(c > r)
    x = x';
end
n = max(size(x));
    switch type        
        case 'UKF'
    
            % your code
            
            SP = zeros(n, 2*n+1);
            W0 = 1 - n/3;
            P = sqrtm(P);
            offset = sqrt(n/(1-W0));
            SP(:,1) = x;
            for i = 1:n
                SP(:,i+1) = x + offset*P(:,i);
                SP(:,i+n+1) = x - offset*P(:,i);
            end
            W= [W0,(1-W0)/(2*n)*ones(1, 2*n)] ;
        case 'CKF'
            
            % your code
            SP = zeros(n, 2*n)
            offset = sqrt(n);
             P = sqrtm(P);
            for i = 1:n
                SP(:,i) = x + offset*P(:,i);
                SP(:,i+n) = x - offset*P(:,i);
            end    
            W= (1/(2*n)*ones(1, 2*n)) ;
        otherwise
            error('Incorrect type of sigma point')
    end

end