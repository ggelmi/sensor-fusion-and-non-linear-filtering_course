function [xp, P] = nonLinKFprediction(x, P, f, Q, type)
%NONLINKFPREDICTION calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   f           Motion model function handle
%               [fx,Fx]=f(x) 
%               Takes as input x (state), 
%               Returns fx and Fx, motion model and Jacobian evaluated at x
%               All other model parameters, such as sample time T,
%               must be included in the function
%   Q           [n x n] Process noise covariance
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] predicted state mean
%   P           [n x n] predicted state covariance
%
n = max(size(x));
    switch type
        case 'EKF'
            % Your EKF code here
            [fx, dfx] = f(x);
            % predict using first order Taylor expansion
        	xp = fx;
            P = dfx * P * dfx' + Q;
            
        case 'UKF'
             num = 2*n+1;
            % Your UKF code here
             [SP,W] = sigmaPoints(x,P,'UKF');
             %Getting the transformed sigma points after inputting into the
             %model
              xp = zeros(n,1);
              for i = 1:num
                  xp = xp + f(SP(:,i))*W(i);
              end
              P = zeros(n,n);
              for i = 1:num
                 
                  P = P + ((f(SP(:,i))-xp)*(f(SP(:,i))-xp)')*W(i);
              end
                  P = P + Q;
                  
        case 'CKF'
            
            % Your CKF code here
             num = 2*n;
            % Your UKF code here
             [SP,W] = sigmaPoints(x,P,'CKF');
             %Getting the transformed sigma points after inputting into the
             %model
              xp = zeros(n,1);
              for i = 1:num
                  xp = xp + f(SP(:,i))*W(i);
              end
              P = zeros(n,n);
              for i = 1:num
                 
                  P = P + ((f(SP(:,i))-xp)*(f(SP(:,i))-xp)')*W(i);
              end
                  P = P + Q;
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end
            % Make sure the covariance matrix is semi-definite
        if min(eig(P))<=0
            [v,e] = eig(P, 'vector');
            e(e<0) = 1e-4;
            P = v*diag(e)/v;
        end
end