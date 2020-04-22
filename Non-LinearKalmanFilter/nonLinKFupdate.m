function [x, P] = nonLinKFupdate(x, P, y, h, R, type)
%NONLINKFUPDATE calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Predicted mean
%   P           [n x n] Predicted covariance
%   y           [m x 1] measurement vector
%   h           Measurement model function handle
%               [hx,Hx]=h(x) 
%               Takes as input x (state), 
%               Returns hx and Hx, measurement model and Jacobian evaluated at x
%               Function must include all model parameters for the particular model, 
%               such as sensor position for some models.
%   R           [m x m] Measurement noise covariance
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] updated state mean
%   P           [n x n] updated state covariance
%
n = max(size(x));
m = max(size(y));
    switch type
        case 'EKF'
            
    % Your EKF update here
            %Innovation
            [hx, Hx]  = h(x);
            v = y - hx;
            % Predicted covariance of the measurement
            S_k =  (Hx*P*Hx') + R;
            %Kalman gain
            K = (P*Hx')/S_k;
            %updating the state
            x = x + K*v;
            %updating the covariance
            P = P - K*S_k*K';
            
        case 'UKF'
    
            % Your UKF update here
             num = 2*n+1;
            % Your UKF code here
             [SP,W] = sigmaPoints(x,P,'UKF');
              yp = zeros(m,1);
              for i = 1:num
                  yp = yp + h(SP(:,i))*W(i);
              end
              Sk = zeros(m,m);
              for i = 1:num
                  Sk= Sk + ((h(SP(:,i))-yp)*(h(SP(:,i))-yp)')*W(i);
              end
                  Sk= Sk + R;
              Pxy = zeros(n,m);
              for i = 1:num
                  Pxy= Pxy + ((SP(:,i)-x)*(h(SP(:,i))-yp)')*W(i);
              end
              
              x = x + (Pxy/Sk*(y-yp));
              P = P - (Pxy /Sk *Pxy');
        case 'CKF'
    
            % Your CKF update here
             % Your UKF update here
             num = 2*n;
            % Your UKF code here
             [SP,W] = sigmaPoints(x,P,'CKF');
              yp = zeros(m,1);
              for i = 1:num
                  yp = yp + h(SP(:,i))*W(i);
              end
              Sk = zeros(m,m);
              for i = 1:num
                  Sk= Sk + ((h(SP(:,i))-yp)*(h(SP(:,i))-yp)')*W(i);
              end
                  Sk= Sk + R;
              Pxy = zeros(n,m);
              for i = 1:num
                  Pxy= Pxy + ((SP(:,i)-x)*(h(SP(:,i))-yp)')*W(i);
              end
              
              x = x + (Pxy/Sk*(y-yp));
              P = P - (Pxy /Sk *Pxy');
             
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end