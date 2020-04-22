function [fx, Fx] = coordinatedTurnMotion(x, T)
%COORDINATEDTURNMOTION calculates the predicted state using a coordinated
%turn motion model, and also calculated the motion model Jacobian
%
%Input:
%   x           [5 x 1] state vector
%   T           [1 x 1] Sampling time
%
%Output:
%   fx          [5 x 1] motion model evaluated at state x
%   Fx          [5 x 5] motion model Jacobian evaluated at state x
%
% NOTE: the motion model assumes that the state vector x consist of the
% following states:
%   px          X-position
%   py          Y-position
%   v           velocity
%   phi         heading
%   omega       turn-rate

% Your code here
  xk_1 = x(1);
  yk_1 = x(2);
  vk_1 = x(3);
  thetak_1 = x(4);
  wk_1 = x(5);
  
  fx = [ xk_1+ T*vk_1*cos(thetak_1) ; 
         yk_1+ T*vk_1*sin(thetak_1);
         vk_1;
         thetak_1 + T*wk_1;
         wk_1];
   if nargout > 1
        
        Fx = [1 0 T*cos(thetak_1) -T*vk_1*sin(thetak_1) 0;
              0 1 T*sin(thetak_1)  T*vk_1*cos(thetak_1) 0;
              0 0 1                    0                0;
              0 0 0                    1                T;
              0 0 0                    0                1];
    end
end