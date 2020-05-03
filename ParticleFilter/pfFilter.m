function [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, f, Q, h, R, ...
                             N, bResample, plotFunc)
%PFFILTER Filters measurements Y using the SIS or SIR algorithms and a
% state-space model.
%
% Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   Y           [m x K] Measurement sequence to be filtered
%   f           Handle for process function f(x_k-1)
%   Q           [n x n] process noise covariance
%   h           Handle for measurement model function h(x_k)
%   R           [m x m] measurement noise covariance
%   N           Number of particles
%   bResample   boolean false - no resampling, true - resampling
%   plotFunc    Handle for plot function that is called when a filter
%               recursion has finished.
% Output:
%   xfp         [n x K] Posterior means of particle filter
%   Pfp         [n x n x K] Posterior error covariances of particle filter
%   Xp          [n x N x K] Particles for posterior state distribution in times 1:K
%   Wp          [N x K] Non-resampled weights for posterior state x in times 1:K

% Your code here, please. 
% If you want to be a bit fancy, then only store and output the particles if the function
% is called with more than 2 output arguments.

 % The dimension of the state
 n = size(x_0,1);
 % The number of measurement data points
 K = size(Y,2);
 % place holder
  xfp = zeros(n,K);
  Pfp = zeros(n,n,K);
  Xp = zeros(n,N,K);
  Wp = zeros(N,K);

  % sample initial particles around prior distribution
    % if x_0 has only one column, x_0 is the mean of the prior
    % otherwise x_0 are the initial particles states
    
  % This is basically telling us if the initial samples has been given to
  % us and if not we have to draw random samples from the initial state
  % given
    if size(x_0,2) == 1
        Xp(:,:,1) = mvnrnd(x_0,P_0,N)';
    else
        Xp(:,:,1) = x_0;
    end
    
    Wp(:,1)  = 1/N * ones(1,N);
    
    j = 1:N;
    
  for i=2:K+1
      Xp_km_1 = Xp(:,:,i-1);
      Wp_km_1 = Wp(:,i-1)';
      
      % resample
        if bResample
            [Xp_km_1, Wp_km_1, j] = resampl(Xp_km_1, Wp_km_1);        
        end
        
        % perform a particle filter step for the next measurement
        [Xp(:,:,i), Wp(:,i)] = pfFilterStep( Xp_km_1, Wp_km_1, Y(:,i-1), f, Q, h, R);
        
        % plot particles using function handle
        if ~isempty(plotFunc)
            plotFunc(i-1, Xp(:,:,i), Xp(:,:,i-1), Wp(:,i)', j);
        end
        
        % estimate mean and covariance given the particles
        xfp(:,i)   = sum( Xp(:,:,i).*Wp(:,i)' ,2 );
        Pfp(:,:,i) = Wp(:,i)'.*(Xp(:,:,i) - xfp(:,i))*(Xp(:,:,i) - xfp(:,i))';
      
  end
  
  % removing the priors 
    xfp = xfp(:,2:end);
    Pfp = Pfp(:,:,2:end);
    Xp  = Xp(:,:,2:end);
    Wp  = Wp(:,2:end);

end
