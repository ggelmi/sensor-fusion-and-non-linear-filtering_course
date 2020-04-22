
% Number of time steps;
N = 500;

% Define prior
x_0     = rand(5,1); 
n       = length(x_0); 
P_0     = rand(5,5);
P_0     = P_0*P_0';

% Sample time
T = 1;

% Covariance
sigV = 1;
sigOmega = 1*pi/180;
G = [zeros(2,2); 1 0; 0 0; 0 1];
Q = G*diag([sigV^2 sigOmega^2])*G';

% Motion model function handle. Note how sample time T is inserted into the function.
motionModel = @(x) coordinatedTurnMotion(x, T);

% generate state sequence
X = genNonLinearStateSequence(x_0, P_0, motionModel, Q, N);

% show some results
disp(X(:,1:5));