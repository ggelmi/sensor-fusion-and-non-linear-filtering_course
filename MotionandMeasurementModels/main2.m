% Number of states
N = 500;

% Position sequence
X = [(1:N); rand(1,N)];

% Random sensor position sequence
s1 = [0, 100]';
s2 = [0, -100]';

% Measurement noise covariance
R = diag([2*pi/180 2*pi/180].^2);

% Measurement model
measModel = @(X) dualBearingMeasurement(X, s1, s2)

% Generate measurements
Y = genNonLinearMeasurementSequence(X, measModel, R);

% Show some results
disp(Y(:,1:5))