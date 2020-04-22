clc;
close all
clear all;

% Choosing the case
case_i = 1;

% Number of points
N = 100;
% prior state and covariance

X_0 = [0 0 14 0 0]';
P_0 = diag([10 10 2 pi/180 5*pi/180].^2);
% Sampling time
T = 1;
% Sensor positions
s1 = [-200 100]';
s2 = [-200 -100]';

% Process and Measurement Noise Covariances
sigma_V = 1;
sigma_W = pi/180;

if case_i==1
    sigma_Phi_1 = 10*pi/180;    
else
    sigma_Phi_1= 0.5*pi/180;
end
sigma_Phi_2 = 0.5*pi/180;

Q = diag([0 0 T*sigma_V^2 0 T*sigma_W^2]);
R = diag([sigma_Phi_1^2 sigma_Phi_2^2]);

% The first task is to generate the "Assumed" true state sequences

% Motion model function handle. Note how sample time T is inserted into the function.
motionModel = @(x) coordinatedTurnMotion(x, T);

% generate state sequence
X = genNonLinearStateSequence(X_0, P_0, motionModel, Q, N);

% Generating the simulated measurements 

% This is the measurement model
measModel = @(X) dualBearingMeasurement(X, s1, s2)

% Generate measurements sequences
Y = genNonLinearMeasurementSequence(X, measModel, R);

% calcualte unfiltered position from sensors given angles
Xm(1,:) = ( s2(2)-s1(2) + tan(Y(1,:))*s1(1) - tan(Y(2,:))*s2(1) ) ./ ( tan(Y(1,:)) - tan(Y(2,:)) );
Xm(2,:) = s1(2) + tan(Y(1,:)) .* ( Xm(1,:) - s1(1) );


for filter_type = {'EKF','UKF','CKF'}
    
    % filter
    [xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, X_0, P_0, motionModel, Q, measModel, R, filter_type{1});

    figure('Color','white','Position',[500  300  603  429]);
    grid on; hold on %, axis equal
    
    for i=1:5:length(xf)
        ell_xy = sigmaEllipse2D(xf(1:2,i),Pf(1:2,1:2,i),3,50);
    %     fill(ell_xy(1,:),ell_xy(2,:), '--', 'Color',cp(5,:), 'DisplayName','3-sigma level');
        p4 = fill(ell_xy(1,:),ell_xy(2,:), [0, 0.4470, 0.7410],'facealpha',.2, 'DisplayName','3-sigma level');   %, 'edgecolor','none'
    end
    
    p1 = plot(X(1,:),X(2,:), 'Color', 'blue', 'LineWidth',2, 'DisplayName','True position');
    
    p2 = plot(xf(1,:),xf(2,:), 'Color', 'red', 'LineWidth',2, 'DisplayName','estimated position');
    
    sc1 = scatter(s1(1), s1(2), 100, 'o', 'MarkerFaceAlpha',0.8, 'MarkerFaceColor', [.5 0 .5], 'MarkerEdgeColor', [.5 0 .5],'DisplayName','sensor 1 location');
    sc2 = scatter(s2(1), s2(2), 200, 'h', 'MarkerFaceAlpha',0.8, 'MarkerFaceColor', [.5 0 .5], 'MarkerEdgeColor', [.5 0 .5],'DisplayName','sensor 2 location');

    axis manual
    p3 = plot(Xm(1,:),Xm(2,:), 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',1, 'DisplayName','Measured position');
    
    xlabel 'pos x', ylabel 'pos y'
    title(sprintf('Case %d, filter type: %s',case_i,filter_type{1}))
    legend([p1 p2 p3 p4 sc1 sc2], 'Location','southwest')

    
end

% Analyzing The filter Characteristics using montecarlo simulation
monte_carlo_Num = 10;
filter_type = {'EKF','UKF','CKF'};

error_estimates = cell(2,3);

for i_mt_c = 1:monte_carlo_Num
    for case_i = 1:2
        
        sigma_V = 1;
        sigma_W = pi/180;

        if case_i==1
            sigma_Phi_1 = 10*pi/180;    
        else
            sigma_Phi_1= 0.5*pi/180;
        end
        sigma_Phi_2 = 0.5*pi/180;

        Q = diag([0 0 T*sigma_V^2 0 T*sigma_W^2]);
        R = diag([sigma_Phi_1^2 sigma_Phi_2^2]);

        
        % Simulate state sequence
        X = genNonLinearStateSequence(X_0, P_0, motionModel, Q, N);
        % Simulate measurements
        Y = genNonLinearMeasurementSequence(X, measModel, R);
        
        for type_i = 1:numel(filter_type)
            % Run Kalman filter (you need to run all three, for comparison)
            [xf,Pf,xp,Pp] = nonLinearKalmanFilter(Y,X_0,P_0,motionModel,Q,measModel,R,filter_type{type_i});
            % Save the estimation errors and the prediction errors
            error_estimates{case_i,type_i}(1:2,end+1:end+length(xf)) = X(1:2,2:end) - xf(1:2,:);
        end
    end
    i_mt_c
end


% Plotting histograms of error estimates

close all;

bins = 100;
close all;
pos = {'x','y'};
for case_i = 1:2
    figure('Color','white','Position',[381   314  1012   537]);
    sgtitle(sprintf('Normalized histogram of the position error, case: %d',case_i))
    for type_i = 1:numel(filter_type)
        for pos_i = 1:numel(pos)
            subplot(2,3, type_i + (pos_i-1)*numel(filter_type) );
            hold on;
            
            data_i = error_estimates{case_i,type_i}(pos_i,:);
            mu  = mean(data_i);
            stddev = std(data_i);
            
            % remove outliers
            idx = abs(data_i-mu) < stddev*3;
            data_i = data_i(idx);
            
            histogram( data_i, bins ,'Normalization','pdf','DisplayName','histogram MSE of position error norm');
            
            [x,y] = normpdf2(mu, stddev^2, 3, 100);
            plot(x,y, 'LineWidth',2, 'DisplayName', sprintf('gaussian N(x; 0, $P_{N|N})$') );
            
            xlims = max(abs(data_i));
            xlim([-xlims xlims]);
            
            xlabel(sprintf('pos-%s error',pos{pos_i}))
            title(sprintf('filter type: %s, pos-%s \n mean: %.2f, std.dev: %.1f',filter_type{type_i},pos{pos_i},mu,stddev ))    
        end
    end
    
end


