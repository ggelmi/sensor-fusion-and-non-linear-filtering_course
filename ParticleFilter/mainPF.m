clear;close all;clc;

T = 0.01;

% Motion Parameters

A = 1;
f = @(x) A*x;
Q = 1.5;
% Measurement Parameters
H = 1;
h = @(x) H*x;
R = 2.5;
% Prior
x_0 = 2;
P_0 = 6;

% number of time steps
N = 50;
% number of particles
Np = 100;
% resample particles?
% bResample = true;
% sigma for the approximation in plotPostPdf
sigma = 1;
% calculate state and measurement sequences
X = genLinearStateSequence(x_0,P_0,A,Q,N);
Y = genLinearMeasurementSequence(X, H, R);

% filter data
[xf, Pf] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);

bResample = true;
plotFunc_handle_r  = @(k, Xk, Xkmin1, Wk, j) plotPostPdf(k, Xk, Wk, xf, Pf, bResample, sigma, []);
[xfpr, Pfpr, Xpr, Wpr] = pfFilter(x_0, P_0, Y, f, Q, h, R, Np, bResample,[] );

bResample = false;
plotFunc_handle_r  = @(k, Xk, Xkmin1, Wk, j) plotPostPdf(k, Xk, Wk, xf, Pf, bResample, sigma, []);
[xfpr_f, Pfpr_f, Xpr_f, Wpr_f] = pfFilter(x_0, P_0, Y, f, Q, h, R, Np, bResample, []);


mse(X(2:end)-xf)
mse(X(2:end)-xfpr)
mse(X(2:end)-xfpr_f)

figure('Color','white','Position',[199   288  1152   457]);
hold on, grid on;

p1 = plot(0:N, H*X, 'b', 'LineWidth',4, 'DisplayName','true state');
p1.Color = [p1.Color 0.2];

p2 = plot(0:N, H*[x_0 xf],  'Color','k', 'LineWidth',2, 'DisplayName','KF estimate');

p_pfr = plot(0:N, H*[x_0 xfpr], 'Color', 'blue', 'LineWidth',2, 'DisplayName','PF estimate with resampling');

p_pfr_f  = plot(0:N, H*[x_0 xfpr_f],  'Color','green', 'LineWidth',2, 'DisplayName','PF estimate without resampling');

p4 = plot(1:N, Y, '*r', 'DisplayName','measurements');

xlabel('k - time step');
ylabel('position');
legend([p1 p2 p_pfr p_pfr_f p4],'Location','southwest');
title('Comparison KF and PF')


% Visualizing a the posterior of one state estimate at  time k for the
% three scenarios along with the true estimate of the state at that point

ts = 45; %19

fig = figure('Color','white','Position',[675  549  570  420]);
hold on, grid on;
set(fig, 'Name', ['p_',num2str(ts), '_', 'SIR']);

[X_lins, KF_pdf, pApprox_r] = plotFunc_handle_r (ts, Xpr(:,:,ts), Xpr(:,:,ts-1), Wpr(:,ts)', []);

plot( X_lins, pApprox_r, 'Color','red', 'LineWidth', 2, 'DisplayName','PF estimate with resampling')

[X_lins, KF, pApprox]   = plotFunc_handle_r(ts, Xpr_f(:,:,ts),  Xpr_f(:,:,ts-1),  Wpr_f(:,ts)',  []);
plot( X_lins, pApprox,   'Color','blue', 'LineWidth', 2, 'DisplayName','PF estimate without resampling')

plot( X_lins, KF, '-.',  'Color','black',     'LineWidth', 2, 'DisplayName','Kalman filter')
legend( 'Location', 'southeast')
title(['p(x_k |  y_{1:k}),   k=', num2str(ts)])
ylim([0,max([pApprox,pApprox_r])*1.2])
xlim( X(ts) + Pf(1,1,ts)^0.5*[-5 5])
p1 = plot( X([ts,ts]+1), [0,max([pApprox,pApprox_r])*1.2],'--' , 'Color','b','LineWidth', 4, 'DisplayName', 'True state')
p1.Color = [p1.Color 0.2];



% Plotting the trajectories of the particles for both scenarios( with resampling and without it) along with the true state
% trajectory. 

close all;

Np = 50;

bAlpha = true;

ploth = @(k, Xk, Xkmin1, Wk, j) plotPartTrajs(k, Xk, Xkmin1, Wk, j, bAlpha);

figure('Color','white','Position',[292   475  1165   422]);

subplot(1,2,1)
hold on, grid on;
bResample = true;
pfFilter(x_0, P_0, Y, f, Q, h, R, Np, bResample, ploth);
plot(0:length(X)-1, X, 'Color','red','LineWidth', 2)
xlabel 'time step', ylabel 'state value', title 'Particle trajectories with resampling'
ylim([-8,12])

subplot(1,2,2)
hold on, grid on;
bResample = false;
pfFilter(x_0, P_0, Y, f, Q, h, R, Np, bResample, ploth);
plot(0:length(X)-1, X, 'Color','red','LineWidth', 2)
xlabel 'time step', ylabel 'state value', title 'Particle trajectories without resampling'
ylim([-32,24])


