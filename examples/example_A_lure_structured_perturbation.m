%% EXAMPLE A -- Positive Lur'e System with Structured Perturbations
% Reproduces Figures 2 and 3 of:
%
%   H. Montazeri Hedesh, M. K. Wafi, B. Shafai, and M. Siami,
%   "Robust Stability Analysis of Positive Lur'e System with Neural
%    Network Feedback," 2025 IEEE Conference on Control Technology and
%    Applications (CCTA), San Diego, CA, USA, pp. 976-981.
%   DOI: 10.1109/CCTA53793.2025.11151493
%
% ------------------------------------------------------------------------
% WHAT THIS SCRIPT DOES
%   1. Defines a 3rd-order, open-loop-unstable LTI plant closed with a
%      static, sector-bounded nonlinearity (a positive Lur'e system).
%   2. Computes the real stability radius r_R(Phi) using Theorem 3:
%
%            r_R(Phi) = 1 / || E (A + B*Sigma2*C)^{-1} D ||_2
%
%   3. Figure 2: draws the nonlinearity together with its sector lines
%      Sigma1*y and Sigma2*y.
%   4. Figure 3: sweeps the structured perturbation Delta and shows that
%      the closed loop remains exponentially stable up to
%      Delta = r_R(Phi) ~ 0.26, then loses stability.
%
% REQUIREMENTS: base MATLAB only (uses ode45). No toolboxes required.
% ------------------------------------------------------------------------

clc; clear; close all;
rng(0, 'twister');            % fixed seed -> reproducible random trajectories

%% ------------------------------ System data -----------------------------
A = [-5  5  1;
      6 -7  1;
      2  1 -5];
B = [1; 1; 1];
C = [1 0.1 0.1];              % system output matrix
D = [1; 0.5; 1];
E = [0.5 1 1];

% Sector [Sigma1, Sigma2] in which the nonlinearity is confined.
% (The paper rounds the upper bound to -0.48.)
Sigma1 = -2;
Sigma2 = -0.4844;

%% -------------------- Stability radius (Theorem 3) ----------------------
r = 1 / norm(E*((A + B*Sigma2*C)\D), 2);
fprintf('Stability radius  r_R(Phi) = %.4f   (paper: 0.26)\n', r);

% Structural conditions required by Theorem 3.
assert(isMetzler(A + B*Sigma1*C),         'A + B*Sigma1*C must be Metzler.');
assert(all(real(eig(A + B*Sigma2*C)) < 0), 'A + B*Sigma2*C must be Hurwitz.');
fprintf('Conditions OK: A+B*Sigma1*C Metzler, A+B*Sigma2*C Hurwitz.\n');

%% --------------- Figure 2: nonlinearity inside its sector ---------------
y   = linspace(-10, 10, 400);
phi = arrayfun(@(v) sectorNonlinearity(v, Sigma1, Sigma2), y);

figure('Name', 'Fig. 2 -- sector bound');
plot(y, Sigma1*y, 'b', y, Sigma2*y, 'r', y, phi, 'k', 'LineWidth', 2.5);
grid on; set(gca, 'FontSize', 16);
xlabel('y'); ylabel('f(y)');
legend('\Sigma_1 y', '\Sigma_2 y', '\Phi(y)', 'Location', 'best');
title('Static nonlinearity within the sector [\Sigma_1, \Sigma_2]');

%% ------------- Figure 3: trajectory sweep over Delta --------------------
Delta_values = [0, 0.25, 0.26, 0.30];      % r_R(Phi) ~ 0.26
tspan        = [0 100];
n_traj       = 50;

figure('Name', 'Fig. 3 -- output trajectories');
for k = 1:numel(Delta_values)
    Delta = Delta_values(k);
    Ap    = A + D*Delta*E;                 % perturbed system matrix
    subplot(2, 2, k); hold on;
    for j = 1:n_traj
        x0     = 25*abs(rand(size(A,1), 1));
        odefun = @(t, x) Ap*x + B*sectorNonlinearity(C*x, Sigma1, Sigma2);
        [t, x] = ode45(odefun, tspan, x0);
        plot(t, C*x', 'b', 'LineWidth', 1.5);
    end
    grid on; set(gca, 'FontSize', 12);
    xlabel('Time'); ylabel('System Output');
    title(sprintf('\\Delta = %g', Delta));
    hold off;
end

%% ============================ local functions ===========================
function y = sectorNonlinearity(x, Sigma1, Sigma2)
% Nonlinearity  f(x) = -1.5x + 0.01 x^3 + sin(2x), clamped so that it stays
% inside the sector, i.e.  Sigma1 <= f(x)/x <= Sigma2  for every x.
    f  = -1.5*x + 0.01*x.^3 + sin(2*x);
    lo = min(Sigma1*x, Sigma2*x);          % lower sector line (any sign of x)
    hi = max(Sigma1*x, Sigma2*x);          % upper sector line
    y  = min(max(f, lo), hi);              % clamp f into [lo, hi]
end

function tf = isMetzler(M)
% True if all off-diagonal entries of M are nonnegative.
    Off = M - diag(diag(M));
    tf  = all(Off(:) >= -1e-12);
end
