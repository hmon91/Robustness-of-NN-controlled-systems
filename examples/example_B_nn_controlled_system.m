%% EXAMPLE B -- Neural-Network-Controlled Positive Lur'e System
% Reproduces Figures 4, 5 and 6 of:
%
%   H. Montazeri Hedesh, M. K. Wafi, B. Shafai, and M. Siami,
%   "Robust Stability Analysis of Positive Lur'e System with Neural
%    Network Feedback," 2025 IEEE Conference on Control Technology and
%    Applications (CCTA), San Diego, CA, USA, pp. 976-981.
%   DOI: 10.1109/CCTA53793.2025.11151493
%
% ------------------------------------------------------------------------
% PIPELINE
%   * A 3rd-order LTI plant is controlled by a feed-forward neural network
%     (architecture 1-5-10-1, tanh activations) trained to imitate a gain
%     feedback controller. The weights are read from ../data/W{1,2,3}.csv.
%
%   * Theorem 4 gives an a-priori sector [Gamma1, Gamma2] for the NN
%     (c = 1 for tanh, which is sector bounded in [0,1]):
%
%            Gamma2 = |W3| * |W2| * |W1|,     Gamma1 = -Gamma2
%
%   * Theorem 5 gives the stability radius of the NN-controlled system:
%
%            r = 1 / || E (A + B*Gamma2*C)^{-1} D ||_2        (~ 2.04)
%
%   * Corollary 1 refines the (conservative) upper sector bound using the
%     critical destabilising perturbation Delta:
%
%            Gamma2_refined(Delta) = 1 / || C (A + D*Delta*E)^{-1} B ||_2
%
% FIGURES
%   Fig. 4 : output trajectories around r ~ 2.04    (Delta = 1.94 .. 3.19)
%   Fig. 5 : output trajectories through the true   (Delta = 2.5  .. 3.25)
%            instability threshold (unstable at 3.15)
%   Fig. 6 : NN output vs. the refined sector for random positive inputs
%
% REQUIREMENTS: base MATLAB only (uses ode45). No toolboxes required.
% ------------------------------------------------------------------------

clc; clear; close all;
rng(0, 'twister');

%% -------------------- Load NN weights (1 -> 5 -> 10 -> 1) ---------------
here    = fileparts(mfilename('fullpath'));
dataDir = fullfile(here, '..', 'data');
W1 = load(fullfile(dataDir, 'W1.csv'));    %  5 x 1
W2 = load(fullfile(dataDir, 'W2.csv'));    % 10 x 5
W3 = load(fullfile(dataDir, 'W3.csv'));    %  1 x 10

%% ------------------------------ System data -----------------------------
A = [-5  2  1;
      3 -7  2;
      2  1 -6];
B = [0.5; 1; 0.4];
C = [0.3 1 1];
D = [1; 0; 0];
E = [1 0 0];

%% ------- Sector bounds (Theorem 4) and stability radius (Theorem 5) -----
Gamma2 = abs(W3)*abs(W2)*abs(W1);          % scalar, ~ 0.91
Gamma1 = -Gamma2;

r = 1 / norm(E*((A + B*Gamma2*C)\D), 2);

% The "real" slope of this SISO NN at the boundary input z = 1. Because the
% a-priori Gamma2 (Theorem 4) is conservative, the true radius r_real is
% larger than r -- this gap is exactly what Corollary 1 exploits.
NN1    = nnEval(W1, W2, W3, 1);
r_real = 1 / norm(E*((A + B*NN1*C)\D), 2);

fprintf('Theorem 4 sector : [%.4f, %.4f]   (paper: [-0.91, 0.91])\n', Gamma1, Gamma2);
fprintf('Theorem 5 radius : r      = %.4f   (paper: 2.04)\n', r);
fprintf('NN slope at z=1  : NN(1)  = %.4f\n', NN1);
fprintf('Refined radius   : r_real = %.4f\n', r_real);

assert(isMetzler(A + B*Gamma1*C),          'A + B*Gamma1*C must be Metzler.');
assert(all(real(eig(A + B*Gamma2*C)) < 0), 'A + B*Gamma2*C must be Hurwitz.');

%% ------------- Figure 4: trajectories around the radius r ---------------
% Delta values ~ [r-0.1, r, r+0.2, r_real+0.1] -> [1.94, 2.04, 2.24, 3.19]
Delta_fig4 = [1.94, 2.04, 2.24, 3.19];
plotTrajectories(A, B, C, D, E, W1, W2, W3, Delta_fig4, [0 20], 'Fig. 4');

%% -------- Figure 5: trajectories through the instability threshold ------
Delta_fig5 = [2.5, 3, 3.15, 3.25];         % becomes unstable at Delta = 3.15
plotTrajectories(A, B, C, D, E, W1, W2, W3, Delta_fig5, [0 15], 'Fig. 5');

%% ---------------- Figure 6: refined sector (Corollary 1) ----------------
figure('Name', 'Fig. 6 -- refined sector');
tiledlayout(2, 2);
nSamples = 100;
for k = 1:numel(Delta_fig5)
    Delta   = Delta_fig5(k);
    Gamma2r = 1 / norm(C*((A + D*Delta*E)\B), 2);   % refined upper bound

    z   = abs(rand(1, nSamples));                   % random positive inputs
    yNN = arrayfun(@(v) nnEval(W1, W2, W3, v), z);
    lo  = Gamma1  * z;
    hi  = Gamma2r * z;

    nexttile; hold on;
    fillBetween(1:nSamples, lo, hi, [0.8 0.8 0.8]);
    plot(1:nSamples, yNN, 'k',                          'LineWidth', 2);
    plot(1:nSamples, lo,  'Color', [0.000 0.447 0.741], 'LineWidth', 2);
    plot(1:nSamples, hi,  'Color', [0.635 0.078 0.184], 'LineWidth', 2);
    grid on;
    xlabel('Random Positive Input'); ylabel('Output');
    title(sprintf('\\Delta = %g  \\rightarrow  \\Gamma_{2,refined} = %.2f', ...
                  Delta, Gamma2r));
    hold off;
end
lg = legend({'', 'NN(z)', '\Gamma_1 z', '\Gamma_{2,refined} z'});
lg.Layout.Tile = 'east';

%% ============================ local functions ===========================
function plotTrajectories(A, B, C, D, E, W1, W2, W3, Deltas, tspan, figname)
% Simulate the NN-controlled closed loop from random nonnegative initial
% conditions for each perturbation level in Deltas, on a 2x2 grid.
    figure('Name', figname);
    n_traj = 100;
    for k = 1:numel(Deltas)
        Delta = Deltas(k);
        Ap    = A + D*Delta*E;
        subplot(2, 2, k); hold on;
        for j = 1:n_traj
            x0     = 10*abs(rand(size(A,1), 1));
            odefun = @(t, x) Ap*x + B*nnEval(W1, W2, W3, C*x);
            [t, x] = ode45(odefun, tspan, x0);
            plot(t, C*x', 'b', 'LineWidth', 1.5);
        end
        grid on; set(gca, 'FontSize', 12);
        xlabel('Time'); ylabel('System Output');
        title(sprintf('\\Delta = %g', Delta));
        hold off;
    end
end

function u = nnEval(W1, W2, W3, z)
% Forward pass of the feed-forward NN with tanh activations.
    u = W3 * tanh(W2 * tanh(W1 * z));
end

function fillBetween(x, lo, hi, color)
% Shade the region between the two sector lines.
    fill([x, fliplr(x)], [lo, fliplr(hi)], color, ...
         'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

function tf = isMetzler(M)
% True if all off-diagonal entries of M are nonnegative.
    Off = M - diag(diag(M));
    tf  = all(Off(:) >= -1e-12);
end
