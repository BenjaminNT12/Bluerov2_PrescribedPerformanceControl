function [ITAE_formation, ITAE_tracking, ControlEnergy] = calculateAdditionalMetrics(t, edgeError, V, vds, u, T, m, n)
% calculateAdditionalMetrics computes:
%   1) ITAE for formation (based on edgeError)
%   2) ITAE for tracking (based on velocity or position tracking error)
%   3) Total control energy
%
% Inputs:
%   t         - Time vector [1 x num_steps]
%   edgeError - Matrix of formation distance errors [num_edges x num_steps]
%   V         - Velocity matrix [m*n x num_steps]
%   vds       - Desired velocity matrix [3 x num_steps]
%   u         - Control input matrix [m*n x num_steps]
%   T         - Sampling time
%   m         - Spatial dimension (e.g., 3)
%   n         - Number of agents
%
% Outputs:
%   ITAE_formation - Integral of Time-weighted Absolute Error for formation
%   ITAE_tracking  - Integral of Time-weighted Absolute Error for tracking
%   ControlEnergy  - Total control energy, ∑(u^2)*T
%
% Dr. in Automatic Control – Research and Development

%% 1) Compute Formation Error at Each Time Step
% We assume 'edgeError' is [l x num_steps], where l is the number of edges.
% We average the absolute edge error across all edges to get one formation error per time step.
num_steps = length(t);
formationError_time = mean(abs(edgeError), 1);  % [1 x num_steps] (average across edges)

% ITAE_formation = ∑ [ t(k) * |formationError_time(k)| ] * T
% Make sure the length of edgeError matches length(t). 
% If edgeError is [l x (num_steps-1)], adjust indices accordingly.
ITAE_formation = sum(t .* formationError_time) * T;

%% 2) Compute Tracking Error at Each Time Step (Velocity-based example)
% For each time step, we calculate the average norm of (V - vds).
trackingError_time = zeros(1, num_steps);
for kk = 1:num_steps
   error_sum = 0;
   for j = 1:n
       idx = (j-1)*m + (1:m);
       error_sum = error_sum + norm(V(idx, kk) - vds(:, kk));
   end
   trackingError_time(kk) = error_sum / n;
end

% ITAE_tracking = ∑ [ t(k) * |trackingError_time(k)| ] * T
ITAE_tracking = sum(t .* abs(trackingError_time)) * T;

%% 3) Compute Total Control Energy
% ControlEnergy = ∑ (u^2) * T, summing over all agents and time steps
ControlEnergy = sum(u(:).^2) * T;

%% 4) Plot All Metrics in a Single Figure (3 Subplots)
figure;

% Subplot (1): ITAE for Formation
subplot(1,3,1);
bar(ITAE_formation);
grid on;
title('ITAE (Formation)');
ylabel('Value');

% Subplot (2): ITAE for Tracking
subplot(1,3,2);
bar(ITAE_tracking);
grid on;
title('ITAE (Tracking)');
ylabel('Value');

% Subplot (3): Total Control Energy
subplot(1,3,3);
bar(ControlEnergy);
grid on;
title('Total Control Energy \int u^2 dt');
ylabel('Value');
end
