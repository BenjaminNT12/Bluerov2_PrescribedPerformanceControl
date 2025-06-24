% FILEPATH: /home/nicolas/Github/Matlab_algorithm/Tesis/Matlab/PrescribedPerformanceControlMultiagent/PrescribedPerformanceControl_Multiagent_test.Framework.SPACE_DIM
% This script implements a multi-agent system with prescribed performance control.
% The agents are modeled as double integrators, and the control objective is to make
% them track a desired trajectory while maintaining a certain formation. The control
% law is based on a prescribed performance function and a Lyapunov function. The
% script generates random initial positions for the agents and plots their
% trajectories in 3D.
%
% Inputs:
%   - None
%
% Outputs:
%   - None
%
% Author: Benjamin Nicolas Trinidad
% Date: August 2023

clear;
close all;
clc;
tic

disp("iniciando simulacion")

ANIMATION = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation and Control Parameters Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-------------------------------
% 1. Simulation Time Parameters
%-------------------------------
SimulationParams = struct();
SimulationParams.TIME_STEP  = 0.01;   % Simulation time step [seconds]
SimulationParams.T_END      = 50;     % Total simulation time [seconds]
SimulationParams.TIME       = 0 : SimulationParams.TIME_STEP : SimulationParams.T_END; % Time vector

%-------------------------------------------------
% 2. Multi-Agent Formation Parameters
%-------------------------------------------------
Framework = struct();
Framework.SPACE_DIM          = 3;    % Number of spatial dimensions (3D)
Framework.NUM_AGENTS         = 3;    % Number of agents in the formation
Framework.NUM_EDGES          = 3;    % Number of edges in the formation graph
Framework.STATES_PER_VEHICLE = 12;   % Number of states per vehicle [η₁; η₂; ν₁; ν₂]
Framework.LEADER_AGENT       = 3;    % Index of the leader agent
Framework.distance           = zeros(Framework.NUM_EDGES, 1); % Desired inter-agent distances (to be computed later)

%-------------------------------------------------
% 3. Prescribed Performance Control (PPC) Parameters
%-------------------------------------------------
PPControl = struct();
PPControl.K_V       = 1.5;   % Gain for virtual control (velocity tracking)
PPControl.K_TANG    = 9900;     % Gain for hyperbolic tangent term (robustness)
PPControl.K_SIGMA   = 9750;      % Gain for sigma term (damping)
PPControl.TAU_SIGMA = 10.5;      % Time constant for sigma term

%-------------------------------------------------
% 4. PID Controller Gains (for orientation control)
%-------------------------------------------------
PidControl = struct();
PidControl.KP   = 2*100;   % Proportional gain
PidControl.KD   = 60;    % Derivative gain
PidControl.KI   = 40;    % Integral gain

%-------------------------------------------------
% 5. Prescribed Performance Function (PPF) Parameters
%-------------------------------------------------
PPF = struct();
PPF.DELTA_UPPER_LIMIT = 4;      % Upper bound for performance envelope
PPF.DELTA_LOWER_LIMIT = 4;      % Lower bound for performance envelope
PPF.PPF_START         = 1;      % Initial value of the performance function
PPF.PPF_END           = 0.07;   % Final value of the performance function
PPF.BETA              = 0.9;    % Exponential decay rate for the performance function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prescribed Performance Function (PPF) Calculation and Agent Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-----------------------------------------------------
% 1. Prescribed Performance Function (PPF) Calculation
%-----------------------------------------------------
% The PPF defines a time-varying performance envelope for the tracking error.
% It starts at PPF_START and exponentially decays to PPF_END with rate BETA.
% This envelope is used to guarantee transient and steady-state performance.
%
% Equation:
%   PPF(t) = (PPF_START - PPF_END) * exp(-BETA * t) + PPF_END
PPF.PPF = (PPF.PPF_START - PPF.PPF_END) ...
    .* exp(-PPF.BETA * SimulationParams.TIME) ...
    + PPF.PPF_END;

% Pre-allocate arrays for the upper and lower bounds of the PPF for each edge.
PPF.b_plus  = zeros(Framework.NUM_EDGES, 1);
PPF.b_minus = zeros(Framework.NUM_EDGES, 1);

%---------------------------------
% 2. Agent Initial Position Setup
%---------------------------------
% Define the initial positions for each agent in 3D space (meters).
% These are column vectors [x; y; z] for each agent, scaled by 3 for scenario.
AgentSetup = struct();
AgentSetup.AGENT_1_POS_M = 3 * [1.5579; 1.2313; -0.6123]; % Agent 1 initial position
AgentSetup.AGENT_2_POS_M = 3 * [3.5815; 2.8371; -0.1681]; % Agent 2 initial position
AgentSetup.AGENT_3_POS_M = 3 * [3.0201; 4.3460; -0.2113]; % Agent 3 initial position

% Aggregate all initial positions into a single column vector for convenience.
% The vector is stacked as [agent1; agent2; agent3], each as [x; y; z].
AgentSetup.positionsArray_M = [
    AgentSetup.AGENT_1_POS_M;
    AgentSetup.AGENT_2_POS_M;
    AgentSetup.AGENT_3_POS_M
];

%-------------------------------------------
% 3. Agent Trajectory Buffer Initialization
%-------------------------------------------
% Pre-allocate a buffer to store the trajectory (positions) of each agent
% over the simulation time for later plotting or animation.
% Dimensions: [time_steps x num_agents x space_dim]
AgentSetup.agentTrajectoryBuffer_M = zeros( ...
    length(SimulationParams.TIME) - 1, ...
    Framework.NUM_AGENTS, ...
    Framework.SPACE_DIM ...
        );
        
%-------------------------------------------
% 4. Desired Target Positions for Each Agent
%-------------------------------------------
% Define the desired (goal) positions for each agent in 3D space (meters).
% These are row vectors [x, y, z] for each agent, scaled by 3 for scenario.
DesiredTargets = struct();
DesiredTargets.AGENT_1_POS_M = 3 * [1, 1, -1]; % Agent 1 desired position
DesiredTargets.AGENT_2_POS_M = 3 * [3, 1, -1]; % Agent 2 desired position
DesiredTargets.AGENT_3_POS_M = 3 * [2, 3, -1]; % Agent 3 desired position

% Aggregate all desired positions into a single column vector for convenience.
% The vector is stacked as [agent1; agent2; agent3], each as [x; y; z].
DesiredTargets.agentPositionsArray_M = [
    DesiredTargets.AGENT_1_POS_M';
    DesiredTargets.AGENT_2_POS_M';
    DesiredTargets.AGENT_3_POS_M'
];

% Generate plot with initial positions for the agents
figure(1);
for i = 1:Framework.NUM_AGENTS
    index_qx = (i - 1) * Framework.SPACE_DIM + 1;
    index_qy = (i - 1) * Framework.SPACE_DIM + 2;
    index_qz = (i - 1) * Framework.SPACE_DIM + 3;

    plot3(AgentSetup.positionsArray_M(index_qx),...
          AgentSetup.positionsArray_M(index_qy),...
          AgentSetup.positionsArray_M(index_qz),...
          'x', 'LineWidth', 2, 'MarkerSize', 15);

    hold on;
end

% Define the edges of the formation
EdgesFormation = struct();
EdgesFormation.E1       = [1, 2];
EdgesFormation.E2       = [2, 3];
EdgesFormation.E3       = [3, 1];
EdgesFormation.E_ARRAY  = [EdgesFormation.E1; EdgesFormation.E2; EdgesFormation.E3];


% Calculate the initial distances between agents based on the desired positions
% Calculate the desired distances between agents for each edge in the formation
for i = 1:Framework.NUM_EDGES
    % Get the indices for the first and second agent in the edge
    idx_agent1 = (EdgesFormation.E_ARRAY(i, 1) - 1) * Framework.SPACE_DIM + 1;
    idx_agent2 = (EdgesFormation.E_ARRAY(i, 2) - 1) * Framework.SPACE_DIM + 1;
    
    % Compute the Euclidean distance between the two agents' desired positions
    Framework.distance(i, 1) = norm(DesiredTargets.agentPositionsArray_M(idx_agent1:idx_agent1 + 2) ...
                                        - DesiredTargets.agentPositionsArray_M(idx_agent2:idx_agent2 + 2));
end

% Compute the upper and lower bounds for the prescribed performance function for each edge
for i = 1:Framework.NUM_EDGES
    % Upper bound (b_plus) for the prescribed performance function
    % Ensures the error stays within a shrinking envelope over time
    PPF.b_plus(i) = (PPF.DELTA_UPPER_LIMIT^2 ...
                              + 2 * Framework.distance(i, 1) * PPF.DELTA_UPPER_LIMIT) ...
                              / PPF.PPF(1);


    % Lower bound (b_minus) for the prescribed performance function
    % Ensures the error does not exceed the lower performance envelope
    PPF.b_minus(i) = (2 * Framework.distance(i, 1) * PPF.DELTA_LOWER_LIMIT ...
                               - PPF.DELTA_LOWER_LIMIT^2) ...
                               / PPF.PPF(1);

end

% Buffers for storing error and transformed signals over time
ErrorSignals = struct();
ErrorSignals.e        = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Standard error
ErrorSignals.e_plus   = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Positive error bound/component
ErrorSignals.e_minus  = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Negative error bound/component
ErrorSignals.eta_plus = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Transformed error variable (+)
ErrorSignals.eta_minus= zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Transformed error variable (-)
ErrorSignals.etaij    = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Another transformed error variable
ErrorSignals.gammaV   = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Performance function output
ErrorSignals.zeta     = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Transformed error for control
ErrorSignals.epsilon  = zeros(Framework.NUM_EDGES, length(SimulationParams.TIME) - 1); % Possibly a small constant or margin

% Time-series data buffers for Prescribed Performance Control signals
PrescribedPerformanceControl = struct();
PPC_Signals.virtualControlDot_V    = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, length(SimulationParams.TIME) - 1); % Derivative of virtual control (e.g., in Volts/second)
PPC_Signals.virtualControl_V       = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, length(SimulationParams.TIME) - 1);    % Virtual control signal (e.g., in Volts)
PPC_Signals.velocityError_MPS      = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, length(SimulationParams.TIME) - 1);   % Position error vector (in meters)
PPC_Signals.controlInput_V         = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, length(SimulationParams.TIME) - 1);   % Actual control input (e.g., in Volts)
PPC_Signals.velocityErrorSigma_MPS = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, length(SimulationParams.TIME) - 1);  % Velocity error (in meters/second)
PPC_Signals.qTildeDistanceError_M  = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, length(SimulationParams.TIME) - 1); % Transformed distance error (in meters)
PPC_Signals.desiredVelocity_MPS    = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, length(SimulationParams.TIME) - 1);  % Desired velocity (in meters/second)

% Rotation matrices of agents from the previous time step/iteration
% Initialize a 3D array to store the previous rotation matrices for each agent (size: NUM_AGENTS x 3 x 3)
prevAgentRotMatrices = zeros(Framework.NUM_AGENTS, 3, 3);

% Loop over each edge in the formation to compute initial distance errors
for j = 1:Framework.NUM_EDGES
    % Compute the indices for agent j's and i's position in the state vector
    index_qj = EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM;
    index_qi = EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM;
    % Compute the indices for the error vector corresponding to this edge
    index_qij = j * Framework.SPACE_DIM - (Framework.SPACE_DIM - 1) : Framework.SPACE_DIM * j;

    % Calculate the initial relative position error for this edge (difference between agent i and agent j)
    PPC_Signals.qTildeDistanceError_M(index_qij, 1) = AgentSetup.positionsArray_M(index_qi, 1) - AgentSetup.positionsArray_M(index_qj, 1);
end

% Compute the initial transformed distance error vector for all edges
PPC_Signals.qTinDistanceError_M = qtinVector(EdgesFormation.E_ARRAY, PPC_Signals.qTildeDistanceError_M(:, 1), Framework.LEADER_AGENT, Framework.NUM_EDGES, Framework.SPACE_DIM);

% Initialize initial linear and angular velocities for each agent
initialLinearVelocity_MPS = [
    2.2786; 0.7190; 2.8849;
    0.7418; 1.0539; 1.0164;
    0.5796; 0.7611; 2.2201;
];

initialAngularVelocity_RADPS = [
    0.7099; 1.0521; 1.4225;
    1.1179; 0.2414; 0.5600;
    1.2600; 0.3263; 1.2705;
];

% Store initial velocities in AgentSetup structure for clarity and reuse
AgentSetup.initialLinearVelocity_MPS    = initialLinearVelocity_MPS;
AgentSetup.initialAngularVelocity_RADPS = initialAngularVelocity_RADPS;
AgentSetup.velocity_MPS                 = AgentSetup.initialLinearVelocity_MPS + AgentSetup.initialAngularVelocity_RADPS;
    
% Desired trajectory functions
TrajectoryDefVelocity = struct();

%% Trajectory Definition for Leader Agent
% This section defines the desired linear trajectory functions for the leader agent
% in a multi-agent underwater vehicle simulation. The trajectory is parameterized
% as a function of time (t) and includes the position, velocity (first derivative),
% and acceleration (second derivative) for each spatial axis (X, Y, Z).
%
% Equations:
%   - X trajectory:      x(t)   = 2*sin(0.15*t)
%   - Y trajectory:      y(t)   = 2*cos(0.15*t)
%   - Z trajectory:      z(t)   = 0
%   - X velocity:        dx/dt  = 2*0.15*cos(0.15*t)
%   - Y velocity:        dy/dt  = -2*0.15*sin(0.15*t)
%   - Z velocity:        dz/dt  = 0
%   - X acceleration:    d²x/dt² = -2*0.15^2*sin(0.15*t)
%   - Y acceleration:    d²y/dt² = -2*0.15^2*cos(0.15*t)
%   - Z acceleration:    d²z/dt² = 0
%
% The Z trajectory is constant (zero), indicating planar motion in the XY plane.
% The functions are defined as anonymous functions for use in trajectory tracking
% and control algorithms.
% Define the desired linear trajectory functions for the leader agent
TrajectoryDefVelocity.x_linear_desired    = @(t) 2*sin(0.15*t);           % X trajectory
TrajectoryDefVelocity.y_linear_desired    = @(t) 2*cos(0.15*t);           % Y trajectory
TrajectoryDefVelocity.z_linear_desired    = @(t) 1*ones(1,length(t));     % Z trajectory (linear)

% First derivatives (velocities)
TrajectoryDefVelocity.xp_linear_desired   = @(t) 2*0.15*cos(0.15*t);      % dx/dt
TrajectoryDefVelocity.yp_linear_desired   = @(t) -2*0.15*sin(0.15*t);     % dy/dt
TrajectoryDefVelocity.zp_linear_desired   = @(t) zeros(1,length(t));      % dz/dt


% Define the desired angular trajectory functions for the leader agent
%% Desired Angular Trajectory Definitions for the Leader Agent
% This section defines the desired angular trajectories (roll, pitch, yaw) for the leader agent.
% Each trajectory is specified as a function of time, including the angle itself,
% its first derivative (angular velocity), and its second derivative (angular acceleration).
% These trajectories are used as references for the prescribed performance control.

TrajectoryDefVelocity.phi_desired    = @(t) zeros(1, length(t));               % Roll angle
TrajectoryDefVelocity.theta_desired  = @(t) zeros(1, length(t));               % Pitch angle
TrajectoryDefVelocity.psi_desired    = @(t) zeros(1, length(t));               % Yaw angle (constant)

% First derivatives (angular velocities) BODY
TrajectoryDefVelocity.phi_dot_desired   = @(t) zeros(1, length(t));            % d(phi)/dt
TrajectoryDefVelocity.theta_dot_desired = @(t) zeros(1, length(t));            % d(theta)/dt
TrajectoryDefVelocity.psi_dot_desired   = @(t) zeros(1, length(t));            % d(psi)/dt


%% Description
% This section defines the variables or parameters associated with each axis of 'eta1',
% which likely represents the position and orientation states of the underwater vehicle
% in the multi-agent prescribed performance control simulation model.
%definition for each axis of eta1
TrajectoryDefVelocity.eta1_desired = @(t) [TrajectoryDefVelocity.x_linear_desired(t); 
                                          TrajectoryDefVelocity.y_linear_desired(t); 
                                          TrajectoryDefVelocity.z_linear_desired(t)]; % Desired trajectory

%definition for each axis of nu1
TrajectoryDefVelocity.nu1_desired  = @(t) [TrajectoryDefVelocity.xp_linear_desired(t); 
                                          TrajectoryDefVelocity.yp_linear_desired(t); 
                                          TrajectoryDefVelocity.zp_linear_desired(t)];

% This section defines the parameters or variables associated with each axis of the eta2 vector,
% which typically represents a subset of the state variables (such as orientation or position)
% in the underwater vehicle model. The definitions provided here are essential for the
% implementation of prescribed performance control in a multi-agent underwater system.
% definition for each axis of eta2
TrajectoryDefVelocity.eta2_desired = @(t) [TrajectoryDefVelocity.phi_desired(t); 
                                          TrajectoryDefVelocity.theta_desired(t); 
                                          TrajectoryDefVelocity.psi_desired(t)]; % Desired trajectory

% definition for each axis of nu2
TrajectoryDefVelocity.nu2_desired  = @(t) [TrajectoryDefVelocity.phi_dot_desired(t); 
                                          TrajectoryDefVelocity.theta_dot_desired(t); 
                                          TrajectoryDefVelocity.psi_dot_desired(t)];


                                          
% Assigns the desired linear and angular velocity trajectories for the leader agent.
% The desired velocities are extracted from the precomputed trajectory definitions
% at the specified simulation time array.
%
% - TrajectoryDefVelocity.VelLinearLeaderDesired: Desired linear velocity trajectory for the leader.
% - TrajectoryDefVelocity.VelAngularLeaderDesired: Desired angular velocity trajectory for the leader.
% - SimulationParams.TIME: Array of time steps for the simulation.
TrajectoryDefVelocity.VelLinearLeaderDesired  = TrajectoryDefVelocity.eta1_desired(SimulationParams.TIME); % Desired trajectory

TrajectoryDefVelocity.VelAngularLeaderDesired = [0*ones(1,length(SimulationParams.TIME));   % Z y Y descendente
                                                 0*ones(1,length(SimulationParams.TIME));   % Z y X ascendente
                                                 0*ones(1,length(SimulationParams.TIME))];  % X y Y 

% Integral of the desired trajectory TrajectoryDefVelocity.VelLinearLeaderDesired
TrajectoryDefVelocity.PosLeaderDesired = [
    TrajectoryDefVelocity.x_linear_desired(SimulationParams.TIME);
    TrajectoryDefVelocity.y_linear_desired(SimulationParams.TIME);
    TrajectoryDefVelocity.z_linear_desired(SimulationParams.TIME)
];

% Initialize the struct array to store PID gains
PID_gains = struct('Kp', [], 'Ki', [], 'Kd', []);

PID_gains(1).Kp = [PidControl.KP, PidControl.KP, PidControl.KP];
PID_gains(1).Ki = [PidControl.KD, PidControl.KD, PidControl.KD+15];
PID_gains(1).Kd = [PidControl.KI, PidControl.KI, PidControl.KI];

PID_gains(2).Kp = [PidControl.KP, PidControl.KP, PidControl.KP];
PID_gains(2).Ki = [PidControl.KD, PidControl.KD, PidControl.KD+15];
PID_gains(2).Kd = [PidControl.KI, PidControl.KI, PidControl.KI];

PID_gains(3).Kp = [PidControl.KP, PidControl.KP, PidControl.KP];
PID_gains(3).Ki = [PidControl.KD, PidControl.KD, PidControl.KD+15];
PID_gains(3).Kd = [PidControl.KI, PidControl.KI, PidControl.KI];

% Initialize PID controllers using the struct array
PID_controllers = struct();
for i = 1:Framework.NUM_AGENTS
    PID_controllers(i).PID = PIDController(PID_gains(1).Kp, PID_gains(1).Ki, PID_gains(1).Kd);
end

vehicle = struct();

for i = 1:Framework.NUM_AGENTS
    vehicle(i).eta1    = zeros(Framework.SPACE_DIM, 1);
    vehicle(i).eta2    = zeros(Framework.SPACE_DIM, 1);
    vehicle(i).nu1     = zeros(Framework.SPACE_DIM, 1);
    vehicle(i).nu2     = zeros(Framework.SPACE_DIM, 1);
    vehicle(i).u1      = zeros(Framework.SPACE_DIM, length(SimulationParams.TIME) - 1);
    vehicle(i).u2      = zeros(Framework.SPACE_DIM, length(SimulationParams.TIME) - 1);
    vehicle(i).eta     = zeros(6, length(SimulationParams.TIME) - 1);
    vehicle(i).nu      = zeros(6, length(SimulationParams.TIME) - 1);
    vehicle(i).error   = zeros(Framework.SPACE_DIM, length(SimulationParams.TIME) - 1);
    vehicle(i).eta2error_i = zeros(Framework.SPACE_DIM, length(SimulationParams.TIME) - 1);
    vehicle(i).eta2error_v = zeros(Framework.SPACE_DIM, length(SimulationParams.TIME) - 1);
    vehicle(i).phi     = zeros(length(SimulationParams.TIME)-1);
    vehicle(i).theta   = zeros(length(SimulationParams.TIME)-1);
    vehicle(i).psi     = zeros(length(SimulationParams.TIME)-1);
    % vehicle(i).ModelSetting = zeros(length(SimulationParams.TIME)-1);
end


for i = 1:Framework.NUM_AGENTS
    base_idx = (i - 1) * Framework.SPACE_DIM;
    vehicle(i).eta1(:, 1) = AgentSetup.positionsArray_M(base_idx + 1:base_idx + Framework.SPACE_DIM, 1);
    vehicle(i).nu1(:, 1)  = AgentSetup.velocity_MPS(base_idx + 1:base_idx + Framework.SPACE_DIM, 1);
end

% Initialize state vector X
X = zeros(Framework.NUM_AGENTS * Framework.STATES_PER_VEHICLE, length(SimulationParams.TIME));
for i = 1:Framework.NUM_AGENTS
    base_idx = (i - 1) * Framework.STATES_PER_VEHICLE;
    X(base_idx + 1:base_idx + Framework.STATES_PER_VEHICLE, 1) = [
        vehicle(i).eta1(:, 1);
        vehicle(i).eta2(:, 1);
        vehicle(i).nu1(:, 1);
        vehicle(i).nu2(:, 1);
    ];
end

% modelParameters_Bluerov2 struct
for i = 1:Framework.NUM_AGENTS
    modelParameters_Bluerov2(i) = BlueROV2ModelParameters(vehicle(i).nu(:, 1), vehicle(i).eta(:, 1));
end

% Main Loop
totalIterations = length(SimulationParams.TIME) - 1;
hWait = waitbar(0, 'Processing...');  % Inicialización de la barra de progreso



for time_iterator = 1:totalIterations
    % Actualización de la barra de progreso con el porcentaje completado
    if mod(time_iterator, 20) == 0
        waitbar(time_iterator/totalIterations, hWait, sprintf('Progress: %.2f%%', (time_iterator/totalIterations)*100));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 1. Compute Edge Errors and Prescribed Performance Variables
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % For each edge in the formation, calculate the relative position error,
    % prescribed performance bounds, transformed errors, and PPC variables.
    % This section implements the prescribed performance control (PPC) error
    % transformation and envelope logic for each edge.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:Framework.NUM_EDGES
        %----------------------------------------------------------------------
        % (a) Indices for agent positions in the state vector
        %----------------------------------------------------------------------
        idx_agent_i = EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM;
        idx_agent_j = EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM;
        idx_edge    = j * Framework.SPACE_DIM - (Framework.SPACE_DIM - 1) : Framework.SPACE_DIM * j;

        %----------------------------------------------------------------------
        % (b) Relative position error for this edge (vector)
        %     qTilde_ij = q_i - q_j
        %----------------------------------------------------------------------
        PPC_Signals.qTildeDistanceError_M(idx_edge, time_iterator) = ...
                    AgentSetup.positionsArray_M(idx_agent_i, time_iterator) - ...
                    AgentSetup.positionsArray_M(idx_agent_j, time_iterator);

        %----------------------------------------------------------------------
        % (c) Distance error (scalar)
        %     e_ij = ||qTilde_ij|| - d_ij^*
        %----------------------------------------------------------------------
        ErrorSignals.e(j, time_iterator) = ...
                    norm(PPC_Signals.qTildeDistanceError_M(idx_edge, time_iterator)) - Framework.distance(j);

        %----------------------------------------------------------------------
        % (d) Epsilon error (quadratic form)
        %     epsilon_ij = e_ij * (e_ij + 2*d_ij^*)
        %     This is a transformed error used for PPC
        %----------------------------------------------------------------------
        ErrorSignals.epsilon(j, time_iterator) = ...
                    ErrorSignals.e(j, time_iterator) * (ErrorSignals.e(j, time_iterator) + 2 * Framework.distance(j));

        %----------------------------------------------------------------------
        % (e) Prescribed performance bounds (upper/lower envelopes)
        %     e_plus: upper bound, e_minus: lower bound
        %     These bounds shrink over time according to the PPF
        %     e_plus = -d_ij^* + sqrt(d_ij^*^2 + b_plus * PPF)
        %     e_minus = d_ij^* - sqrt(d_ij^*^2 - b_minus * PPF)
        %----------------------------------------------------------------------
        ErrorSignals.e_plus(j, time_iterator) = ...
                    -Framework.distance(j) + sqrt(Framework.distance(j)^2 + PPF.b_plus(j) * PPF.PPF(time_iterator));

        ErrorSignals.e_minus(j, time_iterator) = ...
                    Framework.distance(j) - sqrt(Framework.distance(j)^2 - PPF.b_minus(j) * PPF.PPF(time_iterator));

        %----------------------------------------------------------------------
        % (f) Transformed error variables (for Lyapunov/PPC design)
        %     eta_plus = e_plus^2 + 2*d_ij^* * e_plus
        %     eta_minus = e_minus^2 - 2*d_ij^* * e_minus
        %----------------------------------------------------------------------
        ErrorSignals.eta_plus(j, time_iterator) = ...
                    ErrorSignals.e_plus(j, time_iterator)^2 + 2 * Framework.distance(j) * ErrorSignals.e_plus(j, time_iterator);

        ErrorSignals.eta_minus(j, time_iterator) = ...
                    ErrorSignals.e_minus(j, time_iterator)^2 - 2 * Framework.distance(j) * ErrorSignals.e_minus(j, time_iterator);

        %----------------------------------------------------------------------
        % (g) Normalized error (zeta)
        %     zeta_ij = epsilon_ij / PPF
        %     This normalization ensures the error is relative to the shrinking envelope
        %----------------------------------------------------------------------
        ErrorSignals.zeta(j, time_iterator) = ...
                    ErrorSignals.epsilon(j, time_iterator) / PPF.PPF(time_iterator);

        %----------------------------------------------------------------------
        % (h) GammaV transformation for PPC
        %     gammaV_ij = 0.5 * log( (b_plus*zeta + b_minus*b_plus) / (b_plus*b_minus - b_minus*zeta) )
        %     This transformation is used in the PPC law to ensure error convergence
        %----------------------------------------------------------------------
        ErrorSignals.gammaV(j, time_iterator) = 0.5 * log( ...
                    (PPF.b_plus(j) * ErrorSignals.zeta(j, time_iterator) + PPF.b_minus(j) * PPF.b_plus(j)) / ...
                    (PPF.b_plus(j) * PPF.b_minus(j) - PPF.b_minus(j) * ErrorSignals.zeta(j, time_iterator)) ...
                );
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2. Compute Transformed Distance Errors and Desired Velocities
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This section computes the transformed inter-agent distance errors,
    % the desired velocities for each agent (including formation and leader tracking),
    % and the position error (velocity error) for prescribed performance control.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %--- (a) Compute transformed distance errors for all edges
    %     qTinDistanceError_M = qtinVector(...)
    %     This function computes the transformed relative positions for all edges,
    %     considering the leader agent and the formation structure.
    PPC_Signals.qTinDistanceError_M = qtinVector(EdgesFormation.E_ARRAY, ...
                                                 PPC_Signals.qTildeDistanceError_M(:, time_iterator), ...
                                                 Framework.LEADER_AGENT,...
                                                 Framework.NUM_EDGES,...
                                                 Framework.SPACE_DIM);

    %--- (b) Compute desired velocity for each agent
    %     v_desired = v_leader + omega_leader x qTin
    %     Each agent's desired velocity is the sum of the leader's desired linear velocity
    %     and the cross product of the leader's angular velocity with the transformed distance error.
    for k = 1:Framework.NUM_AGENTS
        PPC_Signals.desiredVelocity_MPS(k*Framework.SPACE_DIM-2: k*Framework.SPACE_DIM, time_iterator) = ...
                                                    TrajectoryDefVelocity.VelLinearLeaderDesired(:, time_iterator) + ...
                                                    cross(TrajectoryDefVelocity.VelAngularLeaderDesired(:, time_iterator), ...
                                                          PPC_Signals.qTinDistanceError_M(k*Framework.SPACE_DIM-2: k*Framework.SPACE_DIM, 1));
    end


    %--- (c) Compute position error (velocity error)
    %     positionError_M = v_actual - v_desired
    %     This is the difference between the actual and desired velocities for all agents.
    
    % no need to compute velocityError_MPS here, as it is already computed in the next section
    % PPC_Signals.velocityError_MPS(:, time_iterator) = ...
    %     AgentSetup.velocity_MPS(:, time_iterator) - PPC_Signals.desiredVelocity_MPS(:, time_iterator);

    %--- (d) Compute R matrix for the current agent positions
    %     The R matrix encodes the formation structure and is used in the PPC law.
    R = matrizRTriangle3AgentWithLeader(AgentSetup.positionsArray_M(:, time_iterator), Framework.SPACE_DIM);

    %--- (e) Compute etaij for each edge (transformed error variable)
    %     etaij_j = (1/PPF) * (1/(zeta_j + b_minus_j) - 1/(zeta_j - b_plus_j))
    %     This transformation is used in the PPC law to ensure error convergence.
    for j = 1:Framework.NUM_EDGES
        ErrorSignals.etaij(j, time_iterator) = (1 / PPF.PPF(time_iterator)) * ...
                                               (1 / (ErrorSignals.zeta(j, time_iterator)...
                                                + PPF.b_minus(j))...
                                                - 1 / (ErrorSignals.zeta(j, time_iterator)...
                                                - PPF.b_plus(j)));
    end

    %--- (f) Construct diagonal eta matrix for current time
    %     This matrix is used in the PPC control law.
    eta = diag(ErrorSignals.etaij(:, time_iterator));

    %--- (g) Compute rhop (discrete-time derivative of eta)
    %     This is the time derivative of the eta matrix, used for feedforward compensation.
    if time_iterator == 1
        rhop     = eta / SimulationParams.TIME_STEP;
        temp_eta = eta;
    else
        rhop     = (eta - temp_eta) / SimulationParams.TIME_STEP;
        temp_eta = eta;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2.1 Virtual Control and Velocity Error Calculation (Prescribed Performance)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This section computes the virtual control signal for the Prescribed Performance Control (PPC)
    % and the associated velocity error. The virtual control is designed to ensure that the
    % system tracks the desired velocity while respecting the prescribed performance bounds.
    %
    % Equation for Virtual Control:
    %   v_virtual = -K_V * R' * eta' * gammaV + v_desired
    %   where:
    %     - K_V: PPC gain
    %     - R: Formation structure matrix
    %     - eta: Diagonal matrix of transformed errors
    %     - gammaV: Transformed error vector for PPC
    %     - v_desired: Desired velocity for each agent
    %
    % The velocity error is then computed as:
    %   velocityError = v_actual - v_virtual
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %--- (a) Compute the virtual control signal for PPC
    PPC_Signals.virtualControl_V(:, time_iterator) = ...
                                -PPControl.K_V * R' * eta' * ErrorSignals.gammaV(:, time_iterator) + ...
                                 PPC_Signals.desiredVelocity_MPS(:, time_iterator);  % [Eq. 1] Virtual control law

    %--- (b) Compute the velocity error (difference between actual and virtual control)
    PPC_Signals.velocityErrorSigma_MPS(:, time_iterator) = ...
        AgentSetup.velocity_MPS(:, time_iterator) - PPC_Signals.virtualControl_V(:, time_iterator); % [Eq. 2] Velocity error

    %--- (c) Compute the derivative of the virtual control (finite difference)
    % This is used as a feedforward term in the PPC law.
    if time_iterator == 1
        PPC_Signals.virtualControlDot_V(:, time_iterator) = ...
            PPC_Signals.virtualControl_V(:, time_iterator) / SimulationParams.TIME_STEP;
    else
        PPC_Signals.virtualControlDot_V(:, time_iterator) = ...
            (PPC_Signals.virtualControl_V(:, time_iterator) - PPC_Signals.virtualControl_V(:, time_iterator - 1)) / SimulationParams.TIME_STEP;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model Matrices Initialization for Each Agent
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % These matrices will be built up for all agents in the next loop
    GAMMA_MATRIX = [];
    Ceta1V = [];
    Deta1V = [];
    geta1V = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 3. Dynamic Model and Error Calculation for Each Vehicle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % For each agent, compute the tracking error, update the rotation matrix,
    % build the dynamic model matrices (Gamma, C, D, g), and update error integrals.
    % This section prepares the matrices for the PPC control law.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for j = 1:Framework.NUM_AGENTS
        %----------------------------------------------------------------------
        % (a) Tracking Error Calculation
        %     error = eta2_desired - eta2_actual
        %     eta2: orientation (roll, pitch, yaw)
        %----------------------------------------------------------------------
        vehicle(j).eta2error(:, time_iterator) = ...
            TrajectoryDefVelocity.eta2_desired(SimulationParams.TIME(time_iterator)) - ...
            vehicle(j).eta2(:, time_iterator);

        %----------------------------------------------------------------------
        % (b) Rotation Matrix Calculation
        %     R(φ, θ, ψ): rotation matrix from body to inertial frame
        %----------------------------------------------------------------------
        vehicleRotationMatrix = rotationMatrix( ...
            vehicle(j).eta2(1, time_iterator), ...
            vehicle(j).eta2(2, time_iterator), ...
            vehicle(j).eta2(3, time_iterator) ...
        );

        %----------------------------------------------------------------------
        % (c) Update Model Parameters for Current State
        %     BlueROV2ModelParameters(nu, eta): returns updated model parameters
        %----------------------------------------------------------------------
        modelParameters_Bluerov2(j) = BlueROV2ModelParameters( ...
            vehicle(j).nu(:, time_iterator), ...
            vehicle(j).eta(:, time_iterator) ...
        );

        %----------------------------------------------------------------------
        % (d) Inertia Matrix and Its Inverse
        %     M = [M11, M12; M21, M22]
        %     M_inv = inv(M)
        %----------------------------------------------------------------------
        M = [modelParameters_Bluerov2(j).M11, modelParameters_Bluerov2(j).M12; ...
             modelParameters_Bluerov2(j).M21, modelParameters_Bluerov2(j).M22];
        M_inv = inv(M);

        % Split M_inv into 3x3 blocks for further calculations
        inv_M11 = M_inv(1:3, 1:3);
        inv_M12 = M_inv(1:3, 4:6);
        inv_M21 = M_inv(4:6, 1:3);
        inv_M22 = M_inv(4:6, 4:6);

        %----------------------------------------------------------------------
        % (e) Gamma Matrix Construction
        %     Gamma = block diagonal of (R * M11)
        %     Used for transforming control inputs to inertial frame
        %----------------------------------------------------------------------
        rotation_M11_matrix_product = vehicleRotationMatrix * modelParameters_Bluerov2(j).M11;
        GAMMA_MATRIX = blkdiag(GAMMA_MATRIX, rotation_M11_matrix_product);

        %----------------------------------------------------------------------
        % (f) Dynamic Model Matrices in Body Frame
        %     Cnu1 = inv_M11 * C11 + inv_M12 * C21
        %     Dnu1 = -inv_M11 * D11
        %     gnu1 = -inv_M11 * g1
        %     These represent Coriolis, damping, and gravity terms
        %----------------------------------------------------------------------
        Cnu1 = -inv_M11 * modelParameters_Bluerov2(j).C11 - inv_M12 * modelParameters_Bluerov2(j).C21;
        Dnu1 = -inv_M11 * modelParameters_Bluerov2(j).D11;
        gnu1 = -inv_M11 * modelParameters_Bluerov2(j).g1;

        %----------------------------------------------------------------------
        % (g) Rotation Matrix Derivative (for time-varying orientation)
        %     Used in transformation of dynamic terms to inertial frame
        %----------------------------------------------------------------------
        diff_RotationMatrix = (vehicleRotationMatrix - squeeze(prevAgentRotMatrices(j, :, :))) / SimulationParams.TIME_STEP;
        prevAgentRotMatrices(j, :, :) = vehicleRotationMatrix;
        
        %----------------------------------------------------------------------
        % (h) Transform Dynamic Model Matrices to Inertial Frame
        %     Ceta1i = R * Cnu1 / R + dR/dt / R
        %     Deta1i = R * Dnu1 / R
        %     geta1i = R * gnu1
        %     These are used in the PPC control law
        %----------------------------------------------------------------------
        Ceta1i = vehicleRotationMatrix * Cnu1 / vehicleRotationMatrix + diff_RotationMatrix / vehicleRotationMatrix;
        Deta1i = vehicleRotationMatrix * Dnu1 / vehicleRotationMatrix;
        geta1i = vehicleRotationMatrix * gnu1;

        %----------------------------------------------------------------------
        % Derivate the follow equation vehicle(j).nu1;
        %----------------------------------------------------------------------
        % Compute the derivative of nu1 (nu1dot) using finite differences
        if time_iterator == 1
            nu1dot = zeros(Framework.SPACE_DIM, 1);
        else
            nu1dot = (vehicle(j).nu1(:, time_iterator) - vehicle(j).nu1(:, time_iterator - 1)) / SimulationParams.TIME_STEP;
        end
        % nu1dot now contains the time derivative of nu1 for agent j at this time step

        %----------------------------------------------------------------------
        % (i) Stack Dynamic Terms for All Agents
        %     These vectors are used to build the OMEGA term in the PPC law
        %----------------------------------------------------------------------
        Ceta1V = vertcat(Ceta1V, Ceta1i * nu1dot);
        Deta1V = vertcat(Deta1V, Deta1i * nu1dot);
        geta1V = vertcat(geta1V, geta1i);

        %----------------------------------------------------------------------
        % (j) Error Integrator for PID Control
        %     error_i = integral of error over time
        %     error_v = derivative error (nu2_desired - nu2_actual)
        %----------------------------------------------------------------------
        if time_iterator == 1
            vehicle(j).eta2error_i(:, time_iterator) = 0;
        else
            vehicle(j).eta2error_i(:, time_iterator) = ...
                vehicle(j).eta2error_i(:, time_iterator-1) + ...
                vehicle(j).eta2error(:, time_iterator) * SimulationParams.TIME_STEP;
        end
        vehicle(j).eta2error_v(:, time_iterator) = ...
            TrajectoryDefVelocity.nu2_desired(SimulationParams.TIME(time_iterator)) - ...
            vehicle(j).nu2(:, time_iterator);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 4. Control Law Calculation and State Update
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This section computes the control input for each agent using the PPC law,
    % updates the PID control for orientation, aggregates the control inputs,
    % and advances the system state using RK4 integration.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %----------------------------------------------------------------------
    % (a) Compute OMEGA: Dynamic Model Terms
    %     OMEGA = Ceta1V + Deta1V + geta1V
    %     This term aggregates Coriolis, damping, and gravity effects for all agents.
    %----------------------------------------------------------------------
    OMEGA = Ceta1V + Deta1V + geta1V;

    %----------------------------------------------------------------------
    % (b) Compute PPC Control Input (u1) for All Agents
    %     Equation:
    %     u1 = GAMMA^{-1} * [virtualControlDot - R' * eta' * gammaV - OMEGA
    %           - ((TAU_SIGMA^2)/2 + K_SIGMA) * velocityError
    %           - K_TANG * tanh(velocityError)]
    %
    %     This is the main PPC law for the translational dynamics.
    %----------------------------------------------------------------------
    PPC_Signals.controlInput_V(:, time_iterator) = GAMMA_MATRIX \ ( ...
        PPC_Signals.virtualControlDot_V(:, time_iterator) ...
        - R' * eta' * ErrorSignals.gammaV(:, time_iterator) ...
        - OMEGA ...
        - ((PPControl.TAU_SIGMA^2) / 2 + PPControl.K_SIGMA) * PPC_Signals.velocityErrorSigma_MPS(:, time_iterator) ...
        - PPControl.K_TANG * tanh(PPC_Signals.velocityErrorSigma_MPS(:, time_iterator)) ...
    );

    %----------------------------------------------------------------------
    % (c) Compute PID Control Input (u2) for Orientation for Each Agent
    %     u2 = PID(error, error_i, error_v)
    %     This is the PID law for the rotational/orientation dynamics.
    %----------------------------------------------------------------------
    for j = 1:Framework.NUM_AGENTS
        vehicle(j).u2(:, time_iterator) = PID_controllers(j).PID.computeControl( ...
            vehicle(j).eta2error(:, time_iterator), ...
            vehicle(j).eta2error_i(:, time_iterator), ...
            vehicle(j).eta2error_v(:, time_iterator) ...
        );
    end

    %----------------------------------------------------------------------
    % (d) Assign PPC Control Input (u1) to Each Agent
    %     Extract the relevant slice from the global control vector.
    %----------------------------------------------------------------------
    for j = 1:Framework.NUM_AGENTS
        index_u1 = j * Framework.SPACE_DIM - 2 : j * Framework.SPACE_DIM;
        vehicle(j).u1(:, time_iterator) = PPC_Signals.controlInput_V(index_u1, time_iterator);
    end
    %----------------------------------------------------------------------
    % (e) Aggregate Control Inputs for All Agents
    %     u = [u1; u2] for each agent, stacked into a single vector.
    %----------------------------------------------------------------------
    for j = 1:Framework.NUM_AGENTS
        base_u = (j - 1) * 6;
        index_u = base_u + 1 : base_u + 6;
        u(index_u, time_iterator) = [vehicle(j).u1(:, time_iterator); vehicle(j).u2(:, time_iterator)];
    end

    %----------------------------------------------------------------------
    % (f) Advance System State Using RK4 Integration
    %     X(:, t+1) = RK4step_params(...)
    %     This integrates the multi-agent underwater vehicle model forward in time.
    %----------------------------------------------------------------------
    X(:, time_iterator + 1) = RK4step_new( ...
        @multiagent_underwater_model_bluerov2Params, ...
        SimulationParams.TIME(time_iterator), ...
        X(:, time_iterator), ...
        SimulationParams.TIME_STEP, ...
        u(:, time_iterator), ...
        Framework.SPACE_DIM, ...
        Framework.NUM_AGENTS, ...
        modelParameters_Bluerov2(:) ...
    );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update States for Each Vehicle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:Framework.NUM_AGENTS
        %---------------------------------------------------------------------- 
        % (a) Calculate the starting index for the state vector of the current vehicle
        %     Each vehicle has a state vector of size STATES_PER_VEHICLE
        %---------------------------------------------------------------------- 
        start_idx = (j - 1) * Framework.STATES_PER_VEHICLE + 1;
        base_idx  = (j - 1) * Framework.SPACE_DIM;

        %---------------------------------------------------------------------- 
        % (b) Update Position (η₁), Orientation (η₂), Linear Velocity (ν₁),
        %     and Angular Velocity (ν₂) for the current vehicle
        %---------------------------------------------------------------------- 
        vehicle(j).eta1(:, time_iterator + 1) = X(start_idx:start_idx + 2, time_iterator + 1); % η₁: Position
        vehicle(j).eta2(:, time_iterator + 1) = X(start_idx + 3:start_idx + 5, time_iterator + 1); % η₂: Orientation
        vehicle(j).nu1(:, time_iterator + 1)  = X(start_idx + 6:start_idx + 8, time_iterator + 1); % ν₁: Linear Velocity
        vehicle(j).nu2(:, time_iterator + 1)  = X(start_idx + 9:start_idx + 11, time_iterator + 1); % ν₂: Angular Velocity

        %---------------------------------------------------------------------- 
        % (c) Aggregate Position and Orientation (η) and Velocities (ν)
        %---------------------------------------------------------------------- 
        vehicle(j).eta(:, time_iterator + 1) = X(start_idx:start_idx + 5, time_iterator + 1); % η = [η₁; η₂]
        vehicle(j).nu(:, time_iterator + 1)  = X(start_idx + 6:start_idx + 11, time_iterator + 1); % ν = [ν₁; ν₂]

        %---------------------------------------------------------------------- 
        % (d) Update Initial Positions and Velocities in AgentSetup Structure
        %---------------------------------------------------------------------- 
        AgentSetup.positionsArray_M(base_idx + 1:base_idx + Framework.SPACE_DIM, time_iterator + 1) = vehicle(j).eta1(:, time_iterator + 1); % Update position
        AgentSetup.velocity_MPS(base_idx + 1:base_idx + Framework.SPACE_DIM, time_iterator + 1) = vehicle(j).nu1(:, time_iterator + 1); % Update velocity

    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update Agent Trajectories and Orientation Angles
    % This section updates the trajectory buffer for animation and stores the
    % orientation angles (phi, theta, psi) for each agent at the current time step.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Update trajectory buffer for animation
    % The trajectory buffer stores the 3D positions (X, Y, Z) of each agent
    % at the current time step for visualization purposes.
    for j = 1:Framework.NUM_AGENTS
        % Extract the X, Y, Z positions of agent `j` from the positions array
        AgentSetup.agentTrajectoryBuffer_M(time_iterator, j, :) = [
            AgentSetup.positionsArray_M(j * Framework.SPACE_DIM - 2, time_iterator+1), ... % X position
            AgentSetup.positionsArray_M(j * Framework.SPACE_DIM - 1, time_iterator+1), ... % Y position
            AgentSetup.positionsArray_M(j * Framework.SPACE_DIM,     time_iterator+1)         % Z position
        ];
    end

    % Update orientation angles (phi, theta, psi) for each agent
    % Orientation angles are extracted from the eta2 state variable of each agent.
    % These angles represent the roll (phi), pitch (theta), and yaw (psi) of the agent.
    for j = 1:Framework.NUM_AGENTS
        % Store the roll (phi), pitch (theta), and yaw (psi) angles for agent `j`
        % at the current time step.
        vehicle(j).phi(time_iterator) = vehicle(j).eta2(1, time_iterator + 1);   % Roll (phi)
        vehicle(j).theta(time_iterator) = vehicle(j).eta2(2, time_iterator + 1); % Pitch (theta)
        vehicle(j).psi(time_iterator) = vehicle(j).eta2(3, time_iterator + 1);   % Yaw (psi)
    end
    
end





close(hWait);  % Cierre de la barra de progreso


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animation and Trajectory Plotting Section
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This section handles the visualization of the agents' trajectories.
% If ANIMATION is enabled, it animates the 3D movement of the agents.
% Otherwise, it plots the 3D trajectories of the leader and agent 3,
% and overlays the final formation framework.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp("Plotting")

if ANIMATION == 1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 3D Animation of Agent Trajectories
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calls a custom animation function to visualize the movement of all agents
    % in 3D space over time, along with the leader's desired trajectory.
    %
    % Inputs:
    %   - AgentSetup.agentTrajectoryBuffer_M: [time_steps x num_agents x 3] positions
    %   - TrajectoryDefVelocity.VelLinearLeaderDesired: Leader's desired trajectory
    %   - SimulationParams.TIME: Time vector
    %   - AgentSetup.positionsArray_M: Initial positions of all agents
    %   - EdgesFormation.E_ARRAY: Formation edges
    %   - f: Figure handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    f = figure(1);
    
    plotAnimation3Agents_Optimized( ...
        AgentSetup.agentTrajectoryBuffer_M, ...
        TrajectoryDefVelocity.VelLinearLeaderDesired, ...
        SimulationParams.TIME, ...
        AgentSetup.positionsArray_M, ...
        EdgesFormation.E_ARRAY, ...
        f ...
    );
else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Static 3D Trajectory Plotting
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plots the desired trajectory of the leader and the actual trajectory
    % of agent 3 in 3D space. Also overlays the final formation framework.
    %
    % Equation for Leader's Desired Trajectory (for plotting):
    %   X_leader = 22.23 - (1 / 0.132) * y_linear_desired
    %   Y_leader = 8.8   + (1 / 0.132) * x_linear_desired
    %   Z_leader = time (for visualization)
    %
    % The agent's trajectory is plotted using its position history.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(1);

    % plot3(19.23 - (1 / 0.163) * vds(2, :)', 12.8 + (1 / 0.163) * vds(1, :)', 2.8 * 1.0 + t(:), 'LineStyle', '-.', 'Color', 'red', 'LineWidth', 2);

    % Plot the leader's desired trajectory in 3D (red dashed line)
    plot3( ...
        18.4 - (1 / 0.192) * TrajectoryDefVelocity.VelLinearLeaderDesired(2, :)', ...
        11.8 + (1 / 0.192) * TrajectoryDefVelocity.VelLinearLeaderDesired(1, :)', ...
        0.8*SimulationParams.TIME', ...
        'LineStyle', '-.', 'Color', 'red', 'LineWidth', 2 ...
    );
    hold on;
    
    % Plot the actual trajectory of agent 3 in 3D (blue solid line)
    plot3( ...
        AgentSetup.positionsArray_M(3 * Framework.SPACE_DIM - 2, :), ...
        AgentSetup.positionsArray_M(3 * Framework.SPACE_DIM - 1, :), ...
        AgentSetup.positionsArray_M(3 * Framework.SPACE_DIM, :), ...
        'LineStyle', '-', 'Color', 'blue', 'LineWidth', 2 ...
    );
    hold on;

    % Overlay the final formation framework (edges between agents)
    [grf, points] = Framework3Dplot(AgentSetup.positionsArray_M(:, end), EdgesFormation.E_ARRAY);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting Prescribed Performance Bounds and Edge Errors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This section visualizes the prescribed performance envelopes (upper and lower bounds)
% and the actual inter-agent distance errors for each edge in the formation.
% - For edge 1, the plot is shown in Figure 33.
% - For edges 2 and 3, the plot is shown in Figure 34.
% The plots help verify that the errors remain within the prescribed bounds.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

temp = 0; % Flag to track if any error exceeds its bounds

for i = 1:Framework.NUM_EDGES
    % Select figure and legend label based on edge index
    if ismember(i, [1])
        %-------------------------------
        % Plot for Edge 1 (Figure 33)
        %-------------------------------
        figure(33);
        % Plot upper performance bound (e_plus)
        plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e_plus(i, :), ...
            'Color', 'r', 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Upper Bound');
        hold on;
        % Plot lower performance bound (-e_minus)
        plot(SimulationParams.TIME(1:end - 1), -ErrorSignals.e_minus(i, :), ...
            'Color', 'r', 'LineWidth', 2, 'LineStyle', '--', 'HandleVisibility', 'off');
        hold on;
        % Plot actual edge error (e)
        plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e(i, :), ...
            'LineWidth', 2, 'DisplayName', 'Edge Error');
        hold on;
        grid on;
        xlabel('Tiempo [Seg]', 'FontSize', 18);
        ylabel('Error $e_{ij}$', 'Interpreter', 'latex', 'FontSize', 18);
        title('Prescribed Performance Bounds and Edge Error (Edge 1)', 'FontSize', 16);
        legend('show');
        % Check if error exceeds bounds
        if any(ErrorSignals.e(i, :) > ErrorSignals.e_plus(i, :) | ...
               ErrorSignals.e(i, :) < -ErrorSignals.e_minus(i, :)) && temp ~= i
            temp = i;
        end
    end

    if ismember(i, [2, 3])
        %-------------------------------
        % Plot for Edges 2 and 3 (Figure 34)
        %-------------------------------
        figure(34);
        % Plot upper performance bound (e_plus)
        plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e_plus(i, :), ...
            'Color', 'r', 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Upper Bound');
        hold on;
        % Plot lower performance bound (-e_minus)
        plot(SimulationParams.TIME(1:end - 1), -ErrorSignals.e_minus(i, :), ...
            'Color', 'r', 'LineWidth', 2, 'LineStyle', '--', 'HandleVisibility', 'off');
        hold on;
        % Plot actual edge error (e)
        plot(SimulationParams.TIME(1:end - 1), ErrorSignals.e(i, :), ...
            'LineWidth', 2, 'DisplayName', ['Edge Error (Edge ', num2str(i), ')']);
        hold on;
        grid on;
        xlabel('Tiempo [Seg]', 'FontSize', 18);
        ylabel('Error $e_{ij}$', 'Interpreter', 'latex', 'FontSize', 18);
        title('Prescribed Performance Bounds and Edge Errors (Edges 2 & 3)', 'FontSize', 16);
        % Only show legend once
        if i == 2
            legend('show');
        end
        % Check if error exceeds bounds
        if any(ErrorSignals.e(i, :) > ErrorSignals.e_plus(i, :) | ...
               ErrorSignals.e(i, :) < -ErrorSignals.e_minus(i, :)) && temp ~= i
            temp = i;
        end
    end
end


figure;
view([-45, -90, 60]);
% axis([-4 4 -4 4 -4 4]); % Uncomment if adaptive window is desired
grid on;
[grf, points] = Framework3Dplot(DesiredTargets.agentPositionsArray_M(:, 1), EdgesFormation.E_ARRAY);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Orientation Tracking Plots for Each Agent
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This section visualizes the orientation (roll, pitch, yaw) of each agent
% over time, comparing the actual orientation to the desired orientation.
% Each agent's orientation is plotted in a separate row of subplots:
%   - Column 1: Roll (φ)
%   - Column 2: Pitch (θ)
%   - Column 3: Yaw (ψ)
% The desired orientation is overlaid for reference.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
desired_trajectory = zeros(3, length(SimulationParams.TIME)-1);

% Compute the desired orientation trajectory for all time steps
for i = 1:length(SimulationParams.TIME)-1
    desired_trajectory(:, i) = TrajectoryDefVelocity.eta2_desired(SimulationParams.TIME(i));
end

% Loop through each agent to plot orientation tracking
for agent_idx = 1:Framework.NUM_AGENTS
    %----------------------------------------------------------------------
    % Extract orientation states (φ, θ, ψ) for the current agent
    %----------------------------------------------------------------------
    % State vector X layout for each agent:
    %   [η₁(1:3); η₂(4:6); ν₁(7:9); ν₂(10:12)]
    % η₂ (orientation) starts at index 4 for each agent's block
    start_idx = (agent_idx - 1) * Framework.STATES_PER_VEHICLE + 4;
    phi   = X(start_idx,   1:end-1); % Roll
    theta = X(start_idx+1, 1:end-1); % Pitch
    psi   = X(start_idx+2, 1:end-1); % Yaw

    time_vec = SimulationParams.TIME(1:end-1);

    %----------------------------------------------------------------------
    % Plot Roll (φ) Tracking
    %----------------------------------------------------------------------
    subplot(Framework.NUM_AGENTS, 3, (agent_idx - 1) * 3 + 1);
    plot(time_vec, phi, 'LineWidth', 2, 'DisplayName', 'Actual');
    hold on;
    plot(time_vec, desired_trajectory(1, :), 'LineWidth', 2, 'DisplayName', 'Desired');
    grid on;
    ylim([-0.5, 0.5]);
    xlabel('Tiempo [Seg]', 'FontSize', 18);
    ylabel(['$\phi_{', num2str(agent_idx), '}$ [rad]'], 'Interpreter', 'latex', 'FontSize', 18);
    title(['Agent ', num2str(agent_idx), ' Roll'], 'Interpreter', 'latex', 'FontSize', 18);
    legend('show');

    %----------------------------------------------------------------------
    % Plot Pitch (θ) Tracking
    %----------------------------------------------------------------------
    subplot(Framework.NUM_AGENTS, 3, (agent_idx - 1) * 3 + 2);
    plot(time_vec, theta, 'LineWidth', 2, 'DisplayName', 'Actual');
    hold on;
    plot(time_vec, desired_trajectory(2, :), 'LineWidth', 2, 'DisplayName', 'Desired');
    grid on;
    ylim([-0.5, 0.5]);
    xlabel('Tiempo [Seg]', 'FontSize', 18);
    ylabel(['$\theta_{', num2str(agent_idx), '}$ [rad]'], 'Interpreter', 'latex', 'FontSize', 18);
    title(['Agent ', num2str(agent_idx), ' Pitch'], 'Interpreter', 'latex', 'FontSize', 18);
    legend('show');

    %----------------------------------------------------------------------
    % Plot Yaw (ψ) Tracking
    %----------------------------------------------------------------------
    subplot(Framework.NUM_AGENTS, 3, (agent_idx - 1) * 3 + 3);
    plot(time_vec, psi, 'LineWidth', 2, 'DisplayName', 'Actual');
    hold on;
    plot(time_vec, desired_trajectory(3, :), 'LineWidth', 2, 'DisplayName', 'Desired');
    grid on;
    ylim([-0.5, 0.5]);
    xlabel('Tiempo [Seg]', 'FontSize', 18);
    ylabel(['$\psi_{', num2str(agent_idx), '}$ [rad]'], 'Interpreter', 'latex', 'FontSize', 18);
    title(['Agent ', num2str(agent_idx), ' Yaw'], 'Interpreter', 'latex', 'FontSize', 18);
    legend('show');
end

% Add a super title for the entire orientation tracking figure
% sgtitle('Orientación 3 Vehiculos', 'Interpreter', 'latex', 'FontSize', 20);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Input Plotting for Each Agent (u_x, u_y, u_z)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This section visualizes the control inputs (u) applied to each agent
% along the X, Y, and Z axes over the simulation time.
% Each subplot corresponds to one axis, and each agent's control input is
% plotted in the same subplot for comparison.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
% Loop through each agent to plot their control inputs for each axis
for i = 1:Framework.NUM_AGENTS
    %-------------------------------
    % Plot Control Input for X Axis
    %-------------------------------
    subplot(3,1,1);
    plot(SimulationParams.TIME(1:end-1), u(i*Framework.SPACE_DIM-2, :), 'LineWidth', 2);
    hold on;
    title('Control Input $u_x$ for Each Agent', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('$u_x$', 'Interpreter', 'latex', 'FontSize', 12);
    grid on;

    %-------------------------------
    % Plot Control Input for Y Axis
    %-------------------------------
    subplot(3,1,2);
    plot(SimulationParams.TIME(1:end-1), u(i*Framework.SPACE_DIM-1, :), 'LineWidth', 2);
    hold on;
    title('Control Input $u_y$ for Each Agent', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('$u_y$', 'Interpreter', 'latex', 'FontSize', 12);
    grid on;

    %-------------------------------
    % Plot Control Input for Z Axis
    %-------------------------------
    subplot(3,1,3);
    plot(SimulationParams.TIME(1:end-1), u(i*Framework.SPACE_DIM, :), 'LineWidth', 2);
    hold on;
    title('Control Input $u_z$ for Each Agent', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Time [s]', 'FontSize', 12);
    ylabel('$u_z$', 'Interpreter', 'latex', 'FontSize', 12);
    grid on;
end

% Add legends to each subplot for agent identification
subplot(3,1,1);
legend(arrayfun(@(x) sprintf('Agent %d', x), 1:Framework.NUM_AGENTS, 'UniformOutput', false), 'Location', 'best');
subplot(3,1,2);
legend(arrayfun(@(x) sprintf('Agent %d', x), 1:Framework.NUM_AGENTS, 'UniformOutput', false), 'Location', 'best');
subplot(3,1,3);
legend(arrayfun(@(x) sprintf('Agent %d', x), 1:Framework.NUM_AGENTS, 'UniformOutput', false), 'Location', 'best');

sgtitle('Control Inputs ($u_x$, $u_y$, $u_z$) for All Agents', 'Interpreter', 'latex', 'FontSize', 16);

plotPositionErrors(3, AgentSetup.positionsArray_M, TrajectoryDefVelocity.PosLeaderDesired, Framework.SPACE_DIM, SimulationParams.TIME);

figure;
% plot each axis position of the agent 3 
plot(SimulationParams.TIME(1:end), AgentSetup.positionsArray_M(3 * Framework.SPACE_DIM - 2, :), 'LineWidth', 2, 'DisplayName', 'Posición X');
hold on;
plot(SimulationParams.TIME(1:end), AgentSetup.positionsArray_M(3 * Framework.SPACE_DIM - 1, :), 'LineWidth', 2, 'DisplayName', 'Posición Y');
plot(SimulationParams.TIME(1:end), AgentSetup.positionsArray_M(3 * Framework.SPACE_DIM, :), 'LineWidth', 2, 'DisplayName', 'Posición Z');
grid on;

% Plot TrajectoryDefVelocity.PosLeaderDesired
% figure(2);
plot(SimulationParams.TIME, TrajectoryDefVelocity.PosLeaderDesired(1, :),'--' ,'LineWidth', 2, 'DisplayName', 'TrajectoryDefVelocity.VelLinearLeaderDesired X');
hold on;
plot(SimulationParams.TIME, TrajectoryDefVelocity.PosLeaderDesired(2, :),'--', 'LineWidth', 2, 'DisplayName', 'TrajectoryDefVelocity.VelLinearLeaderDesired Y');
plot(SimulationParams.TIME, TrajectoryDefVelocity.PosLeaderDesired(3, :),'--', 'LineWidth', 2, 'DisplayName', 'TrajectoryDefVelocity.VelLinearLeaderDesired Z');
grid on;
xlabel('Tiempo [s]', 'FontSize', 12);
ylabel('Integral de TrajectoryDefVelocity.VelLinearLeaderDesired', 'FontSize', 12);
title('Integral de la Trayectoria Deseada', 'FontSize', 14);
legend show;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Metric Calculation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ErrorSignals.e(1:21,end+1) = ErrorSignals.e(1:21,end);
[avgFormErr, avgTrackErr, totCtrlEff, avgCtrlEff] = calculatePerformanceMetrics(SimulationParams.TIME(1:end-1), ErrorSignals.e, AgentSetup.velocity_MPS, TrajectoryDefVelocity.VelLinearLeaderDesired, u(1:9,:), SimulationParams.TIME_STEP, Framework.SPACE_DIM, Framework.NUM_AGENTS);
[ITAE_formation, ITAE_tracking, ControlEnergy] = calculateAdditionalMetrics(SimulationParams.TIME(1:end-1), ErrorSignals.e, AgentSetup.velocity_MPS, TrajectoryDefVelocity.VelLinearLeaderDesired, u(1:9,:), SimulationParams.TIME_STEP, Framework.SPACE_DIM, Framework.NUM_AGENTS);

disp("Prescribed Performance Control Metrics:")
disp("ITAE (Formation): " + ITAE_formation)
disp("ITAE (Tracking):  " + ITAE_tracking)
disp("Total Control Energy: " + ControlEnergy)

% Adjust layout for better visualization
sgtitle('Velocity Errors (position Error [M]) for 9 Agents', 'FontSize', 14);

toc
