% close(hWait);  % Cierre de la barra de progreso
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

clear; close all; clc;
tic
disp("iniciando simulacion")

ANIMATION = 1;

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
SimulationParams.noise      = 0.00; % Noise level for sensor measurements
totalIterations             = numel(SimulationParams.TIME) - 1;

%-------------------------------------------------
% 2. Multi-Agent Formation Parameters
%-------------------------------------------------
Framework = struct();
Framework.SPACE_DIM          = 3;    % Number of spatial dimensions (3D)
Framework.NUM_AGENTS         = 3;    % Number of agents in the formation
Framework.NUM_EDGES          = 3;    % Number of edges in the formation graph
Framework.STATES_PER_VEHICLE = 12;   % Number of states per vehicle [η₁; η₂; ν₁; ν₂]
Framework.LEADER_AGENT       = 3;    % Index of the leader agent
Framework.distance           = zeros(Framework.NUM_EDGES, 1); % Desired inter-agent distances

%-------------------------------------------------
% 3. Prescribed Performance Control (PPC) Parameters
%-------------------------------------------------
PPControl = struct();
% PPControl.K_V       = 1.5;        % Gain for virtual control (velocity tracking)
% PPControl.K_TANG    = 2;          % Gain for hyperbolic tangent term (robustness)
% PPControl.K_SIGMA   = 48750;      % Gain for sigma term (damping)

PPControl.K_V       = 1.5;      % Gain for virtual control (velocity tracking)
PPControl.K_TANG    = 2;        % Gain for hyperbolic tangent term (robustness)
PPControl.K_SIGMA   = 72750;    % Gain for sigma term (damping)

% PPControl.K_V       = 3;        % Gain for virtual control (velocity tracking)
% PPControl.K_TANG    = 11900;    % Gain for hyperbolic tangent term (robustness)
% PPControl.K_SIGMA   = 72750;    % Gain for sigma term (damping)

%-------------------------------------------------
% 4. PID Controller Gains (for orientation control)
%-------------------------------------------------
PidControl = struct();
PidControl.KP   = 3*100;   % Proportional gain
PidControl.KD   = 60;      % Derivative gain
PidControl.KI   = 40;      % Integral gain

%-------------------------------------------------
% 5. Prescribed Performance Function (PPF) Parameters
%-------------------------------------------------
PPF = struct();
PPF.DELTA_UPPER_LIMIT = 4;      % Upper bound for performance envelope
PPF.DELTA_LOWER_LIMIT = 4;      % Lower bound for performance envelope
PPF.PPF_START         = 1;      % Initial value of the performance function
PPF.PPF_END           = 0.07;   % Final value of the performance function
PPF.BETA              = 0.9;    % Exponential decay rate for the performance function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parámetros de la trayectoria
R     = 5;           % Radio en metros
omega = 0.15;        % Velocidad angular (rad/s)
xc    = 2.35;         % Centro en X
yc    = 12.2;        % Centro en Y
zc    = -0.2;           % Altura inicial
phi0  = 0.0;         % Fase inicial
vz    = 0.2;           % Velocidad vertical constante en m/s

% Ecuaciones de la trayectoria deseada (numéricas, sin variables simbólicas)
% Parámetros:
%   R     = 5
%   omega = 0.15
%   xc    = 2.35
%   yc    = 12.2
%   zc    = -0.2
%   phi0  = 0.0
%   vz    = 0.2


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prescribed Performance Function (PPF) Calculation and Agent Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%-----------------------------------------------------
% 1. Prescribed Performance Function (PPF) Calculation
%-----------------------------------------------------
PPF.PPF = (PPF.PPF_START - PPF.PPF_END).*exp(-PPF.BETA * SimulationParams.TIME) + PPF.PPF_END;
PPF.PPF_DOT = -PPF.BETA * (PPF.PPF - PPF.PPF_END);

% Pre-allocate arrays for the upper and lower bounds of the PPF for each edge.
PPF.b_plus  = zeros(Framework.NUM_EDGES, 1);
PPF.b_minus = zeros(Framework.NUM_EDGES, 1);

%---------------------------------
% 2. Agent Initial Position Setup
%---------------------------------
AgentSetup = struct();
AgentSetup.AGENT_1_POS_M = 3 * [1.5579; 1.2313; 0.06123]; % Agent 1 initial position
AgentSetup.AGENT_2_POS_M = 3 * [3.5815; 2.8371; 0.01681]; % Agent 2 initial position
AgentSetup.AGENT_3_POS_M = 3 * [3.0201; 4.3460; 0.0]; % Agent 3 initial position

AgentSetup.positionsArray_M = [ ...
    AgentSetup.AGENT_1_POS_M; ...
    AgentSetup.AGENT_2_POS_M; ...
    AgentSetup.AGENT_3_POS_M ...
];

%-------------------------------------------
% 3. Agent Trajectory Buffer Initialization
%-------------------------------------------
AgentSetup.agentTrajectoryBuffer_M = zeros(totalIterations, Framework.NUM_AGENTS, Framework.SPACE_DIM);

%-------------------------------------------
% 4. Desired Target Positions for Each Agent
%-------------------------------------------
DesiredTargets = struct();
DesiredTargets.AGENT_1_POS_M = 3 * [1, 1, -1];
DesiredTargets.AGENT_2_POS_M = 3 * [3, 1, -1];
DesiredTargets.AGENT_3_POS_M = 3 * [2, 3, -1];

DesiredTargets.agentPositionsArray_M = [ ...
    DesiredTargets.AGENT_1_POS_M'; ...
    DesiredTargets.AGENT_2_POS_M'; ...
    DesiredTargets.AGENT_3_POS_M' ...
];

% Plot initial positions
figure(1);
for i = 1:Framework.NUM_AGENTS
    index_qx = (i - 1) * Framework.SPACE_DIM + 1;
    index_qy = (i - 1) * Framework.SPACE_DIM + 2;
    index_qz = (i - 1) * Framework.SPACE_DIM + 3;
    plot3(AgentSetup.positionsArray_M(index_qx), ...
          AgentSetup.positionsArray_M(index_qy), ...
          AgentSetup.positionsArray_M(index_qz), ...
          'x', 'LineWidth', 2, 'MarkerSize', 15);
    hold on;
end

% Define the edges of the formation
EdgesFormation = struct();
EdgesFormation.E1       = [1, 2];
EdgesFormation.E2       = [2, 3];
EdgesFormation.E3       = [3, 1];
EdgesFormation.E_ARRAY  = [EdgesFormation.E1; EdgesFormation.E2; EdgesFormation.E3];

% Desired inter-agent distances
for i = 1:Framework.NUM_EDGES
    idx_agent1 = (EdgesFormation.E_ARRAY(i, 1) - 1) * Framework.SPACE_DIM + 1;
    idx_agent2 = (EdgesFormation.E_ARRAY(i, 2) - 1) * Framework.SPACE_DIM + 1;
    Framework.distance(i, 1) = norm(DesiredTargets.agentPositionsArray_M(idx_agent1:idx_agent1 + 2) ...
                                  - DesiredTargets.agentPositionsArray_M(idx_agent2:idx_agent2 + 2));
end

% b_plus, b_minus from initial PPF
for i = 1:Framework.NUM_EDGES
    di = Framework.distance(i,1);
    PPF.b_plus(i)  = (PPF.DELTA_UPPER_LIMIT^2 + 2*di*PPF.DELTA_UPPER_LIMIT) / PPF.PPF(1);
    PPF.b_minus(i) = (2*di*PPF.DELTA_LOWER_LIMIT - PPF.DELTA_LOWER_LIMIT^2) / PPF.PPF(1);
end

% Buffers for errors
ErrorSignals = struct();
ErrorSignals.e        = zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.e_plus   = zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.e_minus  = zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.eta_plus = zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.eta_minus= zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.etaij    = zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.gammaV   = zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.zeta     = zeros(Framework.NUM_EDGES, totalIterations);
ErrorSignals.epsilon  = zeros(Framework.NUM_EDGES, totalIterations);

% PPC time-series data
PrescribedPerformanceControl = struct(); %#ok<NASGU>
PPC_Signals.virtualControlDot_V       = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, totalIterations);
PPC_Signals.virtualControl_V          = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, totalIterations);
PPC_Signals.velocityError_MPS         = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, totalIterations);
PPC_Signals.controlInput_V            = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, totalIterations);
PPC_Signals.velocityErrorSigma_MPS    = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, totalIterations);
PPC_Signals.qTildeDistanceError_M     = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, totalIterations);
PPC_Signals.qTinDistanceError_M       = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, 1);  % updated every step
% Nota: este es el nombre coherente usado en el lazo
PPC_Signals.desiredVelocity_MPS       = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, totalIterations);

% Rotation matrices (prev step) initialized at identidad (evita pico en dR/dt en t=1)
prevAgentRotMatrices = repmat(eye(3), 1, 1, Framework.NUM_AGENTS);
prevAgentRotMatrices = permute(prevAgentRotMatrices, [3 1 2]); % (N x 3 x 3)

% Inicialización de q~ para t=1
for j = 1:Framework.NUM_EDGES
    index_qj = EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM;
    index_qi = EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM;
    index_qij = j * Framework.SPACE_DIM - (Framework.SPACE_DIM - 1) : Framework.SPACE_DIM * j;
    PPC_Signals.qTildeDistanceError_M(index_qij, 1) = ...
        AgentSetup.positionsArray_M(index_qi, 1) - AgentSetup.positionsArray_M(index_qj, 1);
end
PPC_Signals.qTinDistanceError_M = qtinVector(EdgesFormation.E_ARRAY, ...
                                PPC_Signals.qTildeDistanceError_M(:, 1), ...
                                Framework.LEADER_AGENT, Framework.NUM_EDGES, Framework.SPACE_DIM);

% Initial velocities
initialLinearvelocityArray_MPS = [ ...
    2.2786; 0.7190; 2.8849; ...
    0.7418; 1.0539; 1.0164; ...
    0.5796; 0.7611; 2.2201 ];

initialAngularVelocity_RADPS = [ ...
    0.7099; 1.0521; 1.4225; ...
    1.1179; 0.2414; 0.5600; ...
    1.2600; 0.3263; 1.2705 ];

AgentSetup.initialLinearvelocityArray_MPS = initialLinearvelocityArray_MPS;
AgentSetup.initialAngularVelocity_RADPS   = initialAngularVelocity_RADPS;

% Preasignar velocidad de todos los agentes a lo largo del horizonte (consistencia dimensional)
AgentSetup.velocityArray_MPS = zeros(Framework.SPACE_DIM * Framework.NUM_AGENTS, numel(SimulationParams.TIME));
AgentSetup.velocityArray_MPS(:,1) = AgentSetup.initialLinearvelocityArray_MPS;

% Desired trajectory functions (líder)
TrajectoryDefVelocity = struct();

% Posición deseada
pos_x_desired = @(t) xc + R*cos(omega*t + phi0);
pos_y_desired = @(t) yc + R*sin(omega*t + phi0);
pos_z_desired = @(t) zc + vz*t;

% Velocidad deseada
vel_x_desired = @(t) -R*omega*sin(omega*t);
vel_y_desired = @(t)  R*omega*cos(omega*t);
vel_z_desired = @(t)  vz*ones(size(t));

TrajectoryDefVelocity.eta1_desired_func = @(t) [pos_x_desired(t); pos_y_desired(t); pos_z_desired(t)];
TrajectoryDefVelocity.nu1_desired_func  = @(t) [vel_x_desired(t); vel_y_desired(t); vel_z_desired(t)];

% Orientaciones deseadas (cuerpo) y sus derivadas
TrajectoryDefVelocity.eta2_desired_func = @(t) [zeros(1,length(t)); zeros(1,length(t)); zeros(1,length(t))];
TrajectoryDefVelocity.nu2_desired_func  = @(t) [zeros(1,length(t)); zeros(1,length(t)); zeros(1,length(t))];

% Vectores de referencia en todo el horizonte
TrajectoryDefVelocity.eta1_desired_vector = TrajectoryDefVelocity.eta1_desired_func(SimulationParams.TIME);
TrajectoryDefVelocity.nu1_desired_vector  = TrajectoryDefVelocity.nu1_desired_func(SimulationParams.TIME);
TrajectoryDefVelocity.eta2_desired_vector = TrajectoryDefVelocity.eta2_desired_func(SimulationParams.TIME);
TrajectoryDefVelocity.nu2_desired_vector  = TrajectoryDefVelocity.nu2_desired_func(SimulationParams.TIME);

% Aliases solicitados en el documento
phi_desired       = @(t) zeros(1, length(t));
theta_desired     = @(t) zeros(1, length(t));
psi_desired       = @(t) zeros(1, length(t));
phi_dot_desired   = @(t) zeros(1, length(t));
theta_dot_desired = @(t) zeros(1, length(t));
psi_dot_desired   = @(t) zeros(1, length(t));

TrajectoryDefVelocity.eta2_desired = @(t) [phi_desired(t); theta_desired(t); psi_desired(t)];
TrajectoryDefVelocity.nu2_desired  = @(t) [phi_dot_desired(t); theta_dot_desired(t); psi_dot_desired(t)];

% PID controllers
PID_gains = struct('Kp', [], 'Ki', [], 'Kd', []);
PID_gains(1).Kp = [PidControl.KP, PidControl.KP, PidControl.KP];
PID_gains(1).Ki = [PidControl.KD, PidControl.KD, PidControl.KD+15];
PID_gains(1).Kd = [PidControl.KI, PidControl.KI, PidControl.KI];

PID_gains(2) = PID_gains(1);
PID_gains(3) = PID_gains(1);

PID_controllers = struct();
for i = 1:Framework.NUM_AGENTS
    PID_controllers(i).PID = PIDController(PID_gains(1).Kp, PID_gains(1).Ki, PID_gains(1).Kd);
end

% Estructura de vehículos
vehicle = struct();
for i = 1:Framework.NUM_AGENTS
    vehicle(i).eta1    = zeros(Framework.SPACE_DIM, numel(SimulationParams.TIME));
    vehicle(i).eta2    = zeros(Framework.SPACE_DIM, numel(SimulationParams.TIME));
    vehicle(i).nu1     = zeros(Framework.SPACE_DIM, numel(SimulationParams.TIME));
    vehicle(i).nu2     = zeros(Framework.SPACE_DIM, numel(SimulationParams.TIME));
    vehicle(i).u1      = zeros(Framework.SPACE_DIM, totalIterations);
    vehicle(i).u2      = zeros(Framework.SPACE_DIM, totalIterations);
    vehicle(i).eta     = zeros(6, numel(SimulationParams.TIME));
    vehicle(i).nu      = zeros(6, numel(SimulationParams.TIME));
    vehicle(i).error   = zeros(Framework.SPACE_DIM, totalIterations);
    vehicle(i).eta2error_i = zeros(Framework.SPACE_DIM, totalIterations);
    vehicle(i).eta2error_v = zeros(Framework.SPACE_DIM, totalIterations);
    vehicle(i).phi     = zeros(1, totalIterations);
    vehicle(i).theta   = zeros(1, totalIterations);
    vehicle(i).psi     = zeros(1, totalIterations);
end

% Set inicial de estados traslacionales
for i = 1:Framework.NUM_AGENTS
    base_idx = (i - 1) * Framework.SPACE_DIM;
    vehicle(i).eta1(:, 1) = AgentSetup.positionsArray_M(base_idx + 1:base_idx + Framework.SPACE_DIM, 1);
    vehicle(i).nu1(:, 1)  = AgentSetup.initialLinearvelocityArray_MPS(base_idx + 1:base_idx + Framework.SPACE_DIM, 1);
end

% Estado completo X
X = zeros(Framework.NUM_AGENTS * Framework.STATES_PER_VEHICLE, numel(SimulationParams.TIME));
for i = 1:Framework.NUM_AGENTS
    base_idx = (i - 1) * Framework.STATES_PER_VEHICLE;
    X(base_idx + 1:base_idx + Framework.STATES_PER_VEHICLE, 1) = [ ...
        vehicle(i).eta1(:, 1); ...
        vehicle(i).eta2(:, 1); ...
        vehicle(i).nu1(:, 1);  ...
        vehicle(i).nu2(:, 1) ];
end

% Parámetros del modelo (inicial)
for i = 1:Framework.NUM_AGENTS
    % modelParameters_Bluerov2(i) = BlueROV2ModelParameters(vehicle(i).nu(:, 1), vehicle(i).eta(:, 1)); %#ok<SAGROW>
    modelParameters_Bluerov2(i) = BlueROV2ModelMatrices(vehicle(i).nu(:, 1), vehicle(i).eta(:, 1), getBaseParams(defaultBlueROV2Params(), i));
end

% Preasignar u (evita growth dinámico)
u = zeros(6*Framework.NUM_AGENTS, totalIterations);

% Main Loop
hWait = waitbar(0, 'Processing...');

EPS_GAMMA = 1e-12; % salvaguarda numérica ligera para log/raíces

for time_iterator = 1:totalIterations
    if mod(time_iterator, max(1, floor(totalIterations/100))) == 0
        waitbar(time_iterator/totalIterations, hWait, ...
            sprintf('Progress: %.2f%%', (time_iterator/totalIterations)*100));
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 1. Compute Edge Errors and Prescribed Performance Variables
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:Framework.NUM_EDGES
        idx_agent_i = EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 2) * Framework.SPACE_DIM;
        idx_agent_j = EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM - 2 : EdgesFormation.E_ARRAY(j, 1) * Framework.SPACE_DIM;
        idx_edge    = j * Framework.SPACE_DIM - (Framework.SPACE_DIM - 1) : Framework.SPACE_DIM * j;

        PPC_Signals.qTildeDistanceError_M(idx_edge, time_iterator) = ...
            AgentSetup.positionsArray_M(idx_agent_i, time_iterator) - ...
            AgentSetup.positionsArray_M(idx_agent_j, time_iterator);

        dij = Framework.distance(j);
        eij = norm(PPC_Signals.qTildeDistanceError_M(idx_edge, time_iterator)) - dij;
        ErrorSignals.e(j, time_iterator) = eij;
        ErrorSignals.epsilon(j, time_iterator) = eij * (eij + 2*dij);

        % Acota el argumento de la raíz para robustez numérica
        bP = PPF.b_plus(j);  bM = PPF.b_minus(j);
        pp = PPF.PPF(time_iterator);

        arg_plus  = max(dij^2 + bP*pp, 0);
        arg_minus = max(dij^2 - bM*pp, 0);
        ErrorSignals.e_plus(j, time_iterator)  = -dij + sqrt(arg_plus);
        ErrorSignals.e_minus(j, time_iterator) =  dij - sqrt(arg_minus);

        ErrorSignals.eta_plus(j, time_iterator)  = ErrorSignals.e_plus(j, time_iterator)^2  + 2*dij*ErrorSignals.e_plus(j, time_iterator);
        ErrorSignals.eta_minus(j, time_iterator) = ErrorSignals.e_minus(j, time_iterator)^2 - 2*dij*ErrorSignals.e_minus(j, time_iterator);

        zeta_ij = ErrorSignals.epsilon(j, time_iterator) / max(pp, EPS_GAMMA);
        ErrorSignals.zeta(j, time_iterator) = zeta_ij;

        % Transformación gammaV con salvaguarda
        num_g = bP*zeta_ij + bM*bP;
        den_g = bP*bM       - bM*zeta_ij;
        num_g = sign(num_g)*max(abs(num_g), EPS_GAMMA);
        den_g = sign(den_g)*max(abs(den_g), EPS_GAMMA);
        ErrorSignals.gammaV(j, time_iterator) = 0.5 * log(num_g/den_g);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2. Compute Transformed Distance Errors and Desired Velocities
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PPC_Signals.qTinDistanceError_M = qtinVector(EdgesFormation.E_ARRAY, ...
                                PPC_Signals.qTildeDistanceError_M(:, time_iterator), ...
                                Framework.LEADER_AGENT, Framework.NUM_EDGES, Framework.SPACE_DIM);

    % Vectorización del cálculo de v_deseada para todos los agentes
    vL = TrajectoryDefVelocity.nu1_desired_vector(:, time_iterator);   % 3x1
    wL = TrajectoryDefVelocity.nu2_desired_vector(:, time_iterator);   % 3x1
    qTin = reshape(PPC_Signals.qTinDistanceError_M, 3, []);            % 3xN
    cr  = cross(repmat(wL,1,Framework.NUM_AGENTS), qTin, 1);           % 3xN
    vdes = (repmat(vL,1,Framework.NUM_AGENTS) + cr);                   % 3xN
    PPC_Signals.desiredVelocity_MPS(:, time_iterator) = vdes(:);
    PPC_Signals.desiredvelocityArray_MPS(:, time_iterator) = vdes(:);  % alias

    % Matriz R basada en posiciones actuales
    Rmat = matrizRTriangle3AgentWithLeader(AgentSetup.positionsArray_M(:, time_iterator), Framework.SPACE_DIM);

    % eta_ij y su matriz diagonal
    for j = 1:Framework.NUM_EDGES
        ErrorSignals.etaij(j, time_iterator) = (1 / max(PPF.PPF(time_iterator), EPS_GAMMA)) * ...
            ( 1 / (ErrorSignals.zeta(j, time_iterator) + PPF.b_minus(j)) ...
            - 1 / (ErrorSignals.zeta(j, time_iterator) - PPF.b_plus(j)) );
    end
    eta = diag(ErrorSignals.etaij(:, time_iterator));

    % Derivada discreta de eta (rhop) — mantenida por estructura
    if time_iterator == 1
        rhop     = eta / SimulationParams.TIME_STEP; %#ok<NASGU>
        temp_eta = eta;
    else
        rhop     = (eta - temp_eta) / SimulationParams.TIME_STEP; %#ok<NASGU>
        temp_eta = eta;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 2.1 Virtual Control and Velocity Error Calculation (PPC)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PPC_Signals.virtualControl_V(:, time_iterator) = ...
        -PPControl.K_V * Rmat' * eta' * ErrorSignals.gammaV(:, time_iterator) + ...
            + PPC_Signals.desiredVelocity_MPS(:, time_iterator);
        % + 0.5 * Rmat' *  eta' * PPF.PPF_DOT(time_iterator) * ErrorSignals.zeta(:, time_iterator) ...

    PPC_Signals.velocityErrorSigma_MPS(:, time_iterator) = ...
        AgentSetup.velocityArray_MPS(:, time_iterator) - PPC_Signals.virtualControl_V(:, time_iterator);

    if time_iterator == 1
        PPC_Signals.virtualControlDot_V(:, time_iterator) = ...
            PPC_Signals.virtualControl_V(:, time_iterator) / SimulationParams.TIME_STEP;
    else
        PPC_Signals.virtualControlDot_V(:, time_iterator) = ...
            (PPC_Signals.virtualControl_V(:, time_iterator) - PPC_Signals.virtualControl_V(:, time_iterator - 1)) ...
            / SimulationParams.TIME_STEP;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic Model Matrices Initialization for Each Agent
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    GAMMA_MATRIX = [];
    Ceta1V = [];
    Deta1V = [];
    geta1V = [];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 3. Dynamic Model and Error Calculation for Each Vehicle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:Framework.NUM_AGENTS
        % (a) Error de orientación
        vehicle(j).eta2error(:, time_iterator) = ...
            TrajectoryDefVelocity.eta2_desired(SimulationParams.TIME(time_iterator)) - ...
            vehicle(j).eta2(:, time_iterator);

        % (b) Rotación cuerpo->inercial
        vehicleRotationMatrix = rotationMatrix( ...
            vehicle(j).eta2(1, time_iterator), ...
            vehicle(j).eta2(2, time_iterator), ...
            vehicle(j).eta2(3, time_iterator) );

        % (c) Actualizar parámetros del modelo
        modelParameters_Bluerov2(j) = BlueROV2ModelMatrices(vehicle(i).nu(:, 1), vehicle(i).eta(:, 1), getBaseParams(defaultBlueROV2Params(), i));

        % (d) Inercia y su inversa (estable numéricamente con “\”)
        M = [modelParameters_Bluerov2(j).M11, modelParameters_Bluerov2(j).M12; ...
             modelParameters_Bluerov2(j).M21, modelParameters_Bluerov2(j).M22];
        Minv = M \ eye(6); % evita inv(M)
        inv_M11 = Minv(1:3, 1:3);
        inv_M12 = Minv(1:3, 4:6);
        % inv_M21 = Minv(4:6, 1:3); % no usado
        % inv_M22 = Minv(4:6, 4:6); % no usado

        % (e) GAMMA = blkdiag(R*M11)
        rotation_M11_matrix_product = vehicleRotationMatrix * modelParameters_Bluerov2(j).M11;
        GAMMA_MATRIX = blkdiag(GAMMA_MATRIX, rotation_M11_matrix_product);

        % (f) Dinámica en cuerpo
        Cnu1 = -inv_M11 * modelParameters_Bluerov2(j).C11 - inv_M12 * modelParameters_Bluerov2(j).C21;
        Dnu1 = -inv_M11 * modelParameters_Bluerov2(j).D11;
        gnu1 = -inv_M11 * modelParameters_Bluerov2(j).g1;

        % (g) dR/dt (consistente en t=1 por inicialización previa)
        Rprev = squeeze(prevAgentRotMatrices(j, :, :));
        diff_RotationMatrix = (vehicleRotationMatrix - Rprev) / SimulationParams.TIME_STEP;
        prevAgentRotMatrices(j, :, :) = vehicleRotationMatrix;

        % (h) Transformación a inercial
        Ceta1i = vehicleRotationMatrix * Cnu1 / vehicleRotationMatrix + diff_RotationMatrix / vehicleRotationMatrix;
        Deta1i = vehicleRotationMatrix * Dnu1 / vehicleRotationMatrix;
        geta1i = vehicleRotationMatrix * gnu1;

        % Derivada de nu1 (discreta)
        if time_iterator == 1
            nu1dot = zeros(Framework.SPACE_DIM, 1);
        else
            nu1dot = (vehicle(j).nu1(:, time_iterator) - vehicle(j).nu1(:, time_iterator - 1)) / SimulationParams.TIME_STEP;
        end

        % (i) Apilados
        Ceta1V = [Ceta1V; Ceta1i * nu1dot];
        Deta1V = [Deta1V; Deta1i * nu1dot];
        geta1V = [geta1V; geta1i];

        % (j) Integradores de PID
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

    % (a) OMEGA
    OMEGA = Ceta1V + Deta1V + geta1V;

    % (b) u1 PPC (resolver con “\” en lugar de inv(GAMMA))
    PPC_Signals.controlInput_V(:, time_iterator) = GAMMA_MATRIX \ ( ...
          PPC_Signals.virtualControlDot_V(:, time_iterator) ...
        - Rmat' * eta' * ErrorSignals.gammaV(:, time_iterator) ...
        - OMEGA ...
        - (PPControl.K_SIGMA) * PPC_Signals.velocityErrorSigma_MPS(:, time_iterator) ...
        - PPControl.K_TANG * tanh(PPC_Signals.velocityErrorSigma_MPS(:, time_iterator)) );

    % (c) PID de orientación
    for j = 1:Framework.NUM_AGENTS
        vehicle(j).u2(:, time_iterator) = PID_controllers(j).PID.computeControl( ...
            vehicle(j).eta2error(:, time_iterator), ...
            vehicle(j).eta2error_i(:, time_iterator), ...
            vehicle(j).eta2error_v(:, time_iterator) );
    end

    % (d) Asignación u1 por agente
    for j = 1:Framework.NUM_AGENTS
        index_u1 = j * Framework.SPACE_DIM - 2 : j * Framework.SPACE_DIM;
        vehicle(j).u1(:, time_iterator) = PPC_Signals.controlInput_V(index_u1, time_iterator);
    end

    % (e) u = [u1; u2] apilado
    for j = 1:Framework.NUM_AGENTS
        base_u = (j - 1) * 6;
        index_u = base_u + 1 : base_u + 6;
        u(index_u, time_iterator) = [vehicle(j).u1(:, time_iterator); vehicle(j).u2(:, time_iterator)];
    end

    % (f) Integración RK4
    X(:, time_iterator + 1) = RK4step_new( ...
        @multiagent_underwater_model_bluerov2Params_2, ...
        SimulationParams.TIME(time_iterator), ...
        X(:, time_iterator), ...
        SimulationParams.TIME_STEP, ...
        u(:, time_iterator), ...
        Framework.SPACE_DIM, ...
        Framework.NUM_AGENTS, ...
        modelParameters_Bluerov2(:) );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update States for Each Vehicle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:Framework.NUM_AGENTS
        start_idx = (j - 1) * Framework.STATES_PER_VEHICLE + 1;
        base_idx  = (j - 1) * Framework.SPACE_DIM;

        vehicle(j).eta1(:, time_iterator + 1) = X(start_idx:start_idx + 2, time_iterator + 1);
        vehicle(j).eta2(:, time_iterator + 1) = X(start_idx + 3:start_idx + 5, time_iterator + 1);
        vehicle(j).nu1(:, time_iterator + 1)  = X(start_idx + 6:start_idx + 8, time_iterator + 1);
        vehicle(j).nu2(:, time_iterator + 1)  = X(start_idx + 9:start_idx + 11, time_iterator + 1);

        % Disturbance (ruido)
        if SimulationParams.noise ~= 0
            vehicle(j).eta1(:, time_iterator + 1) = vehicle(j).eta1(:, time_iterator + 1) + SimulationParams.noise*rand(3,1);
            vehicle(j).eta2(:, time_iterator + 1) = vehicle(j).eta2(:, time_iterator + 1) + SimulationParams.noise*rand(3,1);
            vehicle(j).nu1(:, time_iterator + 1)  = vehicle(j).nu1(:, time_iterator + 1)  + SimulationParams.noise*rand(3,1);
            vehicle(j).nu2(:, time_iterator + 1)  = vehicle(j).nu2(:, time_iterator + 1)  + SimulationParams.noise*rand(3,1);
        end

        vehicle(j).eta(:, time_iterator + 1) = X(start_idx:start_idx + 5, time_iterator + 1);
        vehicle(j).nu(:, time_iterator + 1)  = X(start_idx + 6:start_idx + 11, time_iterator + 1);

        AgentSetup.positionsArray_M(base_idx + 1:base_idx + Framework.SPACE_DIM, time_iterator + 1) = vehicle(j).eta1(:, time_iterator + 1);
        AgentSetup.velocityArray_MPS(base_idx + 1:base_idx + Framework.SPACE_DIM, time_iterator + 1) = vehicle(j).nu1(:, time_iterator + 1);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update Agent Trajectories and Orientation Angles
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j = 1:Framework.NUM_AGENTS
        AgentSetup.agentTrajectoryBuffer_M(time_iterator, j, :) = [ ...
            AgentSetup.positionsArray_M(j * Framework.SPACE_DIM - 2, time_iterator+1), ...
            AgentSetup.positionsArray_M(j * Framework.SPACE_DIM - 1, time_iterator+1), ...
            AgentSetup.positionsArray_M(j * Framework.SPACE_DIM,     time_iterator+1) ];
    end

    for j = 1:Framework.NUM_AGENTS
        vehicle(j).phi(time_iterator)   = vehicle(j).eta2(1, time_iterator + 1);
        vehicle(j).theta(time_iterator) = vehicle(j).eta2(2, time_iterator + 1);
        vehicle(j).psi(time_iterator)   = vehicle(j).eta2(3, time_iterator + 1);
    end
end

close(hWait);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Animation and Trajectory Plotting Section
%% Animation and Trajectory Plotting Section
%% Animation and Trajectory Plotting Section
%% Animation and Trajectory Plotting Section
%% Animation and Trajectory Plotting Section
%% Animation and Trajectory Plotting Section
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
    f = figure(1);

    plotAnimation3Agents_Optimized( ...
        AgentSetup.agentTrajectoryBuffer_M, ...
        TrajectoryDefVelocity.eta1_desired_vector, ...
        SimulationParams.TIME, ...
        AgentSetup.positionsArray_M, ...
        EdgesFormation.E_ARRAY, ...
        f ...
    );
else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Static 3D Trajectory Plotting
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    plotLeaderAndAgentTrajectory3D(TrajectoryDefVelocity, AgentSetup, Framework, EdgesFormation);
end


% Plot the prescribed performance bounds and error signals for each edge
plotPrescribedPerformanceBounds(ErrorSignals, EdgesFormation, Framework, SimulationParams);

% Plot the initial formation shape based on the desired agent positions
plotInitialFormationShape(DesiredTargets.agentPositionsArray_M(:, 1), EdgesFormation.E_ARRAY);

% Plot the orientation tracking performance for each agent
plotOrientationTracking(X, TrajectoryDefVelocity, Framework, SimulationParams);

% Plot the control inputs (u) applied to each agent over time
plotControlInputs(u, SimulationParams, Framework);

% Plot the position errors of agent 3 compared to the leader's desired trajectory
plotPositionErrors(3, AgentSetup.positionsArray_M, TrajectoryDefVelocity.eta1_desired_vector, Framework.SPACE_DIM, SimulationParams.TIME);

% Plot the position of each agent versus the leader's position over time
plotAgentVsLeaderPosition( TrajectoryDefVelocity.eta1_desired_vector, AgentSetup.positionsArray_M, Framework.SPACE_DIM, SimulationParams.TIME);


[avgFormErr, avgTrackErr, totCtrlEff, avgCtrlEff] = calculatePerformanceMetrics(SimulationParams.TIME(1:end-1), ...
                                                                                ErrorSignals.e, ...
                                                                                AgentSetup.positionsArray_M, ...
                                                                                TrajectoryDefVelocity.eta1_desired_vector, ...
                                                                                u(1:9,:), ...
                                                                                SimulationParams.TIME_STEP, ...
                                                                                Framework.SPACE_DIM, ...
                                                                                Framework.NUM_AGENTS);                                                                                

[ITAE_formation, ITAE_tracking, ControlEnergy] = calculateAdditionalMetrics(SimulationParams.TIME(1:end-1), ...
                                                                            ErrorSignals.e,...
                                                                            AgentSetup.positionsArray_M, ...
                                                                            TrajectoryDefVelocity.eta1_desired_vector, ...
                                                                            u(1:9,:), ...
                                                                            SimulationParams.TIME_STEP, ...
                                                                            Framework.SPACE_DIM, ...
                                                                            Framework.NUM_AGENTS);

disp("Prescribed Performance Control Metrics:")
disp("ITAE (Formation): " + ITAE_formation)
disp("ITAE (Tracking):  " + ITAE_tracking)
disp("Total Control Energy: " + ControlEnergy)


if mod(time_iterator, 200)==0
    fprintf('t=%.2f | ||vDot||=%.2f  ||rigidez||=%.2f  ||OMEGA||=%.2f  ||sigma||=%.2f  ||u1||=%.2f\n', ...
        SimulationParams.TIME(time_iterator), ...
        norm(PPC_Signals.virtualControlDot_V(:, time_iterator)), ...
        norm(R' * eta' * ErrorSignals.gammaV(:, time_iterator)), ...
        norm(OMEGA), ...
        norm(PPC_Signals.velocityErrorSigma_MPS(:, time_iterator)), ...
        norm(PPC_Signals.controlInput_V(:, time_iterator)));
end


toc






% Plotting
% Average Formation Error: 0.030396
% Average Trajectory Tracking Error: 5.992917
% Total Control Effort: 3352949818.636063
% Average Control Input Smoothness (Total Variation) per Agent: 38489139.215204
% Prescribed Performance Control Metrics:
% ITAE (Formation): 0.76506
% ITAE (Tracking):  7981.2303
% Total Control Energy: 3352949818.6361
% t=49.99 | ||vDot||=0.19  ||rigidez||=0.00  ||OMEGA||=346.72  ||sigma||=2.43  ||u1||=9692.04
% Elapsed time is 33.287676 seconds.