function dxdt = multiagent_underwater_model_bluerov2Params(t, X, u, m, n, modelParameters)

    % %% Define parameters for both vehicles
    % params.M11 = modelParameters.M11;
    % params.M12 = modelParameters.M12;
    % params.M21 = modelParameters.M21;
    % params.M22 = modelParameters.M22;
    
    % params.C11 = modelParameters.C11;
    % params.C12 = modelParameters.C12;
    % params.C21 = modelParameters.C21;
    % params.C22 = modelParameters.C22;
    
    % params.D11 = modelParameters.D11;
    % params.D22 = modelParameters.D22;

    % params.g1 = modelParameters.g1;
    % params.g2 = modelParameters.g2;


    %% Initialize Vehicle Models
    vehicles = cell(1, n);
    for i = 1:n
        vehicles{i} = VehicleModel(modelParameters(i));
    end

    %% Initialize Results
    dxdt                    = []; 
    state_vars_per_agent    = 12;
    control_input_per_agent = 6; 

    % Loop for Each Vehicle
    for i = 1:n
        % Compute the starting indices for states and control inputs
        start_idx_state = (i - 1) * state_vars_per_agent;
        start_idx_control = (i - 1) * control_input_per_agent;

        % Extract state variables for this agent
        X_i = X(start_idx_state + 1 : start_idx_state + state_vars_per_agent);

        % Extract control inputs for this agent
        tau_i = u(start_idx_control + 1 : start_idx_control + control_input_per_agent);

        % Call the model function for this vehicle
        [eta_v, nu_v] = modelFunction(t, X_i, tau_i, vehicles{i});

        % Append the results to dxdt
        dxdt = [dxdt; eta_v(1:6); nu_v(1:6)];
    end
end
