function result = mpc_controller( ...
    ego_speed, ...
    ego_y, ...
    ego_heading, ...
    ref_y, ...
    ref_heading, ...
    ref_speed)

% ============================================================
% INDRA - MPC VEHICLE CONTROLLER
% Adaptive Closed-Loop Vehicle Control
%
% Inputs:
%   ego_speed    - current vehicle speed (m/s)
%   ego_y        - current lateral position (m)
%   ego_heading  - current heading (rad)
%   ref_y        - planner lateral reference (m)
%   ref_heading  - planner heading reference (rad)
%   ref_speed    - planner target speed (m/s)
%
% Outputs:
%   result.steering
%   result.acceleration
% ============================================================

Ts = 0.1;

% ============================================================
% PERSISTENT MPC OBJECT
% ============================================================
%
% The MPC object is created only once.
% Previously it was being recreated at every simulation step,
% which caused the repeated weight/model messages.
%
% ============================================================

persistent mpc_obj
persistent mpc_state

if isempty(mpc_obj)

    % --------------------------------------------------------
    % Linearized vehicle model
    %
    % States:
    %   x1 = lateral position
    %   x2 = heading
    %   x3 = speed
    %
    % Inputs:
    %   u1 = steering
    %   u2 = acceleration
    %
    % Outputs:
    %   y1 = lateral position
    %   y2 = heading
    %   y3 = speed
    % --------------------------------------------------------

    A = [1   Ts   0;
         0   1    0;
         0   0    1];

    B = [0    0;
         Ts   0;
         0    Ts];

    C = eye(3);

    D = zeros(3,2);

    plant = ss(A,B,C,D,Ts);

    % --------------------------------------------------------
    % MPC configuration
    % --------------------------------------------------------

    prediction_horizon = 15;
    control_horizon = 5;

    mpc_obj = mpc( ...
        plant, ...
        Ts, ...
        prediction_horizon, ...
        control_horizon);

    % --------------------------------------------------------
    % MPC weights
    % --------------------------------------------------------

    mpc_obj.Weights.OutputVariables = ...
        [5 3 1];

    mpc_obj.Weights.ManipulatedVariables = ...
        [0.1 0.05];

    mpc_obj.Weights.ManipulatedVariablesRate = ...
        [0.5 0.2];

    % --------------------------------------------------------
    % Steering limits
    % --------------------------------------------------------

    mpc_obj.MV(1).Min = deg2rad(-30);
    mpc_obj.MV(1).Max = deg2rad(30);

    % --------------------------------------------------------
    % Acceleration limits
    % --------------------------------------------------------

    mpc_obj.MV(2).Min = -6;
    mpc_obj.MV(2).Max = 2;

    % --------------------------------------------------------
    % Vehicle speed limits
    % --------------------------------------------------------

    mpc_obj.OV(3).Min = 0;
    mpc_obj.OV(3).Max = 15;

    % --------------------------------------------------------
    % Create MPC state only once
    % --------------------------------------------------------

    mpc_state = mpcstate(mpc_obj);

end


% ============================================================
% CURRENT VEHICLE OUTPUT
% ============================================================

current_output = [ ...
    ego_y;
    ego_heading;
    ego_speed];


% ============================================================
% REFERENCE TRAJECTORY
% ============================================================

% MPC prediction horizon = 15
%
% Each row:
%   [lateral position, heading, speed]
%
% For this MVP, the planner reference is held over the
% prediction horizon.
% ============================================================

y_reference = repmat( ...
    [ref_y ref_heading ref_speed], ...
    15, ...
    1);


% ============================================================
% MPC CONTROL CALCULATION
% ============================================================

try

    % --------------------------------------------------------
    % IMPORTANT:
    %
    % mpcmove returns:
    %
    %   [mv,info]
    %
    % NOT three output arguments.
    % --------------------------------------------------------

    [mv, info] = mpcmove( ...
        mpc_obj, ...
        mpc_state, ...
        current_output, ...
        y_reference);

    % --------------------------------------------------------
    % Extract control commands
    % --------------------------------------------------------

    steering = mv(1);

    acceleration = mv(2);

    % --------------------------------------------------------
    % Safety saturation
    % --------------------------------------------------------

    steering = max( ...
        deg2rad(-30), ...
        min(deg2rad(30), steering));

    acceleration = max( ...
        -6, ...
        min(2, acceleration));

    controller_status = "MPC ACTIVE";

catch ME

    % --------------------------------------------------------
    % Fail-safe behavior
    %
    % If MPC genuinely fails, stop the vehicle safely.
    % --------------------------------------------------------

    steering = 0;

    acceleration = -6;

    controller_status = "MPC FAIL-SAFE";

    info = [];

    warning( ...
        "MPC failed: %s", ...
        ME.message);

end


% ============================================================
% RESULT STRUCTURE
% ============================================================

result = struct();

% Control commands
result.steering = steering;
result.acceleration = acceleration;

% Steering in degrees for dashboard/plots
result.steering_deg = rad2deg(steering);

% Controller status
result.controller_status = controller_status;

% Planner reference
result.reference_y = ref_y;
result.reference_heading = ref_heading;
result.reference_speed = ref_speed;

% Current vehicle state
result.current_y = ego_y;
result.current_heading = ego_heading;
result.current_speed = ego_speed;

% MPC information
result.mpc_info = info;

end