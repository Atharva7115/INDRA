function result = mpc_controller( ...
    ego_speed, ...
    ego_y, ...
    ego_heading, ...
    ref_y, ...
    ref_heading, ...
    ref_speed)

% ============================================================
% INDRA - MPC VEHICLE CONTROLLER FINAL
% Adaptive Closed-Loop Vehicle Control
%
% Linearized kinematic bicycle MPC
%
% States:
%   y       - lateral position (m)
%   heading - vehicle heading (rad)
%   speed   - vehicle speed (m/s)
%
% Inputs:
%   steering     - steering angle (rad)
%   acceleration - longitudinal acceleration (m/s^2)
% ============================================================

%% PARAMETERS

Ts = 0.1;
wheelbase = 2.5;
nominal_speed = 10.0;

% Keep these in function scope because they are needed on
% every call, not only when the persistent MPC is created.
prediction_horizon = 20;
control_horizon = 6;

persistent mpc_obj
persistent mpc_state

%% CREATE MPC ON FIRST CALL

if isempty(mpc_obj)

    A = [1, nominal_speed*Ts, 0;
         0, 1,                0;
         0, 0,                1];

    B = [0,                          0;
         nominal_speed/wheelbase*Ts, 0;
         0,                          Ts];

    C = eye(3);
    D = zeros(3,2);

    plant = ss(A,B,C,D,Ts);

    warning_state = warning;
    warning('off','all');

    mpc_obj = mpc( ...
        plant, ...
        Ts, ...
        prediction_horizon, ...
        control_horizon);

    warning(warning_state);

    % Tracking priorities:
    % lateral position > heading > speed
    mpc_obj.Weights.OutputVariables = [12 10 0.5];

    % Control effort penalties
    mpc_obj.Weights.ManipulatedVariables = [0.10 0.05];

    % Steering-rate penalty for smoother steering
    mpc_obj.Weights.ManipulatedVariablesRate = [2.0 0.25];

    % Steering limits
    mpc_obj.MV(1).Min = deg2rad(-30);
    mpc_obj.MV(1).Max = deg2rad(30);

    % Acceleration limits
    mpc_obj.MV(2).Min = -6;
    mpc_obj.MV(2).Max = 2;

    % Speed limits
    mpc_obj.OV(3).Min = 0;
    mpc_obj.OV(3).Max = 15;

    mpc_state = mpcstate(mpc_obj);
end

%% INPUT SANITIZATION

if ~isfinite(ego_speed)
    ego_speed = nominal_speed;
end

if ~isfinite(ego_y)
    ego_y = 0;
end

if ~isfinite(ego_heading)
    ego_heading = 0;
end

if ~isfinite(ref_y)
    ref_y = ego_y;
end

if ~isfinite(ref_heading)
    ref_heading = 0;
end

if ~isfinite(ref_speed)
    ref_speed = ego_speed;
end

%% CURRENT VEHICLE STATE

current_output = [ego_y, ego_heading, ego_speed];

%% REFERENCE OVER COMPLETE PREDICTION HORIZON

y_reference = repmat( ...
    [ref_y, ref_heading, ref_speed], ...
    prediction_horizon, ...
    1);

%% MPC CONTROL

try

    [mv, info] = mpcmove( ...
        mpc_obj, ...
        mpc_state, ...
        current_output, ...
        y_reference);

    steering = mv(1);
    acceleration = mv(2);

    % Hard safety constraints
    steering = max( ...
        deg2rad(-30), ...
        min(deg2rad(30), steering));

    acceleration = max( ...
        -6, ...
        min(2, acceleration));

    controller_status = "MPC ACTIVE";

catch ME

    % Fail-safe behavior
    steering = 0;
    acceleration = -6;

    controller_status = "MPC FAIL-SAFE";
    info = [];

    warning("MPC failed: %s", ME.message);
end

%% TRACKING ERRORS

lateral_error = ref_y - ego_y;
heading_error = ref_heading - ego_heading;
speed_error = ref_speed - ego_speed;

%% RESULT STRUCTURE

result = struct();

result.steering = steering;
result.acceleration = acceleration;
result.steering_deg = rad2deg(steering);

result.controller_status = controller_status;

result.reference_y = ref_y;
result.reference_heading = ref_heading;
result.reference_speed = ref_speed;

result.current_y = ego_y;
result.current_heading = ego_heading;
result.current_speed = ego_speed;

result.lateral_error = lateral_error;
result.heading_error = heading_error;
result.speed_error = speed_error;

result.prediction_horizon = prediction_horizon;
result.control_horizon = control_horizon;

result.mpc_info = info;

end
