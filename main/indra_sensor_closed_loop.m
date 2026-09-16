function indra_result = indra_sensor_closed_loop(scenario_input, sensor_params)
%% INDRA - SENSOR-DRIVEN CLOSED-LOOP AUTONOMY SIMULATION
%
% Architecture:
%   Scenario World (Ground Truth)
%         ↓
%   Synthetic Sensor Model (Radar: Range, Bearing, Range Rate)
%         ↓
%   Sensor -> EKF Adapter (Coordinate Transforms & Kalman Updates)
%         ↓
%   Estimated Object State [X, Y, Vx, Vy] (Autonomy Input)
%         ↓
%   Future CV Prediction
%         ↓
%   Uncertainty-Aware Risk Assessment
%         ↓
%   Critical Object Selection & Risk Hysteresis
%         ↓
%   Decision Logic
%         ↓
%   Local Path Planning
%         ↓
%   MPC Control (mpc_controller.m)
%         ↓
%   Kinematic Bicycle Model Motion
%
% IMPORTANT:
%   - The autonomy stack operates STRICTLY on EKF-estimated state.
%   - Ground-truth object states are used ONLY for diagnostics and tracking error metrics.

if nargin < 1 || isempty(scenario_input)
    scenario_input = "VILLAGE_CATTLE";
end

if nargin < 2 || isempty(sensor_params)
    sensor_params = struct('detection_range', 50.0, 'enable_noise', false);
end

% Ensure path setup
addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');
addpath('D:\INDRA\tracking');
addpath('D:\INDRA\prediction');
addpath('D:\INDRA\risk');
addpath('D:\INDRA\planning');
addpath('D:\INDRA\control');

% Reset persistent controller state between runs
clear functions;

% Resolve scenario
if isstruct(scenario_input)
    scenario = scenario_input;
else
    scenario = scenario_config(scenario_input);
end

validate_scenario(scenario);

%% ============================================================
% 1. SIMULATION PARAMETERS
% =============================================================

Ts = 0.1;
simulation_time = 8.0;
t = 0:Ts:simulation_time;
N = numel(t);

rng(1); % Reproducible sensor / simulation noise

%% ============================================================
% 2. VEHICLE INITIAL STATE & LIMITS
% =============================================================

ego_x = scenario.ego_x;
ego_y = scenario.ego_y;
ego_speed = scenario.ego_speed;
ego_heading = scenario.ego_heading;

road_center_y = scenario.road_center_y;
max_avoidance_offset = scenario.max_avoidance_offset;

wheelbase = 2.5;
steering_ratio = 16;
steering_wheel_limit = deg2rad(450);
max_steering_angle = deg2rad(10);
normal_steering_limit = deg2rad(10);
max_steering_rate = deg2rad(25);

% Control limits
min_acceleration = -6.0;
max_acceleration = 1.0;
min_speed = 0.0;
max_speed = 12.0;

applied_steering = 0.0;
steering = 0.0;
acceleration = 0.0;

%% ============================================================
% 3. PLANNER & HYSTERESIS STATE
% =============================================================

risk_level = "LOW";
raw_risk = "LOW";
previous_risk = "LOW";

safe_cycles = 0;
safe_cycles_required = 8;
recovery_hold = 0;
recovery_hold_steps = 5;
avoidance_hold_duration = 10;
avoidance_hold_steps = 0;

commanded_target_speed = ego_speed;
target_speed = ego_speed;

planner_target_y = road_center_y;
planner_action = "CENTER / CRUISE";
planner_safe = true;
maneuver_time = 2.0;
return_to_center_rate = 0.8;

%% ============================================================
% 4. SAFETY PARAMETERS
% =============================================================

base_safety_distance = 2.0;
max_uncertainty_distance = 2.0;

high_distance_threshold = 1.5;
high_time_threshold = 1.0;
high_ttc_threshold = 1.0;

medium_distance_threshold = 3.0;
medium_time_threshold = 2.5;
medium_ttc_threshold = 2.5;

%% ============================================================
% 5. PREDICTION CONFIGURATION
% =============================================================

prediction_horizon = 4.0;
prediction_steps = round(prediction_horizon / Ts);
future_time = (1:prediction_steps) * Ts;

F_cv = [1 0 Ts 0;
        0 1 0 Ts;
        0 0 1  0;
        0 0 0  1];
Q_cv = diag([0.01 0.01 0.1 0.1]);

%% ============================================================
% 6. LOGGING ARRAYS
% =============================================================

num_objects = scenario.object_count;

ego_x_history = zeros(1, N);
ego_y_history = zeros(1, N);
ego_speed_history = zeros(1, N);
ego_heading_history = zeros(1, N);

steering_history = zeros(1, N);
steering_rate_history = zeros(1, N);
acceleration_history = zeros(1, N);

risk_history = strings(1, N);
raw_risk_history = strings(1, N);
behavior_history = strings(1, N);
planning_mode_history = strings(1, N);
target_speed_history = zeros(1, N);

predicted_distance_history = zeros(1, N);
required_safety_distance_history = zeros(1, N);
safety_margin_history = zeros(1, N);
TTC_history = zeros(1, N);
time_to_closest_history = zeros(1, N);
time_to_conflict_history = zeros(1, N);

uncertainty_x_history = zeros(1, N);
uncertainty_y_history = zeros(1, N);
uncertainty_margin_history = zeros(1, N);

planner_safe_history = strings(1, N);
selected_lateral_history = zeros(1, N);
planner_target_history = zeros(1, N);
planner_reference_history = zeros(1, N);
planner_action_history = strings(1, N);
controller_status_history = strings(1, N);

critical_object_id_history = zeros(1, N);
critical_risk_history = strings(1, N);

true_x_history = zeros(num_objects, N);
true_y_history = zeros(num_objects, N);
true_vx_history = zeros(num_objects, N);
true_vy_history = zeros(num_objects, N);

est_x_history = zeros(num_objects, N);
est_y_history = zeros(num_objects, N);
est_vx_history = zeros(num_objects, N);
est_vy_history = zeros(num_objects, N);

actual_distance_history = zeros(num_objects, N);

tracks = [];

%% ============================================================
% 7. MAIN CLOSED-LOOP SIMULATION LOOP
% =============================================================

for k = 1:N
    current_time = t(k);

    % 7.1 True World Generation
    world = scenario_world(scenario, current_time);

    for i = 1:num_objects
        true_x_history(i, k) = world.x(i);
        true_y_history(i, k) = world.y(i);
        true_vx_history(i, k) = world.vx(i);
        true_vy_history(i, k) = world.vy(i);
        actual_distance_history(i, k) = hypot(world.x(i) - ego_x, world.y(i) - ego_y);
    end

    % 7.2 Current Ego State Structure
    ego_state = struct( ...
        'x', ego_x, ...
        'y', ego_y, ...
        'speed', ego_speed, ...
        'heading', ego_heading);

    % 7.3 Synthetic Sensor Model (Radar / Relative Observations)
    obs = synthetic_sensor_model(world, ego_state, sensor_params, current_time);

    % 7.4 Sensor -> EKF Adapter (Multi-Object EKF in World Coordinates)
    [tracks, track_estimates] = sensor_to_ekf_adapter(obs, ego_state, tracks, Ts);

    % 7.5 Autonomy Pipeline: Process Tracked Objects
    M = track_estimates.object_count;

    obj_raw_risk = strings(1, max(M, 1));
    obj_risk_numeric = zeros(1, max(M, 1));
    obj_min_dist = zeros(1, max(M, 1));
    obj_min_margin = zeros(1, max(M, 1));
    obj_tca = zeros(1, max(M, 1));
    obj_t_conflict = zeros(1, max(M, 1));
    obj_ttc = inf(1, max(M, 1));

    obj_pred_x = zeros(max(M, 1), prediction_steps);
    obj_pred_y = zeros(max(M, 1), prediction_steps);
    obj_unc_margin = zeros(max(M, 1), prediction_steps);
    obj_est_state = zeros(4, max(M, 1));
    obj_min_idx = zeros(1, max(M, 1));

    % Straight-line ego prediction from current state
    ego_predicted_x = ego_x + ego_speed * cos(ego_heading) .* future_time;
    ego_predicted_y = ego_y + ego_speed * sin(ego_heading) .* future_time;

    if M == 0
        % Clear path fallback
        obj_raw_risk(1) = "LOW";
        obj_risk_numeric(1) = 1;
        obj_min_dist(1) = 50.0;
        obj_min_margin(1) = 48.0;
        obj_tca(1) = 4.0;
        obj_t_conflict(1) = inf;
        obj_ttc(1) = inf;
        obj_pred_x(1, :) = 100.0;
        obj_pred_y(1, :) = 100.0;
        obj_unc_margin(1, :) = 0.0;
        obj_min_idx(1) = 1;
        best_obj_idx = 1;
        critical_id = 0;
        critical_type = "NONE";
    else
        for i = 1:M
            est_x = track_estimates.x(i);
            est_y = track_estimates.y(i);
            est_vx = track_estimates.vx(i);
            est_vy = track_estimates.vy(i);
            obj_est_state(:, i) = [est_x; est_y; est_vx; est_vy];

            orig_id = track_estimates.object_id(i);
            if orig_id <= num_objects
                est_x_history(orig_id, k) = est_x;
                est_y_history(orig_id, k) = est_y;
                est_vx_history(orig_id, k) = est_vx;
                est_vy_history(orig_id, k) = est_vy;
            end

            % Future CV Prediction from EKF Estimate
            x_future = [est_x; est_y; est_vx; est_vy];
            P_future = track_estimates.P(:, :, i);
            pred_x_i = zeros(1, prediction_steps);
            pred_y_i = zeros(1, prediction_steps);
            pred_cov_i = zeros(2, 2, prediction_steps);

            for p = 1:prediction_steps
                x_future = F_cv * x_future;
                P_future = F_cv * P_future * F_cv' + Q_cv;
                pred_x_i(p) = x_future(1);
                pred_y_i(p) = x_future(2);
                pred_cov_i(:, :, p) = P_future(1:2, 1:2);
            end
            obj_pred_x(i, :) = pred_x_i;
            obj_pred_y(i, :) = pred_y_i;

            % Uncertainty Margin
            unc_margin_i = zeros(1, prediction_steps);
            for p = 1:prediction_steps
                sig_x = sqrt(max(pred_cov_i(1, 1, p), 0));
                sig_y = sqrt(max(pred_cov_i(2, 2, p), 0));
                raw_unc = 2 * sqrt(sig_x^2 + sig_y^2);
                unc_margin_i(p) = min(raw_unc, max_uncertainty_distance);
            end
            obj_unc_margin(i, :) = unc_margin_i;

            % Distance & Safety Margin
            pred_dist_i = zeros(1, prediction_steps);
            req_safety_dist_i = zeros(1, prediction_steps);
            safety_margin_i = zeros(1, prediction_steps);

            for p = 1:prediction_steps
                dx = pred_x_i(p) - ego_predicted_x(p);
                dy = pred_y_i(p) - ego_predicted_y(p);
                pred_dist_i(p) = hypot(dx, dy);
                req_safety_dist_i(p) = base_safety_distance + unc_margin_i(p);
                safety_margin_i(p) = pred_dist_i(p) - req_safety_dist_i(p);
            end

            [min_dist_i, min_idx_i] = min(pred_dist_i);
            obj_min_idx(i) = min_idx_i;
            min_safety_margin_i = safety_margin_i(min_idx_i);
            tca_i = future_time(min_idx_i);

            conflict_indices_i = find(safety_margin_i <= 0);
            if isempty(conflict_indices_i)
                t_conflict_i = inf;
            else
                t_conflict_i = future_time(conflict_indices_i(1));
            end

            % Closing Speed & TTC
            rel_x_est = est_x - ego_x;
            rel_y_est = est_y - ego_y;
            rel_vx_est = est_vx - ego_speed * cos(ego_heading);
            rel_vy_est = est_vy - ego_speed * sin(ego_heading);
            cur_range = hypot(rel_x_est, rel_y_est);

            if cur_range > 1e-6
                ux = rel_x_est / cur_range;
                uy = rel_y_est / cur_range;
                closing_spd = -(rel_vx_est * ux + rel_vy_est * uy);
            else
                closing_spd = 0;
            end

            if closing_spd > 0
                ttc_i = cur_range / closing_spd;
            else
                ttc_i = inf;
            end

            % Risk Assessment
            high_distance_cond = min_dist_i <= high_distance_threshold && tca_i <= high_time_threshold;
            high_conflict_cond = t_conflict_i <= high_time_threshold;
            high_ttc_cond = ttc_i <= high_ttc_threshold;

            if high_distance_cond || high_conflict_cond || high_ttc_cond
                raw_risk_i = "HIGH";
                risk_num_i = 3;
            else
                med_distance_cond = min_dist_i <= medium_distance_threshold && tca_i <= medium_time_threshold;
                med_conflict_cond = t_conflict_i <= medium_time_threshold;
                med_ttc_cond = ttc_i <= medium_ttc_threshold;

                if med_distance_cond || med_conflict_cond || med_ttc_cond
                    raw_risk_i = "MEDIUM";
                    risk_num_i = 2;
                else
                    raw_risk_i = "LOW";
                    risk_num_i = 1;
                end
            end

            if current_time <= 0.2 && raw_risk_i == "HIGH"
                raw_risk_i = "MEDIUM";
                risk_num_i = 2;
            end

            obj_raw_risk(i) = raw_risk_i;
            obj_risk_numeric(i) = risk_num_i;
            obj_min_dist(i) = min_dist_i;
            obj_min_margin(i) = min_safety_margin_i;
            obj_tca(i) = tca_i;
            obj_t_conflict(i) = t_conflict_i;
            obj_ttc(i) = ttc_i;
        end

        % Critical Object Selection (Highest Risk > Lowest Predicted Distance)
        max_risk_num = max(obj_risk_numeric);
        candidate_idxs = find(obj_risk_numeric == max_risk_num);

        if numel(candidate_idxs) == 1
            best_obj_idx = candidate_idxs(1);
        else
            [~, best_cand_subidx] = min(obj_min_dist(candidate_idxs));
            best_obj_idx = candidate_idxs(best_cand_subidx);
        end

        critical_id = track_estimates.object_id(best_obj_idx);
        critical_type = track_estimates.object_type(best_obj_idx);
    end

    raw_risk = obj_raw_risk(best_obj_idx);
    minimum_distance = obj_min_dist(best_obj_idx);
    minimum_safety_margin = obj_min_margin(best_obj_idx);
    time_to_closest = obj_tca(best_obj_idx);
    time_to_conflict = obj_t_conflict(best_obj_idx);
    TTC = obj_ttc(best_obj_idx);

    predicted_x = obj_pred_x(best_obj_idx, :);
    predicted_y = obj_pred_y(best_obj_idx, :);
    uncertainty_margin = obj_unc_margin(best_obj_idx, :);
    estimated_x = obj_est_state(1, best_obj_idx);
    estimated_y = obj_est_state(2, best_obj_idx);
    estimated_vx = obj_est_state(3, best_obj_idx);
    estimated_vy = obj_est_state(4, best_obj_idx);

    critical_object_id_history(k) = critical_id;
    critical_risk_history(k) = raw_risk;

    % 7.6 Risk Hysteresis
    if raw_risk == "HIGH"
        risk_level = "HIGH";
        safe_cycles = 0;
        recovery_hold = recovery_hold_steps;
    elseif raw_risk == "MEDIUM"
        risk_level = "MEDIUM";
        safe_cycles = 0;
    else
        safe_cycles = safe_cycles + 1;
        if previous_risk == "HIGH" || previous_risk == "MEDIUM"
            if safe_cycles >= safe_cycles_required
                risk_level = "LOW";
            else
                risk_level = "MEDIUM";
            end
        else
            risk_level = "LOW";
        end
    end

    % 7.7 Decision Logic
    if risk_level == "HIGH"
        behavior = "EMERGENCY BRAKE";
        planning_mode = "EMERGENCY";
        target_speed = 0;
        commanded_target_speed = 0;
        recovery_hold = recovery_hold_steps;
    elseif risk_level == "MEDIUM"
        behavior = "SLOW / AVOID";
        planning_mode = "AVOID";
        commanded_target_speed = min(ego_speed, 5);
        target_speed = commanded_target_speed;
    else
        behavior = "CRUISE";
        planning_mode = "CRUISE";
        if previous_risk == "HIGH" || previous_risk == "MEDIUM" || recovery_hold > 0
            if recovery_hold > 0
                recovery_hold = recovery_hold - 1;
            end
            commanded_target_speed = max(commanded_target_speed, ego_speed);
            commanded_target_speed = min(commanded_target_speed, 5);
        else
            commanded_target_speed = max(commanded_target_speed, ego_speed);
            commanded_target_speed = min(commanded_target_speed + 0.25, 10);
        end
        target_speed = min(max(commanded_target_speed, ego_speed), 10);
    end

    % 7.8 Local Path Planner
    max_uncertainty_margin = max(uncertainty_margin);
    lateral_candidates = [-3, 0, 3];
    candidate_safe = false(1, numel(lateral_candidates));
    candidate_distance = zeros(1, numel(lateral_candidates));

    for c = 1:numel(lateral_candidates)
        lateral_offset = lateral_candidates(c);
        candidate_min_distance = inf;
        for p = 1:prediction_steps
            candidate_x = ego_predicted_x(p);
            candidate_y = ego_predicted_y(p) + lateral_offset;
            d = hypot(predicted_x(p) - candidate_x, predicted_y(p) - candidate_y);
            candidate_min_distance = min(candidate_min_distance, d);
        end
        candidate_distance(c) = candidate_min_distance;
        candidate_safe(c) = candidate_min_distance - max_uncertainty_margin > base_safety_distance;
    end

    if risk_level == "HIGH"
        planner_safe = false;
        planner_target_y = road_center_y;
        avoidance_hold_steps = 0;
        planner_action = "EMERGENCY STOP";
    elseif raw_risk == "MEDIUM"
        current_offset = planner_target_y - road_center_y;
        if current_offset < -0.1 && candidate_safe(1)
            planner_safe = true;
            planner_target_y = road_center_y - max_avoidance_offset;
            planner_action = "MAINTAIN AVOIDANCE LEFT";
        elseif current_offset > 0.1 && candidate_safe(3)
            planner_safe = true;
            planner_target_y = road_center_y + max_avoidance_offset;
            planner_action = "MAINTAIN AVOIDANCE RIGHT";
        elseif candidate_safe(2)
            planner_safe = true;
            planner_target_y = road_center_y;
            avoidance_hold_steps = 0;
            planner_action = "CENTER / CRUISE";
        elseif candidate_safe(1)
            planner_safe = true;
            planner_target_y = road_center_y - max_avoidance_offset;
            avoidance_hold_steps = avoidance_hold_duration;
            planner_action = "SAFE AVOIDANCE LEFT";
        elseif candidate_safe(3)
            planner_safe = true;
            planner_target_y = road_center_y + max_avoidance_offset;
            avoidance_hold_steps = avoidance_hold_duration;
            planner_action = "SAFE AVOIDANCE RIGHT";
        else
            planner_safe = false;
            planner_target_y = road_center_y;
            avoidance_hold_steps = 0;
            planner_action = "BRAKE";
        end
    else
        planner_safe = true;
        current_offset = planner_target_y - road_center_y;
        if abs(current_offset) <= 1e-9
            planner_target_y = road_center_y;
            planner_action = "CENTER / CRUISE";
        elseif avoidance_hold_steps > 0
            avoidance_hold_steps = avoidance_hold_steps - 1;
            planner_action = "MAINTAIN AVOIDANCE";
        else
            recovery_step = return_to_center_rate * Ts;
            if current_offset > 0
                planner_target_y = max(road_center_y, planner_target_y - recovery_step);
            else
                planner_target_y = min(road_center_y, planner_target_y + recovery_step);
            end
            if abs(planner_target_y - road_center_y) < 1e-9
                planner_target_y = road_center_y;
                planner_action = "RETURN TO CENTER";
            else
                planner_action = "RECOVER TOWARD CENTER";
            end
        end
    end

    planner_target_y = max(road_center_y - max_avoidance_offset, ...
        min(road_center_y + max_avoidance_offset, planner_target_y));
    selected_lateral_offset = planner_target_y - road_center_y;

    % Planner Reference Trajectory
    planner_ref_x = ego_x + ego_speed * cos(ego_heading) .* future_time;
    alpha = min(future_time / maneuver_time, 1);
    planner_ref_y = ego_y + (planner_target_y - ego_y) .* alpha;
    planner_ref_y = max(road_center_y - max_avoidance_offset, ...
        min(road_center_y + max_avoidance_offset, planner_ref_y));
    planner_ref_speed = target_speed * ones(1, prediction_steps);

    % 7.9 MPC Control
    lookahead_distance = max(5.0, ego_speed * 1.0);
    planner_ref_heading = atan2(planner_target_y - ego_y, lookahead_distance);
    planner_ref_heading = max(-deg2rad(8), min(deg2rad(8), planner_ref_heading));

    try
        control_result = mpc_controller( ...
            ego_speed, ...
            ego_y, ...
            ego_heading, ...
            planner_ref_y(end), ...
            planner_ref_heading, ...
            planner_ref_speed(end));

        mpc_steering = control_result.steering;
        acceleration = control_result.acceleration;
        controller_status = control_result.controller_status;
    catch ME
        mpc_steering = 0;
        acceleration = -6;
        controller_status = "MPC FAIL-SAFE";
    end

    % Safety Override
    steering = mpc_steering;
    if risk_level ~= "HIGH"
        steering = max(-normal_steering_limit, min(normal_steering_limit, steering));
    end

    if risk_level == "HIGH"
        steering = 0;
        acceleration = -6;
    elseif risk_level == "MEDIUM"
        if ~planner_safe
            steering = 0;
            acceleration = min(acceleration, -4);
        else
            acceleration = min(acceleration, 0);
        end
    else
        acceleration = max(min(acceleration, max_acceleration), min_acceleration);
    end

    % Steering Rate Limiter
    desired_steering_rate = (steering - applied_steering) / Ts;
    if abs(desired_steering_rate) > max_steering_rate
        applied_steering_rate = sign(desired_steering_rate) * max_steering_rate;
        applied_steering = applied_steering + applied_steering_rate * Ts;
    else
        applied_steering_rate = desired_steering_rate;
        applied_steering = steering;
    end
    applied_steering = max(-max_steering_angle, min(max_steering_angle, applied_steering));

    % 7.10 Kinematic Bicycle Model Update
    beta = atan(0.5 * tan(applied_steering));
    ego_x = ego_x + ego_speed * cos(ego_heading + beta) * Ts;
    ego_y = ego_y + ego_speed * sin(ego_heading + beta) * Ts;
    ego_heading = ego_heading + (ego_speed / wheelbase) * sin(beta) * Ts;
    ego_speed = max(min_speed, min(max_speed, ego_speed + acceleration * Ts));

    previous_risk = risk_level;

    % 7.11 Record Logging
    ego_x_history(k) = ego_x;
    ego_y_history(k) = ego_y;
    ego_speed_history(k) = ego_speed;
    ego_heading_history(k) = ego_heading;

    steering_history(k) = applied_steering;
    steering_rate_history(k) = applied_steering_rate;
    acceleration_history(k) = acceleration;

    risk_history(k) = risk_level;
    raw_risk_history(k) = raw_risk;
    behavior_history(k) = behavior;
    planning_mode_history(k) = planning_mode;
    target_speed_history(k) = target_speed;

    predicted_distance_history(k) = minimum_distance;
    required_safety_distance_history(k) = base_safety_distance + max_uncertainty_margin;
    safety_margin_history(k) = minimum_safety_margin;
    TTC_history(k) = TTC;
    time_to_closest_history(k) = time_to_closest;
    time_to_conflict_history(k) = time_to_conflict;

    if M > 0 && best_obj_idx <= M
        uncertainty_x_history(k) = track_estimates.uncertainty_x(best_obj_idx);
        uncertainty_y_history(k) = track_estimates.uncertainty_y(best_obj_idx);
    else
        uncertainty_x_history(k) = 0.0;
        uncertainty_y_history(k) = 0.0;
    end
    uncertainty_margin_history(k) = max_uncertainty_margin;

    planner_safe_history(k) = string(planner_safe);
    selected_lateral_history(k) = selected_lateral_offset;
    planner_target_history(k) = planner_target_y;
    planner_reference_history(k) = planner_ref_y(end);
    planner_action_history(k) = planner_action;
    controller_status_history(k) = controller_status;
end

%% ============================================================
% 8. PERFORMANCE METRICS
% =============================================================

medium_idx = find(risk_history == "MEDIUM", 1);
high_idx = find(risk_history == "HIGH", 1);
interv_idx = find(acceleration_history < -1.0, 1);

first_med_time = Inf; if ~isempty(medium_idx), first_med_time = t(medium_idx); end
first_high_time = Inf; if ~isempty(high_idx), first_high_time = t(high_idx); end
first_interv_time = Inf; if ~isempty(interv_idx), first_interv_time = t(interv_idx); end

eval_steps = 2:N;
pos_err = hypot(est_x_history(1, eval_steps) - true_x_history(1, eval_steps), ...
                est_y_history(1, eval_steps) - true_y_history(1, eval_steps));
vel_err = hypot(est_vx_history(1, eval_steps) - true_vx_history(1, eval_steps), ...
                est_vy_history(1, eval_steps) - true_vy_history(1, eval_steps));

pos_rmse = sqrt(mean(pos_err.^2));
vel_rmse = sqrt(mean(vel_err.^2));
max_pos_err = max(pos_err);
max_vel_err = max(vel_err);

min_true_dist = min(actual_distance_history(:));
collision_flag = (min_true_dist < 1.0);

%% ============================================================
% 9. ASSEMBLE RESULT STRUCTURE
% =============================================================

indra_result = struct();
indra_result.scenario_name = scenario.name;
indra_result.scenario_type = scenario.type;
indra_result.time = t;

indra_result.ego_x = ego_x_history;
indra_result.ego_y = ego_y_history;
indra_result.ego_speed = ego_speed_history;
indra_result.ego_heading = ego_heading_history;

indra_result.steering = steering_history;
indra_result.steering_rate = steering_rate_history;
indra_result.acceleration = acceleration_history;

indra_result.risk = risk_history;
indra_result.raw_risk = raw_risk_history;
indra_result.behavior = behavior_history;
indra_result.planning_mode = planning_mode_history;
indra_result.target_speed = target_speed_history;

indra_result.predicted_distance = predicted_distance_history;
indra_result.required_safety_distance = required_safety_distance_history;
indra_result.safety_margin = safety_margin_history;
indra_result.TTC = TTC_history;
indra_result.time_to_closest = time_to_closest_history;
indra_result.time_to_conflict = time_to_conflict_history;

indra_result.uncertainty_x = uncertainty_x_history;
indra_result.uncertainty_y = uncertainty_y_history;
indra_result.uncertainty_margin = uncertainty_margin_history;

indra_result.selected_lateral_offset = selected_lateral_history;
indra_result.planner_target_y = planner_target_history;
indra_result.planner_reference_y = planner_reference_history;
indra_result.planner_action = planner_action_history;
indra_result.controller_status = controller_status_history;

indra_result.critical_object_id = critical_object_id_history;
indra_result.critical_risk = critical_risk_history;

indra_result.first_medium_risk_time = first_med_time;
indra_result.first_high_risk_time = first_high_time;
indra_result.first_intervention_time = first_interv_time;

indra_result.pos_rmse = pos_rmse;
indra_result.vel_rmse = vel_rmse;
indra_result.max_pos_err = max_pos_err;
indra_result.max_vel_err = max_vel_err;
indra_result.min_true_distance = min_true_dist;
indra_result.collision_detected = collision_flag;

indra_result.true_x = true_x_history;
indra_result.true_y = true_y_history;
indra_result.true_vx = true_vx_history;
indra_result.true_vy = true_vy_history;
indra_result.est_x = est_x_history;
indra_result.est_y = est_y_history;
indra_result.est_vx = est_vx_history;
indra_result.est_vy = est_vy_history;
indra_result.actual_distance = actual_distance_history;
indra_result.scenario = scenario;
indra_result.road_center_y = road_center_y;
indra_result.max_avoidance_offset = max_avoidance_offset;

fprintf('\n============================================\n');
fprintf('   INDRA SENSOR CLOSED-LOOP RESULTS\n');
fprintf('============================================\n');
fprintf('Initial Speed       : %.2f m/s\n', ego_speed_history(1));
fprintf('Final Speed         : %.2f m/s\n', ego_speed_history(end));
fprintf('Final X             : %.2f m\n', ego_x_history(end));
fprintf('Final Y             : %.2f m\n', ego_y_history(end));
fprintf('Final Heading       : %.2f deg\n', rad2deg(ego_heading_history(end)));
fprintf('First MEDIUM Risk   : %.2f s\n', first_med_time);
fprintf('First HIGH Risk     : %.2f s\n', first_high_time);
fprintf('First Intervention  : %.2f s\n', first_interv_time);
fprintf('Max Steering        : %.2f deg\n', rad2deg(max(abs(steering_history))));
fprintf('Max Braking         : %.2f m/s^2\n', min(acceleration_history));
fprintf('Min Predicted Dist  : %.2f m\n', min(predicted_distance_history));
fprintf('Min True Dist       : %.2f m\n', min_true_dist);
fprintf('Collision Detected  : %s\n', mat2str(collision_flag));
fprintf('Position RMSE       : %.3f m\n', pos_rmse);
fprintf('Velocity RMSE       : %.3f m/s\n', vel_rmse);
fprintf('============================================\n\n');

end
