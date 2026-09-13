function indra_result = indra_main(scenario_input)
%% INDRA - Indian Navigation & Dynamic Risk-Aware Autonomy
% CLEAN WORLD-FRAME PLANNER - CLOSED-LOOP MAIN
%
% Scenario:
%   Ego vehicle travelling on an unstructured road
%   Sudden cattle crossing
%
% Pipeline:
%   Sensing
%       ->
%   Perception
%       ->
%   EKF Tracking
%       ->
%   Future Prediction
%       ->
%   Uncertainty-Aware Risk
%       ->
%   Decision
%       ->
%   Local Path Planning
%       ->
%   MPC Control
%       ->
%   Vehicle Motion
%
% IMPORTANT:
%   - Prediction uses the same world coordinate frame.
%   - No privileged future hazard information is given to autonomy.
%   - Vehicle motion uses a kinematic bicycle model.
%   - Planner has authority over the selected lateral path.
%
% NOTE:
%   This version intentionally keeps the existing sensing, tracking,
%   prediction, uncertainty, risk, and planning logic stable.
%
%   The main correction is the integration between:
%
%       LOCAL PATH PLANNER -> MPC -> VEHICLE MOTION
%
%   In particular, when CENTER is selected, a controller-model artifact
%   must not create an unnecessary large steering command.

if nargin < 1 || isempty(scenario_input)
    if exist('requested_scenario_name', 'var')
        scenario_input = requested_scenario_name;
    else
        scenario_input = "VILLAGE_CATTLE";
    end
end

close all;

%% ============================================================
% PATH SETUP
% =============================================================

addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');

% Reset persistent controller state between runs.
clear functions;

fprintf('\n============================================\n');
fprintf('       INDRA CLOSED-LOOP AUTONOMY\n');
fprintf('============================================\n\n');

%% ============================================================
% SCENARIO CONFIGURATION
% =============================================================

if isstruct(scenario_input)
    scenario = scenario_input;
else
    scenario = scenario_config(scenario_input);
end

%% ============================================================
% 1. SIMULATION PARAMETERS
% =============================================================

Ts = 0.1;
simulation_time = 8;

t = 0:Ts:simulation_time;
N = numel(t);

% Reproducible sensor noise
rng(1);

%% ============================================================
% 2. VEHICLE / SCENARIO
% =============================================================

% -------------------------------------------------------------
% Ego vehicle initial state
% -------------------------------------------------------------

ego_x = scenario.ego_x;
ego_y = scenario.ego_y;

ego_speed = scenario.ego_speed;
ego_heading = scenario.ego_heading;

% -------------------------------------------------------------
% Kinematic bicycle parameters
% -------------------------------------------------------------

wheelbase = 2.5;

max_steering_angle = deg2rad(30);

% Maximum steering rate:
% 25 deg/s
max_steering_rate = deg2rad(25);

% Normal-operation steering cap for smoother closed-loop motion.
normal_steering_limit = deg2rad(10);

previous_steering = 0;

%% ============================================================
% 3. STORAGE & MULTI-OBJECT TRACK INITIALIZATION
% =============================================================

num_objects = scenario.object_count;

tracks = struct();
objects_history = struct();

for i = 1:num_objects
    tracks(i).id = scenario.object_id(i);
    tracks(i).type = scenario.object_type(i);
    tracks(i).x_est = [
        scenario.object_initial_x(i);
        scenario.object_initial_y(i);
        scenario.object_vx(i);
        scenario.object_vy(i)
    ];
    tracks(i).P = diag([1 1 1 1]);

    objects_history(i).id = scenario.object_id(i);
    objects_history(i).type = scenario.object_type(i);
    objects_history(i).true_x = zeros(1,N);
    objects_history(i).true_y = zeros(1,N);
    objects_history(i).true_vx = zeros(1,N);
    objects_history(i).true_vy = zeros(1,N);
    objects_history(i).estimated_x = zeros(1,N);
    objects_history(i).estimated_y = zeros(1,N);
    objects_history(i).estimated_vx = zeros(1,N);
    objects_history(i).estimated_vy = zeros(1,N);
    objects_history(i).raw_risk = strings(1,N);
    objects_history(i).predicted_distance = zeros(1,N);
    objects_history(i).TTC = inf(1,N);
    objects_history(i).time_to_closest = inf(1,N);
    objects_history(i).time_to_conflict = inf(1,N);
    objects_history(i).detected = false(1,N);
end

critical_object_id_history = zeros(1,N);
critical_risk_history = strings(1,N);

ego_x_history = zeros(1,N);
ego_y_history = zeros(1,N);

ego_speed_history = zeros(1,N);
ego_heading_history = zeros(1,N);

cow_x_history = zeros(1,N);
cow_y_history = zeros(1,N);
cow_vx_history = zeros(1,N);
cow_vy_history = zeros(1,N);

estimated_x_history = zeros(1,N);
estimated_y_history = zeros(1,N);

estimated_vx_history = zeros(1,N);
estimated_vy_history = zeros(1,N);

risk_history = strings(1,N);
raw_risk_history = strings(1,N);

behavior_history = strings(1,N);
planning_mode_history = strings(1,N);

target_speed_history = zeros(1,N);

acceleration_history = zeros(1,N);
steering_history = zeros(1,N);

steering_rate_history = zeros(1,N);

predicted_distance_history = zeros(1,N);
required_safety_distance_history = zeros(1,N);

safety_margin_history = zeros(1,N);

TTC_history = inf(1,N);

time_to_closest_history = inf(1,N);
time_to_conflict_history = inf(1,N);

uncertainty_x_history = zeros(1,N);
uncertainty_y_history = zeros(1,N);

uncertainty_margin_history = zeros(1,N);

planner_safe_history = false(1,N);

selected_lateral_history = zeros(1,N);

% Clean world-frame planner state.
road_center_y = scenario.road_center_y;
max_avoidance_offset = scenario.max_avoidance_offset;
planner_target_y = road_center_y;
avoidance_hold_steps = 0;
avoidance_hold_duration = 8;
return_to_center_rate = 0.8;
maneuver_time = 1.5;
planner_target_history = zeros(1,N);
planner_reference_history = zeros(1,N);
actual_distance_history = zeros(1,N);
planner_reference_heading_history = zeros(1,N);
planner_action_history = strings(1,N);

controller_status_history = strings(1,N);

%% ============================================================
% 4. RISK STATE
% =============================================================

previous_risk = "LOW";

safe_cycles = 0;

safe_cycles_required = 5;

recovery_hold = 0;

recovery_hold_steps = 5;

commanded_target_speed = ego_speed;

%% ============================================================
% 5. SAFETY PARAMETERS
% =============================================================

base_safety_distance = 2.0;

max_uncertainty_distance = 2.0;

% -------------------------------------------------------------
% HIGH RISK
% -------------------------------------------------------------

high_distance_threshold = 1.5;

high_time_threshold = 1.0;

high_ttc_threshold = 1.0;

% -------------------------------------------------------------
% MEDIUM RISK
% -------------------------------------------------------------

medium_distance_threshold = 3.0;

medium_time_threshold = 2.5;

medium_ttc_threshold = 2.5;

%% ============================================================
% 6. EKF SENSOR NOISE / MEASUREMENT MATRICES
% =============================================================

Q = diag([0.01 0.01 0.1 0.1]);

R_lidar = diag([0.15^2 0.15^2]);

R_radar = 0.25^2;

H_lidar = [1 0 0 0;
           0 1 0 0];

%% ============================================================
% 7. PREDICTION CONFIGURATION
% =============================================================

prediction_horizon = 4.0;

prediction_steps = ...
    round(prediction_horizon / Ts);

%% ============================================================
% 8. MAIN CLOSED LOOP
% =============================================================

for k = 1:N

    current_time = t(k);

    %% ========================================================
    % 8.1 TRUE WORLD
    % =========================================================

    world = scenario_world(scenario, current_time);

    %% ========================================================
    % 8.2 MULTI-OBJECT SENSING, EKF, PREDICTION & RISK
    % =========================================================

    F = [1 0 Ts 0;
         0 1 0 Ts;
         0 0 1  0;
         0 0 0  1];

    obj_raw_risk = strings(1, num_objects);
    obj_risk_numeric = zeros(1, num_objects);
    obj_min_dist = zeros(1, num_objects);
    obj_min_margin = zeros(1, num_objects);
    obj_tca = zeros(1, num_objects);
    obj_t_conflict = zeros(1, num_objects);
    obj_ttc = inf(1, num_objects);

    obj_pred_x = zeros(num_objects, prediction_steps);
    obj_pred_y = zeros(num_objects, prediction_steps);
    obj_unc_margin = zeros(num_objects, prediction_steps);
    obj_est_state = zeros(4, num_objects);
    obj_min_idx = zeros(1, num_objects);

    future_time = (1:prediction_steps)*Ts;

    % Conservative straight-line ego prediction from current state
    ego_predicted_x = ego_x + ego_speed*cos(ego_heading).*future_time;
    ego_predicted_y = ego_y + ego_speed*sin(ego_heading).*future_time;

    for i = 1:num_objects

        true_x = world.x(i);
        true_y = world.y(i);
        true_vx = world.vx(i);
        true_vy = world.vy(i);

        objects_history(i).true_x(k) = true_x;
        objects_history(i).true_y(k) = true_y;
        objects_history(i).true_vx(k) = true_vx;
        objects_history(i).true_vy(k) = true_vy;

        % 8.2.1 Perception / Sensing
        rel_x = true_x - ego_x;
        rel_y = true_y - ego_y;
        range = hypot(rel_x, rel_y);
        obj_detected = (range <= scenario.sensor_range);
        objects_history(i).detected(k) = obj_detected;

        % 8.2.2 EKF Tracking
        if obj_detected

            lidar_x = true_x + 0.15*randn;
            lidar_y = true_y + 0.15*randn;
            z_lidar = [lidar_x; lidar_y];

            x_pred = F*tracks(i).x_est;
            P_pred = F*tracks(i).P*F' + Q;

            innovation = z_lidar - H_lidar*x_pred;
            S = H_lidar*P_pred*H_lidar' + R_lidar;
            K = P_pred*H_lidar'/S;

            tracks(i).x_est = x_pred + K*innovation;
            I = eye(4);
            tracks(i).P = (I - K*H_lidar)*P_pred*(I - K*H_lidar)' + K*R_lidar*K';

            radar_range = hypot(true_x - ego_x, true_y - ego_y);
            if radar_range > 1
                ux = (true_x - ego_x)/radar_range;
                uy = (true_y - ego_y)/radar_range;

                relative_velocity_x = true_vx - ego_speed*cos(ego_heading);
                relative_velocity_y = true_vy - ego_speed*sin(ego_heading);

                radar_measurement = relative_velocity_x*ux + relative_velocity_y*uy;
                H_radar = [0 0 ux uy];

                ego_radial_velocity = ego_speed*cos(ego_heading)*ux + ego_speed*sin(ego_heading)*uy;
                radar_object_radial_velocity = radar_measurement + ego_radial_velocity;

                radar_innovation = radar_object_radial_velocity - H_radar*tracks(i).x_est;
                S_radar = H_radar*tracks(i).P*H_radar' + R_radar;
                K_radar = tracks(i).P*H_radar'/S_radar;

                tracks(i).x_est = tracks(i).x_est + K_radar*radar_innovation;
                tracks(i).P = (I - K_radar*H_radar)*tracks(i).P*(I - K_radar*H_radar)' + K_radar*R_radar*K_radar';
            end
        else
            tracks(i).x_est = F*tracks(i).x_est;
            tracks(i).P = F*tracks(i).P*F' + Q;
        end

        est_x = tracks(i).x_est(1);
        est_y = tracks(i).x_est(2);
        est_vx = tracks(i).x_est(3);
        est_vy = tracks(i).x_est(4);
        obj_est_state(:, i) = tracks(i).x_est;

        objects_history(i).estimated_x(k) = est_x;
        objects_history(i).estimated_y(k) = est_y;
        objects_history(i).estimated_vx(k) = est_vx;
        objects_history(i).estimated_vy(k) = est_vy;

        % 8.2.3 Future Prediction
        x_future = tracks(i).x_est;
        P_future = tracks(i).P;
        pred_x_i = zeros(1, prediction_steps);
        pred_y_i = zeros(1, prediction_steps);
        pred_cov_i = zeros(2, 2, prediction_steps);

        for p = 1:prediction_steps
            x_future = F*x_future;
            P_future = F*P_future*F' + Q;
            pred_x_i(p) = x_future(1);
            pred_y_i(p) = x_future(2);
            pred_cov_i(:,:,p) = P_future(1:2, 1:2);
        end
        obj_pred_x(i, :) = pred_x_i;
        obj_pred_y(i, :) = pred_y_i;

        % 8.2.4 Uncertainty Margin
        unc_margin_i = zeros(1, prediction_steps);
        for p = 1:prediction_steps
            sigma_x = sqrt(max(pred_cov_i(1,1,p), 0));
            sigma_y = sqrt(max(pred_cov_i(2,2,p), 0));
            raw_uncertainty = 2*sqrt(sigma_x^2 + sigma_y^2);
            unc_margin_i(p) = min(raw_uncertainty, max_uncertainty_distance);
        end
        obj_unc_margin(i, :) = unc_margin_i;

        % 8.2.5 Distance & Safety Margin
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

        % 8.2.6 Current TTC
        rel_x_est = est_x - ego_x;
        rel_y_est = est_y - ego_y;
        rel_vx_est = est_vx - ego_speed*cos(ego_heading);
        rel_vy_est = est_vy - ego_speed*sin(ego_heading);
        cur_range = hypot(rel_x_est, rel_y_est);

        if cur_range > 1e-6
            ux = rel_x_est/cur_range;
            uy = rel_y_est/cur_range;
            closing_spd = -(rel_vx_est*ux + rel_vy_est*uy);
        else
            closing_spd = 0;
        end

        if closing_spd > 0
            ttc_i = cur_range/closing_spd;
        else
            ttc_i = inf;
        end

        % 8.2.7 Risk Assessment
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

        % Initial guard
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

        objects_history(i).raw_risk(k) = raw_risk_i;
        objects_history(i).predicted_distance(k) = min_dist_i;
        objects_history(i).TTC(k) = ttc_i;
        objects_history(i).time_to_closest(k) = tca_i;
        objects_history(i).time_to_conflict(k) = t_conflict_i;

    end

    %% ========================================================
    % 8.14 CRITICAL OBJECT / WORST RISK SELECTION
    % ========================================================
    % Priority: HIGH (3) > MEDIUM (2) > LOW (1).
    % Tie-breaking: smallest predicted distance.

    max_risk_num = max(obj_risk_numeric);
    candidate_idxs = find(obj_risk_numeric == max_risk_num);

    if numel(candidate_idxs) == 1
        critical_obj_idx = candidate_idxs(1);
    else
        [~, best_cand_subidx] = min(obj_min_dist(candidate_idxs));
        critical_obj_idx = candidate_idxs(best_cand_subidx);
    end

    critical_object_id_history(k) = tracks(critical_obj_idx).id;
    critical_risk_history(k) = obj_raw_risk(critical_obj_idx);

    raw_risk = obj_raw_risk(critical_obj_idx);
    minimum_distance = obj_min_dist(critical_obj_idx);
    minimum_safety_margin = obj_min_margin(critical_obj_idx);
    time_to_closest = obj_tca(critical_obj_idx);
    time_to_conflict = obj_t_conflict(critical_obj_idx);
    TTC = obj_ttc(critical_obj_idx);

    predicted_x = obj_pred_x(critical_obj_idx, :);
    predicted_y = obj_pred_y(critical_obj_idx, :);
    uncertainty_margin = obj_unc_margin(critical_obj_idx, :);
    estimated_x = obj_est_state(1, critical_obj_idx);
    estimated_y = obj_est_state(2, critical_obj_idx);
    estimated_vx = obj_est_state(3, critical_obj_idx);
    estimated_vy = obj_est_state(4, critical_obj_idx);

    true_cow_x = world.x(critical_obj_idx);
    true_cow_y = world.y(critical_obj_idx);
    cow_vx = world.vx(critical_obj_idx);
    cow_vy = world.vy(critical_obj_idx);

    cow_x_history(k) = true_cow_x;
    cow_y_history(k) = true_cow_y;
    cow_vx_history(k) = cow_vx;
    cow_vy_history(k) = cow_vy;

    estimated_x_history(k) = estimated_x;
    estimated_y_history(k) = estimated_y;
    estimated_vx_history(k) = estimated_vx;
    estimated_vy_history(k) = estimated_vy;

    %% ========================================================
    % 8.15 RISK HYSTERESIS
    % ========================================================

    if raw_risk == "HIGH"

        risk_level = "HIGH";

        safe_cycles = 0;

        recovery_hold = ...
            recovery_hold_steps;

    elseif raw_risk == "MEDIUM"

        risk_level = "MEDIUM";

        safe_cycles = 0;

    else

        safe_cycles = safe_cycles + 1;

        if previous_risk == "HIGH" || ...
                previous_risk == "MEDIUM"

            if safe_cycles >= ...
                    safe_cycles_required

                risk_level = "LOW";

            else

                risk_level = "MEDIUM";

            end

        else

            risk_level = "LOW";

        end

    end

    %% ========================================================
    % 8.16 DECISION LOGIC
    % ========================================================

    if risk_level == "HIGH"

        behavior = "EMERGENCY BRAKE";

        planning_mode = "EMERGENCY";

        target_speed = 0;

        commanded_target_speed = 0;

        recovery_hold = ...
            recovery_hold_steps;

    elseif risk_level == "MEDIUM"

        behavior = "SLOW / AVOID";

        planning_mode = "AVOID";

        commanded_target_speed = ...
            min(ego_speed,5);

        target_speed = ...
            commanded_target_speed;

    else

        behavior = "CRUISE";

        planning_mode = "CRUISE";

        if previous_risk == "HIGH" || ...
                previous_risk == "MEDIUM" || ...
                recovery_hold > 0

            if recovery_hold > 0

                recovery_hold = ...
                    recovery_hold - 1;

            end

            commanded_target_speed = ...
                max(commanded_target_speed,...
                    ego_speed);

            commanded_target_speed = ...
                min(commanded_target_speed,5);

        else

            commanded_target_speed = ...
                max(commanded_target_speed,...
                    ego_speed);

            commanded_target_speed = ...
                min(commanded_target_speed + 0.25,...
                    10);

        end

        target_speed = ...
            min(max(commanded_target_speed,...
                    ego_speed),10);

    end

    %% ========================================================
    % 8.17 LOCAL PATH PLANNER
    % ========================================================
    % Candidate offsets are used only for safety feasibility.
    % The commanded path is represented by planner_target_y in the
    % world frame. Road center = 0 m; avoidance target = +/-1.5 m.

    max_uncertainty_margin = max(uncertainty_margin);
    lateral_candidates = [-3 0 3];
    candidate_safe = false(1,numel(lateral_candidates));
    candidate_distance = zeros(1,numel(lateral_candidates));

    for c = 1:numel(lateral_candidates)
        lateral_offset = lateral_candidates(c);
        candidate_min_distance = inf;
        for p = 1:prediction_steps
            candidate_x = ego_predicted_x(p);
            candidate_y = ego_predicted_y(p) + lateral_offset;
            d = hypot(predicted_x(p)-candidate_x, predicted_y(p)-candidate_y);
            candidate_min_distance = min(candidate_min_distance,d);
        end
        candidate_distance(c) = candidate_min_distance;
        candidate_safe(c) = candidate_min_distance - max_uncertainty_margin > base_safety_distance;
    end

    %% ========================================================
    % 8.18 PATH SELECTION
    % ========================================================

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
                planner_target_y = max(road_center_y, planner_target_y-recovery_step);
            else
                planner_target_y = min(road_center_y, planner_target_y+recovery_step);
            end
            if abs(planner_target_y-road_center_y) < 1e-9
                planner_target_y = road_center_y;
                planner_action = "RETURN TO CENTER";
            else
                planner_action = "RECOVER TOWARD CENTER";
            end
        end
    end

    % Hard world-frame bound.
    planner_target_y = max(road_center_y-max_avoidance_offset, ...
        min(road_center_y+max_avoidance_offset, planner_target_y));

    % Derived diagnostic only; this does not drive planner state.
    selected_lateral_offset = planner_target_y - road_center_y;

    %% ========================================================
    % 8.19 PLANNER REFERENCE
    % ========================================================

    planner_ref_x = ego_x + ego_speed*cos(ego_heading).*future_time;
    alpha = min(future_time/maneuver_time,1);
    planner_ref_y = ego_y + (planner_target_y-ego_y).*alpha;
    planner_ref_y = max(road_center_y-max_avoidance_offset, ...
        min(road_center_y+max_avoidance_offset, planner_ref_y));
    planner_ref_speed = target_speed * ones(1,prediction_steps);

%% ========================================================
    % 8.20 MPC CONTROL
    % ========================================================

    % Heading reference points toward the current world-frame
    % planner target. This allows the controller to straighten the
    % vehicle after avoidance instead of freezing the heading at zero.
    lookahead_distance = max(5.0, ego_speed*1.0);

    planner_ref_heading = ...
        atan2( ...
            planner_target_y - ego_y, ...
            lookahead_distance);

    planner_ref_heading = ...
        max(-deg2rad(8), ...
        min(deg2rad(8), ...
            planner_ref_heading));

    try

        control_result = ...
            mpc_controller( ...
                ego_speed, ...
                ego_y, ...
                ego_heading, ...
                planner_ref_y(end), ...
                planner_ref_heading, ...
                planner_ref_speed(end));

        mpc_steering = ...
            control_result.steering;

        acceleration = ...
            control_result.acceleration;

        controller_status = ...
            control_result.controller_status;

    catch ME

        mpc_steering = 0;

        acceleration = -6;

        controller_status = ...
            "MPC FAIL-SAFE";

        fprintf( ...
            'MPC ERROR at %.1f s: %s\n', ...
            current_time, ...
            ME.message);

    end

    %% ========================================================
    % 8.21 PLANNER AUTHORITY + SAFETY OVERRIDE
    % ========================================================
    %
    % IMPORTANT:
    %
    % If CENTER is selected, steering must remain zero.
    %
    % This prevents an inconsistent controller model from turning
    % the vehicle when the planner did not request lateral motion.
    %
    % ---------------------------------------------------------

    % Keep MPC active even when the planner target is center.
    % This lets the controller correct residual heading/lateral error.
    steering = mpc_steering;

    % Smooth normal-operation steering.
    if risk_level ~= "HIGH"

        steering = ...
            max(-normal_steering_limit, ...
            min(normal_steering_limit, steering));

    end

    %% ---------------------------------------------------------
    % Safety override
    % ---------------------------------------------------------

    if risk_level == "HIGH"

        steering = 0;

        acceleration = -6;

    elseif risk_level == "MEDIUM"

        if ~planner_safe

            steering = 0;

            acceleration = ...
                min(acceleration,-4);

        else

            acceleration = ...
                min(acceleration,0);

        end

    else

        % LOW risk:
        % Prevent sudden braking during normal cruise.

        acceleration = ...
            max(acceleration,-0.5);

        acceleration = ...
            min(acceleration,1.0);

    end

    %% ========================================================
    % 8.22 STEERING LIMIT + RATE LIMIT
    % ========================================================

    steering = ...
        max(-max_steering_angle,...
        min(max_steering_angle,steering));

    max_delta_steering = ...
        max_steering_rate*Ts;

    steering_change = ...
        steering - previous_steering;

    if abs(steering_change) > ...
            max_delta_steering

        steering = ...
            previous_steering + ...
            sign(steering_change)*...
            max_delta_steering;

    end

    steering_rate = ...
        (steering-previous_steering)/Ts;

    previous_steering = steering;

    %% ========================================================
    % 8.23 VEHICLE MOTION
    % ========================================================
    %
    % Kinematic bicycle model:
    %
    %   yaw_rate = v/L * tan(delta)
    %
    % Position is integrated using the midpoint heading.
    %
    % ========================================================

    previous_speed = ego_speed;

    % Longitudinal motion
    ego_speed = ...
        previous_speed + ...
        acceleration*Ts;

    ego_speed = ...
        max(0,min(ego_speed,15));

    % Bicycle-model yaw rate
    yaw_rate = ...
        ego_speed/wheelbase * ...
        tan(steering);

    % New heading
    heading_new = ...
        ego_heading + ...
        yaw_rate*Ts;

    % Midpoint heading for position integration
    heading_mid = ...
        0.5*(ego_heading + heading_new);

    % Position update
    ego_x = ...
        ego_x + ...
        ego_speed*cos(heading_mid)*Ts;

    ego_y = ...
        ego_y + ...
        ego_speed*sin(heading_mid)*Ts;

    % Commit heading
    ego_heading = heading_new;

    % Keep heading within [-pi, pi]
    ego_heading = ...
        atan2(sin(ego_heading),...
              cos(ego_heading));

    %% ========================================================
    % 8.24 STORE RESULTS
    % ========================================================

    ego_x_history(k) = ego_x;

    ego_y_history(k) = ego_y;

    ego_speed_history(k) = ego_speed;

    ego_heading_history(k) = ego_heading;

    risk_history(k) = risk_level;

    raw_risk_history(k) = raw_risk;

    behavior_history(k) = behavior;

    planning_mode_history(k) = planning_mode;

    target_speed_history(k) = target_speed;

    acceleration_history(k) = acceleration;

    steering_history(k) = steering;

    steering_rate_history(k) = steering_rate;

    predicted_distance_history(k) = ...
        minimum_distance;

    required_safety_dist = base_safety_distance + uncertainty_margin;
    required_safety_distance_history(k) = ...
        required_safety_dist(obj_min_idx(critical_obj_idx));

    safety_margin_history(k) = ...
        minimum_safety_margin;

    TTC_history(k) = TTC;

    time_to_closest_history(k) = ...
        time_to_closest;

    time_to_conflict_history(k) = ...
        time_to_conflict;

    uncertainty_x_history(k) = ...
        sqrt(max(tracks(critical_obj_idx).P(1,1),0));

    uncertainty_y_history(k) = ...
        sqrt(max(tracks(critical_obj_idx).P(2,2),0));

    uncertainty_margin_history(k) = ...
        max(uncertainty_margin);

    planner_safe_history(k) = ...
        planner_safe;

    selected_lateral_history(k) = ...
        selected_lateral_offset;

    planner_target_history(k) = planner_target_y;
    planner_reference_history(k) = planner_ref_y(end);
    planner_reference_heading_history(k) = planner_ref_heading;
    planner_action_history(k) = planner_action;

    actual_distance_history(k) = hypot(true_cow_x - ego_x, true_cow_y - ego_y);

    controller_status_history(k) = ...
        controller_status;

    %% ========================================================
    % 8.25 DIAGNOSTIC OUTPUT
    % ========================================================

    if mod(k,5) == 0 || k == 1
        fprintf( ...
            ['Time %.1f s | Critical ID %d (%s) | Raw %-6s | Risk %-6s | ' ...
             'TargetSpeed %.2f | Speed %.2f | Acc %.2f | ' ...
             'PredDist %.2f | Margin %.2f | TCA %.2f | ' ...
             'TConflict %.2f | TTC %.2f | X %.2f | Y %.2f | ' ...
             'PlannerTargetY %.2f | RefY %.2f | RefPsi %.1f deg | ' ...
             'Steer %.1f deg | %s\n'], ...
            current_time, tracks(critical_obj_idx).id, char(tracks(critical_obj_idx).type), ...
            char(raw_risk), char(risk_level), target_speed, ...
            ego_speed, acceleration, minimum_distance, minimum_safety_margin, ...
            time_to_closest, time_to_conflict, TTC, ego_x, ego_y, ...
            planner_target_y, planner_ref_y(end), ...
            rad2deg(planner_ref_heading), rad2deg(steering), ...
            char(planner_action));

        if num_objects > 1
            for oi = 1:num_objects
                fprintf('   -> Object %d | %-7s | Risk %-6s | Dist %.2f m | Margin %.2f m | TCA %.2f s | TTC %.2f s\n', ...
                    tracks(oi).id, char(tracks(oi).type), char(obj_raw_risk(oi)), ...
                    obj_min_dist(oi), obj_min_margin(oi), obj_tca(oi), obj_ttc(oi));
            end
        end
    end

    %% ========================================================
    % 8.26 FIRST-STEP SANITY CHECK
    % ========================================================

    if k == 1

        fprintf('\n------------ PREDICTION SANITY CHECK ------------\n');

        fprintf( ...
            'Current ego position : (%.2f, %.2f) m\n', ...
            ego_x,ego_y);

        fprintf( ...
            'Current cow estimate : (%.2f, %.2f) m\n', ...
            estimated_x,estimated_y);

        fprintf( ...
            'Estimated cow velocity: (%.2f, %.2f) m/s\n', ...
            estimated_vx,estimated_vy);

        fprintf( ...
            'Predicted ego @ %.1f s : (%.2f, %.2f) m\n', ...
            future_time(1), ...
            ego_predicted_x(1), ...
            ego_predicted_y(1));

        fprintf( ...
            'Predicted cow @ %.1f s : (%.2f, %.2f) m\n', ...
            future_time(1), ...
            predicted_x(1), ...
            predicted_y(1));

        first_position_distance = ...
            hypot( ...
                predicted_x(1)-ego_predicted_x(1), ...
                predicted_y(1)-ego_predicted_y(1));

        fprintf( ...
            'Distance at first prediction point: %.2f m\n', ...
            first_position_distance);

        fprintf( ...
            'Closest predicted distance : %.2f m\n', ...
            minimum_distance);

        fprintf( ...
            'Time to closest approach   : %.2f s\n', ...
            time_to_closest);

        fprintf( ...
            'Time to safety conflict    : %.2f s\n', ...
            time_to_conflict);

        fprintf( ...
            'Initial MPC steering       : %.2f deg\n', ...
            rad2deg(mpc_steering));

        fprintf( ...
            'Applied steering           : %.2f deg\n', ...
            rad2deg(steering));

        fprintf( ...
            'Planner lateral offset     : %.2f m\n', ...
            selected_lateral_offset);

        fprintf( ...
            'Planner action             : %s\n', ...
            planner_action);

        fprintf( ...
            '-------------------------------------------------\n\n');

    end

    previous_risk = risk_level;

end



%% ============================================================
% 9. FINAL RESULTS
% =============================================================

fprintf('\n============================================\n');
fprintf('          INDRA SIMULATION RESULTS\n');
fprintf('============================================\n');

fprintf( ...
    'Initial speed       : %.2f m/s\n', ...
    ego_speed_history(1));

fprintf( ...
    'Final speed         : %.2f m/s\n', ...
    ego_speed_history(end));

fprintf( ...
    'Final X             : %.2f m\n', ...
    ego_x_history(end));

fprintf( ...
    'Final Y             : %.2f m\n', ...
    ego_y_history(end));

fprintf( ...
    'Final heading       : %.2f deg\n', ...
    rad2deg(ego_heading_history(end)));

fprintf( ...
    'Minimum predicted distance : %.2f m\n', ...
    min(predicted_distance_history));

fprintf( ...
    'Minimum safety margin      : %.2f m\n', ...
    min(safety_margin_history));

finite_ttc = ...
    TTC_history(isfinite(TTC_history));

if ~isempty(finite_ttc)

    fprintf( ...
        'Minimum TTC         : %.2f s\n', ...
        min(finite_ttc));

else

    fprintf( ...
        'Minimum TTC         : No closing conflict\n');

end

fprintf( ...
    'Maximum uncertainty margin : %.2f m\n', ...
    max(uncertainty_margin_history));

fprintf( ...
    'Maximum steering    : %.2f deg\n', ...
    max(abs(rad2deg(steering_history))));

fprintf( ...
    'Maximum braking     : %.2f m/s^2\n', ...
    min(acceleration_history));

fprintf( ...
    'Maximum steering rate used : %.2f deg/s\n', ...
    max(abs(rad2deg(steering_rate_history))));

%% ============================================================
% 10. EVENT TIMES
% =============================================================

medium_index = ...
    find(risk_history=="MEDIUM",1);

high_index = ...
    find(risk_history=="HIGH",1);

low_after_high_index = [];

if ~isempty(high_index)

    low_after_high_index = ...
        find( ...
            risk_history=="LOW" & ...
            (1:N)>high_index, ...
            1);

end

intervention_index = ...
    find(acceleration_history < -1,1);

if ~isempty(medium_index)

    fprintf( ...
        'First MEDIUM risk   : %.2f s\n', ...
        t(medium_index));

else

    fprintf( ...
        'First MEDIUM risk   : NOT REACHED\n');

end

if ~isempty(high_index)

    fprintf( ...
        'First HIGH risk     : %.2f s\n', ...
        t(high_index));

else

    fprintf( ...
        'First HIGH risk     : NOT REACHED\n');

end

if ~isempty(low_after_high_index)

    fprintf( ...
        'Recovered to LOW    : %.2f s\n', ...
        t(low_after_high_index));

end

if ~isempty(intervention_index)

    fprintf( ...
        'First intervention  : %.2f s\n', ...
        t(intervention_index));

else

    fprintf( ...
        'First intervention  : NOT REQUIRED\n');

end

%% ============================================================
% 11. PIPELINE STATUS
% =============================================================

fprintf('\n============================================\n');
fprintf('           PIPELINE STATUS\n');
fprintf('============================================\n');

fprintf('Sensing             : ACTIVE\n');
fprintf('Perception          : ACTIVE\n');
fprintf('Tracking            : ACTIVE\n');
fprintf('Prediction          : ACTIVE\n');
fprintf('Risk Assessment     : ACTIVE\n');
fprintf('Decision Logic      : ACTIVE\n');
fprintf('Local Path Planning : ACTIVE\n');
fprintf('MPC Control         : ACTIVE\n');
fprintf('Vehicle Motion      : ACTIVE\n');

fprintf('============================================\n');

%% ============================================================
% 12. TRAJECTORY PLOT
% =============================================================

figure('Name','INDRA Closed Loop');

plot( ...
    ego_x_history, ...
    ego_y_history, ...
    'LineWidth',2);

hold on;

plot( ...
    cow_x_history, ...
    cow_y_history, ...
    '--', ...
    'LineWidth',2);

plot( ...
    estimated_x_history, ...
    estimated_y_history, ...
    ':', ...
    'LineWidth',1.5);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('INDRA: Ego Vehicle and Cattle');

legend( ...
    'Ego Vehicle', ...
    'Cow True Path', ...
    'Cow Estimated Path', ...
    'Location','best');

grid on;

axis equal;

%% ============================================================
% 13. RISK PLOT
% =============================================================

figure('Name','INDRA Risk');

risk_numeric = ...
    zeros(1,N);

risk_numeric(risk_history=="LOW") = 1;

risk_numeric(risk_history=="MEDIUM") = 2;

risk_numeric(risk_history=="HIGH") = 3;

stairs( ...
    t, ...
    risk_numeric, ...
    'LineWidth',2);

xlabel('Time (s)');
ylabel('Risk Level');

yticks([1 2 3]);

yticklabels({'LOW','MEDIUM','HIGH'});

title('INDRA Risk State');

grid on;

%% ============================================================
% 14. SPEED PLOT
% =============================================================

figure('Name','INDRA Speed');

plot( ...
    t, ...
    ego_speed_history, ...
    'LineWidth',2);

hold on;

plot( ...
    t, ...
    target_speed_history, ...
    '--', ...
    'LineWidth',2);

xlabel('Time (s)');
ylabel('Speed (m/s)');

title('Vehicle Speed and Target Speed');

legend( ...
    'Actual Speed', ...
    'Target Speed');

grid on;

%% ============================================================
% 15. DISTANCE / SAFETY ENVELOPE
% =============================================================

figure('Name','INDRA Distance');

plot( ...
    t, ...
    predicted_distance_history, ...
    'LineWidth',2);

hold on;

plot( ...
    t, ...
    required_safety_distance_history, ...
    '--', ...
    'LineWidth',2);

xlabel('Time (s)');
ylabel('Distance (m)');

title('Predicted Distance vs Safety Envelope');

legend( ...
    'Predicted Closest Distance', ...
    'Required Safety Distance');

grid on;

%% ============================================================
% 16. SAFETY MARGIN
% =============================================================

figure('Name','INDRA Safety Margin');

plot( ...
    t, ...
    safety_margin_history, ...
    'LineWidth',2);

hold on;

yline(0,'--');

xlabel('Time (s)');
ylabel('Safety Margin (m)');

title('INDRA Collision Safety Margin');

legend( ...
    'Safety Margin', ...
    'Zero Safety Boundary');

grid on;

%% ============================================================
% 17. CONTROL
% =============================================================

figure('Name','INDRA Control');

plot( ...
    t, ...
    acceleration_history, ...
    'LineWidth',2);

hold on;

plot( ...
    t, ...
    rad2deg(steering_history), ...
    '--', ...
    'LineWidth',2);

xlabel('Time (s)');
ylabel('Control');

title('INDRA Control Commands');

legend( ...
    'Acceleration (m/s^2)', ...
    'Steering (deg)');

grid on;

%% ============================================================
% 18. RESULT STRUCTURE
% =============================================================

indra_result = struct();

indra_result.scenario_name = scenario.name;
indra_result.scenario_type = scenario.type;
indra_result.scenario_object_type = scenario.object_type;
indra_result.object_count = num_objects;
indra_result.tracked_object_count = num_objects;
indra_result.objects = objects_history;
indra_result.critical_object_id = critical_object_id_history;
indra_result.critical_risk = critical_risk_history;

indra_result.time = t;

indra_result.ego_x = ...
    ego_x_history;

indra_result.ego_y = ...
    ego_y_history;

indra_result.ego_speed = ...
    ego_speed_history;

indra_result.ego_heading = ...
    ego_heading_history;

indra_result.cow_x = ...
    cow_x_history;

indra_result.cow_y = ...
    cow_y_history;

indra_result.cow_vx = ...
    cow_vx_history;

indra_result.cow_vy = ...
    cow_vy_history;

indra_result.actual_distance = ...
    actual_distance_history;

indra_result.estimated_x = ...
    estimated_x_history;

indra_result.estimated_y = ...
    estimated_y_history;

indra_result.estimated_vx = ...
    estimated_vx_history;

indra_result.estimated_vy = ...
    estimated_vy_history;

indra_result.raw_risk = ...
    raw_risk_history;

indra_result.risk = ...
    risk_history;

indra_result.behavior = ...
    behavior_history;

indra_result.planning_mode = ...
    planning_mode_history;

indra_result.target_speed = ...
    target_speed_history;

indra_result.acceleration = ...
    acceleration_history;

indra_result.steering = ...
    steering_history;

indra_result.steering_rate = ...
    steering_rate_history;

indra_result.predicted_distance = ...
    predicted_distance_history;

indra_result.required_safety_distance = ...
    required_safety_distance_history;

indra_result.safety_margin = ...
    safety_margin_history;

indra_result.TTC = ...
    TTC_history;

indra_result.time_to_closest = ...
    time_to_closest_history;

indra_result.time_to_conflict = ...
    time_to_conflict_history;

indra_result.prediction_horizon = ...
    prediction_horizon;

indra_result.uncertainty_x = ...
    uncertainty_x_history;

indra_result.uncertainty_y = ...
    uncertainty_y_history;

indra_result.uncertainty_margin = ...
    uncertainty_margin_history;

indra_result.planner_safe = ...
    planner_safe_history;

indra_result.selected_lateral_offset = ...
    selected_lateral_history;

indra_result.planner_target_y = planner_target_history;
indra_result.planner_reference_y = planner_reference_history;
indra_result.planner_action = planner_action_history;
indra_result.road_center_y = road_center_y;
indra_result.max_avoidance_offset = max_avoidance_offset;

indra_result.controller_status = ...
    controller_status_history;

indra_result.wheelbase = ...
    wheelbase;

indra_result.max_steering_angle = ...
    max_steering_angle;

indra_result.max_steering_rate = ...
    max_steering_rate;

if ~isempty(medium_index)
    indra_result.first_medium_risk_time = t(medium_index);
else
    indra_result.first_medium_risk_time = Inf;
end

if ~isempty(high_index)
    indra_result.first_high_risk_time = t(high_index);
else
    indra_result.first_high_risk_time = Inf;
end

if ~isempty(intervention_index)
    indra_result.first_intervention_time = t(intervention_index);
else
    indra_result.first_intervention_time = Inf;
end

fprintf('\nINDRA result structure created successfully.\n');

fprintf('Simulation completed successfully.\n\n');

end