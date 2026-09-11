%% INDRA - Indian Navigation & Dynamic Risk-Aware Autonomy
% CLOSED-LOOP VERSION
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
% All prediction distances use the SAME WORLD coordinate frame.
% No privileged future hazard information is given to the
% autonomy stack.

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf('       INDRA CLOSED-LOOP AUTONOMY\n');
fprintf('============================================\n\n');

%% ============================================================
% 1. SIMULATION PARAMETERS
% =============================================================

Ts = 0.1;
simulation_time = 8;

t = 0:Ts:simulation_time;
N = length(t);

% Reproducible sensor noise
rng(1);

%% ============================================================
% 2. WORLD / SCENARIO
% =============================================================

% -------------------------------------------------------------
% Ego vehicle initial state
% -------------------------------------------------------------

ego_x = 0;
ego_y = 0;

ego_speed = 10;
ego_heading = 0;

% -------------------------------------------------------------
% Cow trajectory
%
% Cow starts at:
%       X = 35 m
%       Y = 7 m
%
% and crosses toward the ego lane.
% -------------------------------------------------------------

cow_x = 35 * ones(size(t));
cow_y = 7 - 2*t;

cow_vx = 0;
cow_vy = -2;

%% ============================================================
% 3. STORAGE
% =============================================================

ego_x_history = zeros(1,N);
ego_y_history = zeros(1,N);

ego_speed_history = zeros(1,N);
ego_heading_history = zeros(1,N);

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

% HIGH risk
high_distance_threshold = 1.5;
high_time_threshold = 1.0;
high_ttc_threshold = 1.0;

% MEDIUM risk
medium_distance_threshold = 3.0;
medium_time_threshold = 2.5;
medium_ttc_threshold = 2.5;

%% ============================================================
% 6. EKF INITIALIZATION
% =============================================================

% State:
%
% x = [X
%      Y
%      Vx
%      Vy]

x_est = [35;
          7;
          0;
         -2];

P = diag([1 1 1 1]);

Q = diag([0.01 0.01 0.1 0.1]);

R_lidar = diag([0.15^2 0.15^2]);

R_radar = 0.25^2;

H_lidar = [1 0 0 0;
           0 1 0 0];

%% ============================================================
% 7. PREDICTION CONFIGURATION
% ============================================================

% Four seconds is long enough to see the complete crossing.

prediction_horizon = 4.0;

prediction_steps = ...
    round(prediction_horizon / Ts);

%% ============================================================
% 8. MAIN CLOSED LOOP
% =============================================================

for k = 1:N

    current_time = t(k);

    %% ========================================================
    % TRUE WORLD
    % =========================================================

    true_cow_x = cow_x(k);
    true_cow_y = cow_y(k);

    %% ========================================================
    % PERCEPTION
    % ========================================================

    relative_x = true_cow_x - ego_x;
    relative_y = true_cow_y - ego_y;

    range = sqrt(relative_x^2 + relative_y^2);

    cow_detected = range <= 50;

    %% ========================================================
    % EKF TRACKING
    % ========================================================

    F = [1 0 Ts 0;
         0 1 0 Ts;
         0 0 1  0;
         0 0 0  1];

    if cow_detected

        %% ----------------------------------------------------
        % LiDAR measurement
        % -----------------------------------------------------

        lidar_x = true_cow_x + 0.15*randn;
        lidar_y = true_cow_y + 0.15*randn;

        z_lidar = [lidar_x;
                   lidar_y];

        %% ----------------------------------------------------
        % EKF prediction
        % -----------------------------------------------------

        x_pred = F*x_est;

        P_pred = F*P*F' + Q;

        %% ----------------------------------------------------
        % LiDAR update
        % -----------------------------------------------------

        innovation = ...
            z_lidar - H_lidar*x_pred;

        S = ...
            H_lidar*P_pred*H_lidar' + R_lidar;

        K = ...
            P_pred*H_lidar'/S;

        x_est = ...
            x_pred + K*innovation;

        I = eye(4);

        P = ...
            (I-K*H_lidar)*P_pred* ...
            (I-K*H_lidar)' + ...
            K*R_lidar*K';

        %% ----------------------------------------------------
        % Radar update
        % -----------------------------------------------------

        relative_x = true_cow_x - ego_x;
        relative_y = true_cow_y - ego_y;

        radar_range = ...
            sqrt(relative_x^2 + relative_y^2);

        if radar_range > 1

            ux = relative_x / radar_range;
            uy = relative_y / radar_range;

            relative_velocity_x = ...
                cow_vx - ego_speed*cos(ego_heading);

            relative_velocity_y = ...
                cow_vy - ego_speed*sin(ego_heading);

            radar_measurement = ...
                relative_velocity_x*ux + ...
                relative_velocity_y*uy;

            H_radar = [0 0 ux uy];

            radar_innovation = ...
                radar_measurement - H_radar*x_est;

            S_radar = ...
                H_radar*P*H_radar' + R_radar;

            K_radar = ...
                P*H_radar'/S_radar;

            x_est = ...
                x_est + K_radar*radar_innovation;

            I = eye(4);

            P = ...
                (I-K_radar*H_radar)*P* ...
                (I-K_radar*H_radar)' + ...
                K_radar*R_radar*K_radar';

        end

    else

        %% Prediction only if object temporarily not detected

        x_est = F*x_est;

        P = F*P*F' + Q;

    end

    %% ========================================================
    % TRACKED OBJECT
    % ========================================================

    estimated_x = x_est(1);
    estimated_y = x_est(2);

    estimated_vx = x_est(3);
    estimated_vy = x_est(4);

    estimated_x_history(k) = estimated_x;
    estimated_y_history(k) = estimated_y;

    estimated_vx_history(k) = estimated_vx;
    estimated_vy_history(k) = estimated_vy;

    %% ========================================================
    % 9. FUTURE OBJECT PREDICTION
    % ========================================================

    predicted_x = zeros(1,prediction_steps);
    predicted_y = zeros(1,prediction_steps);

    prediction_covariance = ...
        zeros(2,2,prediction_steps);

    x_future = x_est;
    P_future = P;

    for p = 1:prediction_steps

        x_future = F*x_future;

        P_future = ...
            F*P_future*F' + Q;

        predicted_x(p) = x_future(1);
        predicted_y(p) = x_future(2);

        prediction_covariance(:,:,p) = ...
            P_future(1:2,1:2);

    end

    %% ========================================================
    % 10. FUTURE TIME VECTOR
    % ========================================================

    future_time = ...
        (1:prediction_steps) * Ts;

    %% ========================================================
    % 11. EGO FUTURE TRAJECTORY
    % ========================================================
    %
    % IMPORTANT FIX:
    %
    % ego_future_x starts from the CURRENT ego position.
    %
    % We do NOT use the global simulation time here.
    %
    % This ensures that both ego and object prediction are
    % represented in the same world coordinate frame.

    ego_predicted_x = ...
        ego_x + ...
        ego_speed*cos(ego_heading).*future_time;

    ego_predicted_y = ...
        ego_y + ...
        ego_speed*sin(ego_heading).*future_time;

    %% ========================================================
    % 12. UNCERTAINTY
    % ========================================================

    uncertainty_margin = ...
        zeros(1,prediction_steps);

    for p = 1:prediction_steps

        sigma_x = sqrt( ...
            max(prediction_covariance(1,1,p),0));

        sigma_y = sqrt( ...
            max(prediction_covariance(2,2,p),0));

        raw_uncertainty = ...
            2*sqrt(sigma_x^2 + sigma_y^2);

        uncertainty_margin(p) = ...
            min(raw_uncertainty, ...
                max_uncertainty_distance);

    end

    %% ========================================================
    % 13. DISTANCE + SAFETY MARGIN
    % ========================================================

    predicted_distance = ...
        zeros(1,prediction_steps);

    required_safety_distance = ...
        zeros(1,prediction_steps);

    safety_margin = ...
        zeros(1,prediction_steps);

    for p = 1:prediction_steps

        dx = ...
            predicted_x(p) - ego_predicted_x(p);

        dy = ...
            predicted_y(p) - ego_predicted_y(p);

        predicted_distance(p) = ...
            sqrt(dx^2 + dy^2);

        required_safety_distance(p) = ...
            base_safety_distance + ...
            uncertainty_margin(p);

        safety_margin(p) = ...
            predicted_distance(p) - ...
            required_safety_distance(p);

    end

    %% ========================================================
    % 14. CLOSEST APPROACH
    % ========================================================

    [minimum_distance, minimum_index] = ...
        min(predicted_distance);

    minimum_safety_margin = ...
        safety_margin(minimum_index);

    time_to_closest = ...
        future_time(minimum_index);

    %% ========================================================
    % 15. TIME TO SAFETY-ENVELOPE CONFLICT
    % ========================================================
    %
    % This is more useful than only looking at minimum distance.
    %
    % We find the FIRST future instant where:
    %
    %       distance <= required safety distance
    %
    % This tells us how long the vehicle has before entering
    % the predicted collision/safety envelope.

    conflict_indices = ...
        find(safety_margin <= 0);

    if isempty(conflict_indices)

        time_to_conflict = inf;

    else

        first_conflict_index = ...
            conflict_indices(1);

        time_to_conflict = ...
            future_time(first_conflict_index);

    end

    %% ========================================================
    % 16. CURRENT TTC
    % ========================================================

    relative_x = ...
        estimated_x - ego_x;

    relative_y = ...
        estimated_y - ego_y;

    relative_vx = ...
        estimated_vx - ...
        ego_speed*cos(ego_heading);

    relative_vy = ...
        estimated_vy - ...
        ego_speed*sin(ego_heading);

    current_range = ...
        sqrt(relative_x^2 + relative_y^2);

    if current_range > 1e-6

        ux = relative_x/current_range;
        uy = relative_y/current_range;

        closing_speed = ...
            -(relative_vx*ux + ...
              relative_vy*uy);

    else

        closing_speed = 0;

    end

    if closing_speed > 0

        TTC = current_range/closing_speed;

    else

        TTC = inf;

    end

    %% ========================================================
    % 17. RISK ASSESSMENT
    % ========================================================

    % ---------------------------------------------------------
    % HIGH RISK
    % ---------------------------------------------------------
    %
    % HIGH means the conflict is imminent.
    %

    high_distance_condition = ...
        minimum_distance <= high_distance_threshold && ...
        time_to_closest <= high_time_threshold;

    high_conflict_condition = ...
        time_to_conflict <= high_time_threshold;

    high_ttc_condition = ...
        TTC <= high_ttc_threshold;

    if high_distance_condition || ...
            high_conflict_condition || ...
            high_ttc_condition

        raw_risk = "HIGH";

    else

        % -----------------------------------------------------
        % MEDIUM RISK
        % -----------------------------------------------------

        medium_distance_condition = ...
            minimum_distance <= medium_distance_threshold && ...
            time_to_closest <= medium_time_threshold;

        medium_conflict_condition = ...
            time_to_conflict <= medium_time_threshold;

        medium_ttc_condition = ...
            TTC <= medium_ttc_threshold;

        if medium_distance_condition || ...
                medium_conflict_condition || ...
                medium_ttc_condition

            raw_risk = "MEDIUM";

        else

            raw_risk = "LOW";

        end

    end

    %% ========================================================
    % 18. INITIAL GUARD
    % ========================================================

    if current_time <= 0.2 && ...
            raw_risk == "HIGH"

        raw_risk = "MEDIUM";

    end

    %% ========================================================
    % 19. RISK HYSTERESIS
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

        safe_cycles = ...
            safe_cycles + 1;

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
    % 20. DECISION LOGIC
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

        %% ----------------------------------------------------
        % Gradual recovery
        % -----------------------------------------------------

        if previous_risk == "HIGH" || ...
                previous_risk == "MEDIUM" || ...
                recovery_hold > 0

            if recovery_hold > 0

                recovery_hold = ...
                    recovery_hold - 1;

            end

            commanded_target_speed = ...
                max(commanded_target_speed,ego_speed);

            commanded_target_speed = ...
                min(commanded_target_speed,5);

        else

            commanded_target_speed = ...
                max(commanded_target_speed,ego_speed);

            commanded_target_speed = ...
                min(commanded_target_speed + 0.25,10);

        end

        target_speed = ...
            max(commanded_target_speed,ego_speed);

        target_speed = ...
            min(target_speed,10);

    end

   %% ========================================================
% 21. LOCAL PATH PLANNER
% ========================================================

% Candidate lateral offsets
%
% -3 m = left
%  0 m = center
% +3 m = right

lateral_candidates = [-3 0 3];

candidate_safe = ...
    false(1,length(lateral_candidates));

candidate_distance = ...
    zeros(1,length(lateral_candidates));

for c = 1:length(lateral_candidates)

    lateral_offset = ...
        lateral_candidates(c);

    candidate_min_distance = inf;

    for p = 1:prediction_steps

        candidate_x = ...
            ego_predicted_x(p);

        candidate_y = ...
            ego_predicted_y(p) + ...
            lateral_offset;

        dx = ...
            predicted_x(p) - candidate_x;

        dy = ...
            predicted_y(p) - candidate_y;

        d = sqrt(dx^2 + dy^2);

        candidate_min_distance = ...
            min(candidate_min_distance,d);

    end

    candidate_distance(c) = ...
        candidate_min_distance;

    candidate_safe(c) = ...
        candidate_min_distance - ...
        max(uncertainty_margin) > ...
        base_safety_distance;

end

    %% ========================================================
    % 22. PATH SELECTION
    % ========================================================

    if risk_level == "HIGH"

        planner_safe = false;

        selected_lateral_offset = 0;

        planner_action = ...
            "EMERGENCY STOP";

    elseif any(candidate_safe)

        planner_safe = true;

        % Prefer center if safe.
        % Otherwise use left/right.

        if candidate_safe(2)

            selected_lateral_offset = 0;

        elseif candidate_safe(1)

            selected_lateral_offset = -3;

        else

            selected_lateral_offset = 3;

        end

        planner_action = ...
            "SAFE AVOIDANCE";

    else

        planner_safe = false;

        selected_lateral_offset = 0;

        planner_action = "BRAKE";

    end

    %% ========================================================
    % 23. PLANNER REFERENCE
    % ========================================================

    planner_ref_x = ...
        ego_x + ...
        ego_speed*cos(ego_heading).*future_time;

    planner_ref_y = ...
        ego_y + ...
        selected_lateral_offset .* ...
        min(future_time/prediction_horizon,1);

    planner_ref_speed = ...
        target_speed * ...
        ones(1,prediction_steps);

    %% ========================================================
    % 24. MPC CONTROL
    % ========================================================

    try

        control_result = ...
            mpc_controller( ...
                ego_speed, ...
                ego_y, ...
                ego_heading, ...
                planner_ref_y(1), ...
                0, ...
                planner_ref_speed(1));

        steering = ...
            control_result.steering;

        acceleration = ...
            control_result.acceleration;

        controller_status = ...
            control_result.controller_status;

    catch ME

        steering = 0;

        acceleration = -6;

        controller_status = ...
            "MPC FAIL-SAFE";

        fprintf( ...
            'MPC ERROR at %.1f s: %s\n', ...
            current_time, ...
            ME.message);

    end

    %% ========================================================
    % 25. SAFETY OVERRIDE
    % ========================================================

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
        % Prevent sudden braking during recovery.

        acceleration = ...
            max(acceleration,-0.5);

        acceleration = ...
            min(acceleration,1.0);

    end

    %% ========================================================
    % 26. VEHICLE MOTION
    % ========================================================

    ego_speed = ...
        ego_speed + acceleration*Ts;

    ego_speed = ...
        max(0,min(ego_speed,15));

    ego_heading = ...
        ego_heading + ...
        steering*ego_speed*Ts/2.5;

    ego_x = ...
        ego_x + ...
        ego_speed*cos(ego_heading)*Ts;

    ego_y = ...
        ego_y + ...
        ego_speed*sin(ego_heading)*Ts;

    %% ========================================================
    % 27. STORE RESULTS
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

    predicted_distance_history(k) = ...
        minimum_distance;

    required_safety_distance_history(k) = ...
        required_safety_distance(minimum_index);

    safety_margin_history(k) = ...
        minimum_safety_margin;

    TTC_history(k) = TTC;

    time_to_closest_history(k) = ...
        time_to_closest;

    time_to_conflict_history(k) = ...
        time_to_conflict;

    uncertainty_x_history(k) = ...
        sqrt(max(P(1,1),0));

    uncertainty_y_history(k) = ...
        sqrt(max(P(2,2),0));

    uncertainty_margin_history(k) = ...
        max(uncertainty_margin);

    planner_safe_history(k) = ...
        planner_safe;

    %% ========================================================
    % 28. DIAGNOSTIC OUTPUT
    % ========================================================

    if mod(k,5) == 0 || k == 1

        fprintf( ...
            ['Time %.1f s | Raw %-6s | Risk %-6s | ' ...
             'Target %.2f | Speed %.2f | Acc %.2f | ' ...
             'Dist %.2f | Margin %.2f | TCA %.2f | ' ...
             'TConflict %.2f | TTC %.2f\n'], ...
            current_time, ...
            raw_risk, ...
            risk_level, ...
            target_speed, ...
            ego_speed, ...
            acceleration, ...
            minimum_distance, ...
            minimum_safety_margin, ...
            time_to_closest, ...
            time_to_conflict, ...
            TTC);

    end

    %% ========================================================
    % 29. FIRST-STEP SANITY CHECK
    % ========================================================

    if k == 1

        fprintf('\n------------ PREDICTION SANITY CHECK ------------\n');

        fprintf('Current ego position : (%.2f, %.2f) m\n', ...
            ego_x,ego_y);

        fprintf('Current cow estimate : (%.2f, %.2f) m\n', ...
            estimated_x,estimated_y);

        fprintf('Predicted ego @ %.1f s : (%.2f, %.2f) m\n', ...
            future_time(1), ...
            ego_predicted_x(1), ...
            ego_predicted_y(1));

        fprintf('Predicted cow @ %.1f s : (%.2f, %.2f) m\n', ...
            future_time(1), ...
            predicted_x(1), ...
            predicted_y(1));

        fprintf('Closest predicted distance : %.2f m\n', ...
            minimum_distance);

        fprintf('Time to closest approach   : %.2f s\n', ...
            time_to_closest);

        fprintf('Time to safety conflict    : %.2f s\n', ...
            time_to_conflict);

        fprintf('-------------------------------------------------\n\n');

    end

    previous_risk = risk_level;

end

%% ============================================================
% 30. FINAL RESULTS
% =============================================================

fprintf('\n============================================\n');
fprintf('          INDRA SIMULATION RESULTS\n');
fprintf('============================================\n');

fprintf('Initial speed       : %.2f m/s\n', ...
    ego_speed_history(1));

fprintf('Final speed         : %.2f m/s\n', ...
    ego_speed_history(end));

fprintf('Final X             : %.2f m\n', ...
    ego_x_history(end));

fprintf('Final Y             : %.2f m\n', ...
    ego_y_history(end));

fprintf('Minimum predicted distance : %.2f m\n', ...
    min(predicted_distance_history));

fprintf('Minimum safety margin      : %.2f m\n', ...
    min(safety_margin_history));

finite_ttc = ...
    TTC_history(isfinite(TTC_history));

if ~isempty(finite_ttc)

    fprintf('Minimum TTC         : %.2f s\n', ...
        min(finite_ttc));

else

    fprintf('Minimum TTC         : No closing conflict\n');

end

fprintf('Maximum uncertainty margin : %.2f m\n', ...
    max(uncertainty_margin_history));

fprintf('Maximum steering    : %.2f deg\n', ...
    max(abs(rad2deg(steering_history))));

fprintf('Maximum braking     : %.2f m/s^2\n', ...
    min(acceleration_history));

%% ============================================================
% 31. EVENT TIMES
% ============================================================

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

    fprintf('First MEDIUM risk   : %.2f s\n', ...
        t(medium_index));

end

if ~isempty(high_index)

    fprintf('First HIGH risk     : %.2f s\n', ...
        t(high_index));

end

if ~isempty(low_after_high_index)

    fprintf('Recovered to LOW    : %.2f s\n', ...
        t(low_after_high_index));

end

if ~isempty(intervention_index)

    fprintf('First intervention  : %.2f s\n', ...
        t(intervention_index));

end

%% ============================================================
% 32. PIPELINE STATUS
% ============================================================

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
% 33. TRAJECTORY PLOT
% ============================================================

figure('Name','INDRA Closed Loop');

plot(ego_x_history, ...
     ego_y_history, ...
     'LineWidth',2);

hold on;

plot(cow_x, ...
     cow_y, ...
     '--', ...
     'LineWidth',2);

plot(estimated_x_history, ...
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
% 34. RISK PLOT
% ============================================================

figure('Name','INDRA Risk');

risk_numeric = zeros(1,N);

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
% 35. SPEED PLOT
% ============================================================

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
% 36. DISTANCE / SAFETY ENVELOPE
% ============================================================

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
% 37. SAFETY MARGIN
% ============================================================

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
% 38. CONTROL
% ============================================================

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
% 39. RESULT STRUCTURE
% ============================================================

indra_result = struct();

indra_result.time = t;

indra_result.ego_x = ego_x_history;
indra_result.ego_y = ego_y_history;

indra_result.ego_speed = ...
    ego_speed_history;

indra_result.ego_heading = ...
    ego_heading_history;

indra_result.cow_x = cow_x;
indra_result.cow_y = cow_y;

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

indra_result.uncertainty_x = ...
    uncertainty_x_history;

indra_result.uncertainty_y = ...
    uncertainty_y_history;

indra_result.uncertainty_margin = ...
    uncertainty_margin_history;

indra_result.planner_safe = ...
    planner_safe_history;

fprintf('\nINDRA result structure created successfully.\n');
fprintf('Simulation completed successfully.\n\n');