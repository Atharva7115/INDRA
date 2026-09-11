%% INDRA - LOCAL PATH PLANNER
% ---------------------------------------------------------
% Adaptive local planner for Indian-road obstacle avoidance.
%
% Candidate maneuvers:
%   1. LEFT AVOID
%   2. CENTER
%   3. RIGHT AVOID
%   4. BRAKE / STOP
%
% Each maneuver is evaluated using:
%   - Predicted obstacle distance
%   - Uncertainty margin
%   - Vehicle safety distance
%   - Path deviation
%   - Smoothness
%   - Braking feasibility
%
% Output:
%   planner_result structure
% ---------------------------------------------------------

clear;
clc;
close all;

fprintf('\n');
fprintf('=====================================================\n');
fprintf('              INDRA LOCAL PATH PLANNER\n');
fprintf('=====================================================\n');

%% =====================================================
% 1. RUN UNCERTAINTY-AWARE RISK ASSESSMENT
% ======================================================

fprintf('\nRunning risk assessment...\n');

run('D:\INDRA\risk\risk_uncertainty.m');

fprintf('Risk assessment completed.\n');

%% =====================================================
% 2. CURRENT EGO STATE
% ======================================================

ego_X0 = ego_x(analysis_index);
ego_Y0 = ego_y(analysis_index);

ego_speed = 10.0;

%% =====================================================
% 3. CURRENT OBSTACLE STATE
% ======================================================

object_X0 = X0;
object_Y0 = Y0;

object_Vx = Vx0;
object_Vy = Vy0;

%% =====================================================
% 4. PLANNING PARAMETERS
% ======================================================

planning_horizon = 3.0;
planning_dt = 0.1;

planning_time = 0:planning_dt:planning_horizon;

N = length(planning_time);

% Road boundaries
road_left_boundary = 4.0;
road_right_boundary = -4.0;

% Lateral candidate offsets
candidate_offsets = [-3.0, 0.0, 3.0];

candidate_names = [
    "LEFT AVOID"
    "CENTER"
    "RIGHT AVOID"
    "BRAKE / STOP"
];

num_candidates = 4;

% Safety parameters
vehicle_radius = 1.0;
base_safety_distance = 2.0;

% Maximum comfortable braking
comfortable_deceleration = 5.0;   % m/s^2

%% =====================================================
% 5. PREDICT OBSTACLE TRAJECTORY
% ======================================================

object_predicted_X = ...
    object_X0 + object_Vx .* planning_time;

object_predicted_Y = ...
    object_Y0 + object_Vy .* planning_time;

%% =====================================================
% 6. PROPAGATE UNCERTAINTY
% ======================================================

P_plan = P0;

F_plan = [1 0 planning_dt 0;
          0 1 0 planning_dt;
          0 0 1 0;
          0 0 0 1];

Q_plan = [0.01 0    0    0;
          0    0.01 0    0;
          0    0    0.05 0;
          0    0    0    0.05];

sigma_x_plan = zeros(1,N);
sigma_y_plan = zeros(1,N);

for k = 1:N

    if k > 1

        P_plan = F_plan * ...
                 P_plan * ...
                 F_plan' + ...
                 Q_plan;

    end

    sigma_x_plan(k) = ...
        sqrt(max(P_plan(1,1),0));

    sigma_y_plan(k) = ...
        sqrt(max(P_plan(2,2),0));

end

% 2-sigma uncertainty margin
uncertainty_margin = ...
    2 .* sqrt(sigma_x_plan.^2 + ...
              sigma_y_plan.^2);

%% =====================================================
% 7. GENERATE CANDIDATE PATHS
% ======================================================

candidate_X = zeros(num_candidates,N);
candidate_Y = zeros(num_candidates,N);

candidate_speed = zeros(num_candidates,N);

shift_duration = 1.5;

%% -----------------------------------------------------
% LEFT / CENTER / RIGHT
% ------------------------------------------------------

for c = 1:3

    lateral_offset = candidate_offsets(c);

    for k = 1:N

        tau = planning_time(k);

        % Forward movement
        candidate_X(c,k) = ...
            ego_X0 + ego_speed * tau;

        % Smooth lateral movement
        if tau <= shift_duration

            s = tau / shift_duration;

            % Cubic smoothstep
            smooth_shift = ...
                3*s^2 - 2*s^3;

            candidate_Y(c,k) = ...
                ego_Y0 + ...
                lateral_offset * smooth_shift;

        else

            candidate_Y(c,k) = ...
                ego_Y0 + lateral_offset;

        end

        candidate_speed(c,k) = ego_speed;

    end

end

%% -----------------------------------------------------
% BRAKING / STOP TRAJECTORY
% ------------------------------------------------------

brake_index = 4;

for k = 1:N

    tau = planning_time(k);

    % Constant deceleration model
    braking_distance = ...
        ego_speed * tau - ...
        0.5 * comfortable_deceleration * tau^2;

    % Vehicle cannot move backward
    braking_distance = max(braking_distance,0);

    candidate_X(brake_index,k) = ...
        ego_X0 + braking_distance;

    candidate_Y(brake_index,k) = ...
        ego_Y0;

    candidate_speed(brake_index,k) = ...
        max(ego_speed - ...
        comfortable_deceleration * tau,0);

end

%% =====================================================
% 8. EVALUATE CANDIDATES
% ======================================================

minimum_distance = ...
    zeros(1,num_candidates);

minimum_adjusted_distance = ...
    zeros(1,num_candidates);

path_deviation = ...
    zeros(1,num_candidates);

smoothness_cost = ...
    zeros(1,num_candidates);

collision_flag = ...
    false(1,num_candidates);

total_cost = ...
    zeros(1,num_candidates);

%% Safety distance

effective_safety_distance = ...
    base_safety_distance + vehicle_radius;

%% =====================================================
% 9. PATH EVALUATION
% ======================================================

for c = 1:num_candidates

    %% -----------------------------------------------
    % Distance to predicted obstacle
    % -----------------------------------------------

    distance = sqrt( ...
        (candidate_X(c,:) - object_predicted_X).^2 + ...
        (candidate_Y(c,:) - object_predicted_Y).^2);

    %% -----------------------------------------------
    % Uncertainty-adjusted distance
    % -----------------------------------------------

    adjusted_distance = ...
        distance - uncertainty_margin;

    minimum_distance(c) = ...
        min(distance);

    minimum_adjusted_distance(c) = ...
        min(adjusted_distance);

    %% -----------------------------------------------
    % Safety check
    % -----------------------------------------------

    if any(distance <= ...
            effective_safety_distance + ...
            uncertainty_margin)

        collision_flag(c) = true;

    end

    %% -----------------------------------------------
    % Road boundary check
    % -----------------------------------------------

    if c <= 3

        max_y = max(candidate_Y(c,:));
        min_y = min(candidate_Y(c,:));

        if max_y > road_left_boundary || ...
           min_y < road_right_boundary

            collision_flag(c) = true;

        end

    end

    %% -----------------------------------------------
    % Path deviation
    % -----------------------------------------------

    path_deviation(c) = ...
        mean(abs(candidate_Y(c,:) - ego_Y0));

    %% -----------------------------------------------
    % Smoothness
    % -----------------------------------------------

    first_difference = ...
        diff(candidate_Y(c,:));

    second_difference = ...
        diff(first_difference);

    if isempty(second_difference)

        smoothness_cost(c) = 0;

    else

        smoothness_cost(c) = ...
            mean(abs(second_difference));

    end

end

%% =====================================================
% 10. SPECIAL BRAKING FEASIBILITY
% ======================================================

% Required stopping distance from current speed
required_stopping_distance = ...
    ego_speed^2 / ...
    (2 * comfortable_deceleration);

% Distance available to obstacle along X
available_distance = ...
    object_X0 - ego_X0;

fprintf('\n');
fprintf('-----------------------------------------------------\n');
fprintf('BRAKING FEASIBILITY\n');
fprintf('-----------------------------------------------------\n');

fprintf('Current ego speed       = %.2f m/s\n', ...
    ego_speed);

fprintf('Available distance      = %.2f m\n', ...
    available_distance);

fprintf('Required stopping dist. = %.2f m\n', ...
    required_stopping_distance);

if available_distance >= ...
        required_stopping_distance

    braking_feasible = true;

else

    braking_feasible = false;

end

if braking_feasible

    fprintf('Braking maneuver        = FEASIBLE\n');

else

    fprintf('Braking maneuver        = NOT FEASIBLE\n');

end

%% =====================================================
% 11. OVERRIDE BRAKING SAFETY RESULT
% ======================================================

if braking_feasible

    % Check the actual braking trajectory.
    brake_distance = sqrt( ...
        (candidate_X(4,:) - object_predicted_X).^2 + ...
        (candidate_Y(4,:) - object_predicted_Y).^2);

    brake_adjusted_distance = ...
        brake_distance - uncertainty_margin;

    minimum_distance(4) = ...
        min(brake_distance);

    minimum_adjusted_distance(4) = ...
        min(brake_adjusted_distance);

    % Braking is safe only if the trajectory remains
    % outside the uncertainty-aware safety boundary.

    if any(brake_distance <= ...
            effective_safety_distance + ...
            uncertainty_margin)

        collision_flag(4) = true;

    else

        collision_flag(4) = false;

    end

else

    collision_flag(4) = true;

end

%% =====================================================
% 12. CALCULATE COSTS
% ======================================================

for c = 1:num_candidates

    if collision_flag(c)

        collision_cost = 10000;

    else

        collision_cost = 0;

    end

    % Lower cost = preferred path.
    total_cost(c) = ...
        collision_cost + ...
        10 * path_deviation(c) + ...
        100 * smoothness_cost(c);

    % Prefer braking slightly over unnecessary
    % lateral movement when both are safe.
    if c == 4 && ~collision_flag(c)

        total_cost(c) = ...
            total_cost(c) - 5;

    end

end

%% =====================================================
% 13. SELECT SAFE MANEUVER
% ======================================================

safe_candidates = ...
    find(~collision_flag);

if isempty(safe_candidates)

    selected_candidate = 0;

    selected_path_name = ...
        "NO SAFE MANEUVER - EMERGENCY STOP";

    selected_target_speed = 0;

    selected_X = ...
        candidate_X(4,:);

    selected_Y = ...
        candidate_Y(4,:);

    selected_speed = ...
        zeros(size(planning_time));

else

    [~,best_index] = ...
        min(total_cost(safe_candidates));

    selected_candidate = ...
        safe_candidates(best_index);

    selected_path_name = ...
        candidate_names(selected_candidate);

    selected_target_speed = ...
        candidate_speed(selected_candidate,end);

    selected_X = ...
        candidate_X(selected_candidate,:);

    selected_Y = ...
        candidate_Y(selected_candidate,:);

    selected_speed = ...
        candidate_speed(selected_candidate,:);

end

%% =====================================================
% 14. PRINT CANDIDATE RESULTS
% ======================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('             CANDIDATE MANEUVER RESULTS\n');
fprintf('=====================================================\n');

for c = 1:num_candidates

    fprintf('\nManeuver: %s\n', ...
        candidate_names(c));

    fprintf('Minimum distance          = %.2f m\n', ...
        minimum_distance(c));

    fprintf('Minimum adjusted distance = %.2f m\n', ...
        minimum_adjusted_distance(c));

    fprintf('Path deviation            = %.2f\n', ...
        path_deviation(c));

    fprintf('Smoothness cost           = %.4f\n', ...
        smoothness_cost(c));

    fprintf('Total cost                = %.2f\n', ...
        total_cost(c));

    if collision_flag(c)

        fprintf('Safety                    = UNSAFE\n');

    else

        fprintf('Safety                    = SAFE\n');

    end

end

%% =====================================================
% 15. FINAL PLANNER RESULT
% ======================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('              INDRA PLANNING RESULT\n');
fprintf('=====================================================\n');

if selected_candidate == 0

    fprintf('Selected Maneuver : NO SAFE MANEUVER\n');
    fprintf('Target Speed      : 0.00 m/s\n');
    fprintf('Action            : EMERGENCY STOP\n');

elseif selected_candidate == 4

    fprintf('Selected Maneuver : BRAKE / STOP\n');
    fprintf('Target Speed      : %.2f m/s\n', ...
        selected_target_speed);

    fprintf('Action            : CONTROLLED BRAKING\n');

else

    fprintf('Selected Maneuver : %s\n', ...
        selected_path_name);

    fprintf('Target Speed      : %.2f m/s\n', ...
        selected_target_speed);

    fprintf('Action            : LATERAL AVOIDANCE\n');

end

fprintf('=====================================================\n');

%% =====================================================
% 16. CREATE OUTPUT STRUCTURE
% ======================================================

planner_result = struct();

planner_result.candidate_names = ...
    candidate_names;

planner_result.minimum_distance = ...
    minimum_distance;

planner_result.minimum_adjusted_distance = ...
    minimum_adjusted_distance;

planner_result.path_deviation = ...
    path_deviation;

planner_result.smoothness_cost = ...
    smoothness_cost;

planner_result.total_cost = ...
    total_cost;

planner_result.collision_flag = ...
    collision_flag;

planner_result.selected_candidate = ...
    selected_candidate;

planner_result.selected_path_name = ...
    selected_path_name;

planner_result.selected_target_speed = ...
    selected_target_speed;

planner_result.planning_time = ...
    planning_time;

planner_result.selected_X = ...
    selected_X;

planner_result.selected_Y = ...
    selected_Y;

planner_result.selected_speed = ...
    selected_speed;

planner_result.braking_feasible = ...
    braking_feasible;

planner_result.required_stopping_distance = ...
    required_stopping_distance;

%% =====================================================
% 17. PLOT CANDIDATE MANEUVERS
% ======================================================

figure;

hold on;

% Plot candidate maneuvers
for c = 1:num_candidates

    if collision_flag(c)

        plot(candidate_X(c,:), ...
             candidate_Y(c,:), ...
             '--', ...
             'LineWidth',1.5);

    else

        plot(candidate_X(c,:), ...
             candidate_Y(c,:), ...
             'LineWidth',1.5);

    end

end

% Predicted obstacle trajectory
plot(object_predicted_X, ...
     object_predicted_Y, ...
     'k--', ...
     'LineWidth',2);

% Ego starting position
plot(ego_X0, ...
     ego_Y0, ...
     'ko', ...
     'MarkerSize',8, ...
     'LineWidth',2);

% Selected maneuver
if selected_candidate ~= 0

    plot(selected_X, ...
         selected_Y, ...
         'LineWidth',3);

end

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('INDRA - Adaptive Local Path Planning');

% Explicit handles prevent legend warnings
h1 = plot(nan,nan,'--','LineWidth',1.5);
h2 = plot(nan,nan,'--','LineWidth',1.5);
h3 = plot(nan,nan,'--','LineWidth',1.5);
h4 = plot(nan,nan,'--','LineWidth',1.5);
h5 = plot(nan,nan,'k--','LineWidth',2);
h6 = plot(nan,nan,'ko','MarkerSize',8,'LineWidth',2);

if selected_candidate ~= 0
    h7 = plot(nan,nan,'-','LineWidth',3);

    legend([h1 h2 h3 h4 h5 h6 h7], ...
        {'Left Candidate', ...
         'Center Candidate', ...
         'Right Candidate', ...
         'Brake / Stop', ...
         'Predicted Cow', ...
         'Ego Vehicle', ...
         'Selected Maneuver'}, ...
         'Location','best');
else
    legend([h1 h2 h3 h4 h5 h6], ...
        {'Left Candidate', ...
         'Center Candidate', ...
         'Right Candidate', ...
         'Brake / Stop', ...
         'Predicted Cow', ...
         'Ego Vehicle'}, ...
         'Location','best');
end

grid on;
axis equal;

%% =====================================================
% 18. PLOT SELECTED MANEUVER SPEED
% ======================================================

figure;

plot(planning_time, ...
     selected_speed, ...
     'LineWidth',2);

xlabel('Planning Time (s)');
ylabel('Vehicle Speed (m/s)');

title('INDRA - Planned Vehicle Speed');

grid on;

%% =====================================================
% 19. PLOT MANEUVER COST
% ======================================================

figure;

bar(total_cost);

xlabel('Maneuver');

ylabel('Total Cost');

title('INDRA - Maneuver Cost');

xticks(1:num_candidates);

xticklabels(candidate_names);

grid on;

%% =====================================================
% 20. FINAL SUMMARY
% ======================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('                  INDRA SUMMARY\n');
fprintf('=====================================================\n');

fprintf('Risk Level          : %s\n', ...
    risk_level);

if selected_candidate == 0

    fprintf('Planning Result     : EMERGENCY STOP\n');

else

    fprintf('Planning Result     : %s\n', ...
        selected_path_name);

end

fprintf('Braking Feasible    : %s\n', ...
    string(braking_feasible));

fprintf('=====================================================\n');