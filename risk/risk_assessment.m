%% INDRA - Risk Assessment and Time-to-Collision
%
% Pipeline:
%
% EKF Tracking
%      ↓
% CV Prediction
%      ↓
% Future Object Trajectory
%      ↓
% Risk Assessment
%      ↓
% TTC + Predicted Conflict
%
% Scenario:
% Ego vehicle approaches a cow crossing the road.

clear;
clc;
close all;

%% =========================================================
% 1. RUN EKF TRACKING
% ==========================================================

run('D:\INDRA\tracking\ekf_tracking.m');

%% =========================================================
% 2. RISK ANALYSIS TIME
% ==========================================================

analysis_time = 3.0;

[~, start_idx] = ...
    min(abs(t - analysis_time));

%% =========================================================
% 3. GET CURRENT EKF STATE
% ==========================================================

X0 = estimated_state(1,start_idx);
Y0 = estimated_state(2,start_idx);

Vx0 = estimated_state(3,start_idx);
Vy0 = estimated_state(4,start_idx);

fprintf('\n');
fprintf('INDRA RISK ASSESSMENT\n');
fprintf('=============================================\n');

fprintf('Analysis time = %.2f s\n', ...
    t(start_idx));

fprintf('\nCurrent tracked object state:\n');

fprintf('X  = %.2f m\n', X0);
fprintf('Y  = %.2f m\n', Y0);
fprintf('Vx = %.2f m/s\n', Vx0);
fprintf('Vy = %.2f m/s\n', Vy0);

%% =========================================================
% 4. PREDICTION PARAMETERS
% ==========================================================

prediction_horizon = 2.0;
prediction_dt = 0.05;

prediction_time = ...
    0:prediction_dt:prediction_horizon;

%% =========================================================
% 5. PREDICT COW TRAJECTORY
% ==========================================================

predicted_cow_x = ...
    X0 + Vx0 .* prediction_time;

predicted_cow_y = ...
    Y0 + Vy0 .* prediction_time;

%% =========================================================
% 6. PREDICT EGO TRAJECTORY
% ==========================================================

ego_speed = 10;                 % m/s

% Ego starts at its position at analysis time

ego_x0 = ego_speed * analysis_time;
ego_y0 = 0;

predicted_ego_x = ...
    ego_x0 + ego_speed .* prediction_time;

predicted_ego_y = ...
    zeros(size(prediction_time));

%% =========================================================
% 7. PREDICTED SEPARATION
% ==========================================================

relative_x = ...
    predicted_cow_x - predicted_ego_x;

relative_y = ...
    predicted_cow_y - predicted_ego_y;

predicted_distance = sqrt( ...
    relative_x.^2 + ...
    relative_y.^2);

%% =========================================================
% 8. TTC CALCULATION
% ==========================================================
%
% TTC is estimated using longitudinal closing motion.
%
% Closing speed:
%
% ego speed - object longitudinal velocity
%
% TTC = longitudinal distance / closing speed
%
% Only calculate TTC when the ego vehicle is closing
% on the object.

closing_speed = ...
    ego_speed - Vx0;

longitudinal_distance = X0 - ego_x0;

if closing_speed > 0 && longitudinal_distance > 0

    TTC = longitudinal_distance / closing_speed;

else

    TTC = Inf;

end

%% =========================================================
% 9. PREDICTED MINIMUM DISTANCE
% ==========================================================

[min_distance, min_idx] = ...
    min(predicted_distance);

time_at_min_distance = ...
    prediction_time(min_idx);

%% =========================================================
% 10. CONFLICT DETECTION
% ==========================================================

% Approximate vehicle/object safety radius

safety_distance = 2.0;          % metres

% Check whether predicted paths come within
% the safety distance.

predicted_conflict = ...
    min_distance <= safety_distance;

%% =========================================================
% 11. RISK LEVEL
% ==========================================================

if TTC < 1.0 || ...
        min_distance < 1.0

    risk_level = "HIGH";

elseif TTC < 2.0 || ...
        min_distance < safety_distance

    risk_level = "MEDIUM";

else

    risk_level = "LOW";

end

%% =========================================================
% 12. DISPLAY TTC
% ==========================================================

fprintf('\n');
fprintf('RISK METRICS\n');
fprintf('---------------------------------------------\n');

fprintf('Longitudinal distance = %.2f m\n', ...
    longitudinal_distance);

fprintf('Closing speed         = %.2f m/s\n', ...
    closing_speed);

fprintf('TTC                   = %.2f s\n', ...
    TTC);

fprintf('Minimum predicted distance = %.2f m\n', ...
    min_distance);

fprintf('Time of minimum distance   = %.2f s\n', ...
    time_at_min_distance);

%% =========================================================
% 13. DISPLAY CONFLICT
% ==========================================================

fprintf('\n');
fprintf('PREDICTED CONFLICT\n');
fprintf('---------------------------------------------\n');

if predicted_conflict

    fprintf('YES - Predicted paths enter safety distance.\n');

else

    fprintf('NO - Predicted paths remain separated.\n');

end

%% =========================================================
% 14. FINAL RISK DECISION
% ==========================================================

fprintf('\n');
fprintf('INDRA RISK DECISION\n');
fprintf('---------------------------------------------\n');

fprintf('Risk Level = %s\n', risk_level);

if risk_level == "HIGH"

    fprintf('Decision = EMERGENCY STOP / AVOID\n');

elseif risk_level == "MEDIUM"

    fprintf('Decision = SLOW DOWN / PREPARE TO YIELD\n');

else

    fprintf('Decision = CONTINUE\n');

end

%% =========================================================
% 15. TRAJECTORY VISUALIZATION
% ==========================================================

figure('Name','INDRA - Risk Assessment');

plot( ...
    predicted_ego_x, ...
    predicted_ego_y, ...
    'b-', ...
    'LineWidth', 3);

hold on;

plot( ...
    predicted_cow_x, ...
    predicted_cow_y, ...
    'r--', ...
    'LineWidth', 3);

% Current positions

plot( ...
    ego_x0, ...
    ego_y0, ...
    'bo', ...
    'MarkerSize', 9, ...
    'LineWidth', 2);

plot( ...
    X0, ...
    Y0, ...
    'ro', ...
    'MarkerSize', 9, ...
    'LineWidth', 2);

% Minimum distance point

plot( ...
    predicted_cow_x(min_idx), ...
    predicted_cow_y(min_idx), ...
    'ko', ...
    'MarkerSize', 10, ...
    'LineWidth', 2);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title( ...
    sprintf( ...
    'INDRA - Risk Assessment | TTC = %.2f s | Risk = %s', ...
    TTC, risk_level));

legend( ...
    'Predicted Ego Path', ...
    'Predicted Cow Path', ...
    'Current Ego Position', ...
    'Current Cow Position', ...
    'Minimum Distance');

grid on;
axis equal;

%% =========================================================
% 16. DISTANCE OVER PREDICTION HORIZON
% ==========================================================

figure('Name','INDRA - Predicted Distance');

plot( ...
    prediction_time, ...
    predicted_distance, ...
    'b-', ...
    'LineWidth', 2);

hold on;

yline( ...
    safety_distance, ...
    'r--', ...
    'Safety Distance');

xlabel('Prediction Horizon (s)');
ylabel('Object-Ego Distance (m)');

title('INDRA - Predicted Object-Ego Separation');

legend( ...
    'Predicted Distance', ...
    'Safety Distance');

grid on;