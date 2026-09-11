%% INDRA - UNCERTAINTY AWARE RISK ASSESSMENT
% ---------------------------------------------------------
% Uses:
%   1. EKF estimated state
%   2. EKF covariance
%   3. Constant Velocity prediction
%   4. TTC
%   5. Predicted minimum distance
%   6. Uncertainty-aware safety margin
%
% Scenario:
%   Ego vehicle approaching a cow crossing the road
%
% NOTE:
%   This is an uncertainty-aware safety heuristic.
%   It is NOT a formal RSS implementation.
% ---------------------------------------------------------

clear;
clc;
close all;

fprintf('\n');
fprintf('=====================================================\n');
fprintf('     INDRA UNCERTAINTY-AWARE RISK ASSESSMENT\n');
fprintf('=====================================================\n');

%% =====================================================
% 1. RUN EKF TRACKING
% ======================================================

fprintf('\nRunning EKF tracking...\n');

run('D:\INDRA\tracking\ekf_tracking.m');

fprintf('EKF tracking completed.\n');

%% =====================================================
% 2. SELECT ANALYSIS TIME
% ======================================================

analysis_time = 3.0;

[~, analysis_index] = min(abs(t - analysis_time));

actual_analysis_time = t(analysis_index);

%% =====================================================
% 3. GET CURRENT EKF STATE
% ======================================================

x0 = estimated_state(:, analysis_index);

X0  = x0(1);
Y0  = x0(2);
Vx0 = x0(3);
Vy0 = x0(4);

%% =====================================================
% 4. GET CURRENT EKF COVARIANCE
% ======================================================

P0 = estimated_covariance(:, :, analysis_index);

% Position covariance
P_position = P0(1:2, 1:2);

sigma_x0 = sqrt(max(P_position(1,1), 0));
sigma_y0 = sqrt(max(P_position(2,2), 0));

fprintf('\n');
fprintf('-----------------------------------------------------\n');
fprintf('CURRENT EKF STATE\n');
fprintf('-----------------------------------------------------\n');

fprintf('Analysis time       = %.2f s\n', actual_analysis_time);
fprintf('X                   = %.2f m\n', X0);
fprintf('Y                   = %.2f m\n', Y0);
fprintf('Vx                  = %.2f m/s\n', Vx0);
fprintf('Vy                  = %.2f m/s\n', Vy0);

fprintf('\nCurrent uncertainty:\n');
fprintf('Sigma X             = %.3f m\n', sigma_x0);
fprintf('Sigma Y             = %.3f m\n', sigma_y0);

%% =====================================================
% 5. PREDICTION SETTINGS
% ======================================================

prediction_horizon = 2.0;
prediction_dt = 0.1;

prediction_time = 0:prediction_dt:prediction_horizon;

N = length(prediction_time);

%% =====================================================
% 6. CONSTANT VELOCITY PREDICTION
% ======================================================

predicted_X = zeros(1, N);
predicted_Y = zeros(1, N);

for k = 1:N

    tau = prediction_time(k);

    predicted_X(k) = X0 + Vx0 * tau;
    predicted_Y(k) = Y0 + Vy0 * tau;

end

%% =====================================================
% 7. PREDICT EGO VEHICLE TRAJECTORY
% ======================================================

ego_speed = 10;       % m/s

ego_X0 = ego_x(analysis_index);
ego_Y0 = ego_y(analysis_index);

predicted_ego_X = ego_X0 + ego_speed .* prediction_time;
predicted_ego_Y = ego_Y0 + 0 .* prediction_time;

%% =====================================================
% 8. PROPAGATE POSITION UNCERTAINTY
% ======================================================

% Constant velocity state transition
F_prediction = [1 0 prediction_dt 0;
                0 1 0 prediction_dt;
                0 0 1 0;
                0 0 0 1];

% Small process noise for future prediction
Q_prediction = [0.01 0    0    0;
                0    0.01 0    0;
                0    0    0.05 0;
                0    0    0    0.05];

P_prediction = P0;

sigma_x = zeros(1, N);
sigma_y = zeros(1, N);

for k = 1:N

    if k > 1
        P_prediction = F_prediction * ...
                       P_prediction * ...
                       F_prediction' + ...
                       Q_prediction;
    end

    sigma_x(k) = sqrt(max(P_prediction(1,1), 0));
    sigma_y(k) = sqrt(max(P_prediction(2,2), 0));

end

%% =====================================================
% 9. UNCERTAINTY MARGIN
% ======================================================

% 2-sigma approximately represents a conservative
% uncertainty boundary around the predicted object.

uncertainty_margin = 2 .* sqrt(sigma_x.^2 + sigma_y.^2);

% Base safety distance
base_safety_distance = 2.0;

% Effective safety distance
effective_safety_distance = ...
    base_safety_distance + uncertainty_margin;

%% =====================================================
% 10. PREDICTED SEPARATION
% ======================================================

predicted_distance = sqrt( ...
    (predicted_X - predicted_ego_X).^2 + ...
    (predicted_Y - predicted_ego_Y).^2 );

%% =====================================================
% 11. UNCERTAINTY-AWARE SEPARATION
% ======================================================

% Distance after accounting for uncertainty
uncertainty_adjusted_distance = ...
    predicted_distance - uncertainty_margin;

%% =====================================================
% 12. MINIMUM PREDICTED DISTANCE
% ======================================================

[min_distance, min_index] = min(predicted_distance);

time_of_min_distance = prediction_time(min_index);

%% =====================================================
% 13. MINIMUM UNCERTAINTY-ADJUSTED DISTANCE
% ======================================================

[min_adjusted_distance, adjusted_index] = ...
    min(uncertainty_adjusted_distance);

time_of_adjusted_minimum = ...
    prediction_time(adjusted_index);

%% =====================================================
% 14. TTC CALCULATION
% ======================================================

longitudinal_distance = X0 - ego_X0;

relative_velocity_x = ego_speed - Vx0;

if relative_velocity_x > 0

    TTC = longitudinal_distance / relative_velocity_x;

else

    TTC = Inf;

end

%% =====================================================
% 15. UNCERTAINTY CONFLICT CHECK
% ======================================================

uncertainty_conflict = false;

for k = 1:N

    if predicted_distance(k) <= ...
            effective_safety_distance(k)

        uncertainty_conflict = true;
        break;

    end

end

%% =====================================================
% 16. BASIC CONFLICT CHECK
% ======================================================

basic_conflict = min_distance <= base_safety_distance;

%% =====================================================
% 17. RISK CLASSIFICATION
% ======================================================

if TTC < 1.0 || ...
        min_adjusted_distance <= 0

    risk_level = "HIGH";

    decision = "EMERGENCY STOP / AVOID";

elseif TTC < 2.0 || ...
        uncertainty_conflict

    risk_level = "MEDIUM";

    decision = "SLOW DOWN / PREPARE TO YIELD";

else

    risk_level = "LOW";

    decision = "CONTINUE";

end

%% =====================================================
% 18. PRINT RESULTS
% ======================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('        UNCERTAINTY-AWARE RISK RESULTS\n');
fprintf('=====================================================\n');

fprintf('\nAnalysis time              = %.2f s\n', ...
    actual_analysis_time);

fprintf('Longitudinal distance      = %.2f m\n', ...
    longitudinal_distance);

fprintf('Closing speed              = %.2f m/s\n', ...
    relative_velocity_x);

fprintf('TTC                        = %.2f s\n', TTC);

fprintf('\nPrediction results:\n');

fprintf('Minimum predicted distance = %.2f m\n', ...
    min_distance);

fprintf('Time of minimum distance   = %.2f s\n', ...
    time_of_min_distance);

fprintf('\nUncertainty:\n');

fprintf('Initial Sigma X            = %.3f m\n', ...
    sigma_x0);

fprintf('Initial Sigma Y            = %.3f m\n', ...
    sigma_y0);

fprintf('Final Sigma X              = %.3f m\n', ...
    sigma_x(end));

fprintf('Final Sigma Y              = %.3f m\n', ...
    sigma_y(end));

fprintf('Maximum uncertainty margin = %.3f m\n', ...
    max(uncertainty_margin));

fprintf('\nSafety:\n');

fprintf('Base safety distance       = %.2f m\n', ...
    base_safety_distance);

fprintf('Minimum adjusted distance = %.2f m\n', ...
    min_adjusted_distance);

fprintf('Time of adjusted minimum  = %.2f s\n', ...
    time_of_adjusted_minimum);

if basic_conflict
    fprintf('Basic predicted conflict   = YES\n');
else
    fprintf('Basic predicted conflict   = NO\n');
end

if uncertainty_conflict
    fprintf('Uncertainty conflict       = YES\n');
else
    fprintf('Uncertainty conflict       = NO\n');
end

fprintf('\n');
fprintf('-----------------------------------------------------\n');
fprintf('FINAL RISK LEVEL = %s\n', risk_level);
fprintf('DECISION          = %s\n', decision);
fprintf('-----------------------------------------------------\n');

%% =====================================================
% 19. PLOT 1 - PREDICTED TRAJECTORIES
% ======================================================

figure;

plot(predicted_ego_X, predicted_ego_Y, ...
    'LineWidth', 2);

hold on;

plot(predicted_X, predicted_Y, ...
    'LineWidth', 2);

plot(predicted_X(min_index), ...
     predicted_Y(min_index), ...
     'o', ...
     'MarkerSize', 8, ...
     'LineWidth', 2);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('INDRA - Predicted Ego and Cow Trajectories');

legend('Ego Vehicle', ...
       'Predicted Cow', ...
       'Minimum Distance', ...
       'Location', 'best');

grid on;
axis equal;

%% =====================================================
% 20. PLOT 2 - UNCERTAINTY GROWTH
% ======================================================

figure;

plot(prediction_time, sigma_x, ...
    'LineWidth', 2);

hold on;

plot(prediction_time, sigma_y, ...
    'LineWidth', 2);

xlabel('Prediction Time (s)');
ylabel('Position Uncertainty (m)');

title('INDRA - Predicted Position Uncertainty');

legend('Sigma X', ...
       'Sigma Y', ...
       'Location', 'best');

grid on;

%% =====================================================
% 21. PLOT 3 - SAFETY DISTANCE
% ======================================================

figure;

plot(prediction_time, predicted_distance, ...
    'LineWidth', 2);

hold on;

plot(prediction_time, effective_safety_distance, ...
    'LineWidth', 2);

xlabel('Prediction Time (s)');
ylabel('Distance (m)');

title('INDRA - Uncertainty-Aware Safety Boundary');

legend('Predicted Distance', ...
       'Effective Safety Distance', ...
       'Location', 'best');

grid on;

%% =====================================================
% 22. FINAL SUMMARY
% ======================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('                  INDRA SUMMARY\n');
fprintf('=====================================================\n');

fprintf('Tracking              : EKF\n');
fprintf('Prediction            : Constant Velocity\n');
fprintf('Uncertainty            : EKF covariance + 2-sigma\n');
fprintf('Risk indicators        : TTC + predicted distance\n');
fprintf('Risk level             : %s\n', risk_level);
fprintf('Autonomous decision    : %s\n', decision);

fprintf('=====================================================\n');