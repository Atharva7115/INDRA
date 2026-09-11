%% INDRA - Constant Velocity (CV) Motion Prediction
%
% Pipeline:
%
% Sensor Measurements
%        ↓
%       EKF
%        ↓
%   Current State
%        ↓
%   CV Prediction
%        ↓
% Future Trajectory
%
% State:
% [X; Y; Vx; Vy]

clear;
clc;
close all;

%% =========================================================
% 1. LOAD / RUN EKF TRACKING
% ==========================================================

% Run the existing EKF script.
%
% This creates:
%   t
%   estimated_state
%   cow_x
%   cow_y

run('D:\INDRA\tracking\ekf_tracking.m');

%% =========================================================
% 2. PREDICTION PARAMETERS
% ==========================================================

% Prediction horizon
prediction_horizon = 2.0;       % seconds

% Prediction time step
prediction_dt = 0.1;            % seconds

prediction_time = ...
    0:prediction_dt:prediction_horizon;

%% =========================================================
% 3. SELECT CURRENT TRACKING STATE
% ==========================================================

% We predict from the moment when the cow is close
% to the ego vehicle's path.

prediction_start_time = 3.0;

% Find nearest EKF sample

[~, start_idx] = ...
    min(abs(t - prediction_start_time));

% Extract EKF state

X0 = estimated_state(1,start_idx);
Y0 = estimated_state(2,start_idx);

Vx0 = estimated_state(3,start_idx);
Vy0 = estimated_state(4,start_idx);

%% =========================================================
% 4. DISPLAY CURRENT STATE
% ==========================================================

fprintf('\n');
fprintf('INDRA CV MOTION PREDICTION\n');
fprintf('=============================================\n');

fprintf('Prediction start time = %.2f s\n', ...
    t(start_idx));

fprintf('\nCurrent EKF state:\n');

fprintf('X  = %.2f m\n', X0);
fprintf('Y  = %.2f m\n', Y0);
fprintf('Vx = %.2f m/s\n', Vx0);
fprintf('Vy = %.2f m/s\n', Vy0);

%% =========================================================
% 5. CONSTANT VELOCITY MODEL
% ==========================================================
%
% X(t) = X0 + Vx0*t
%
% Y(t) = Y0 + Vy0*t
%
% Velocity remains constant over the short prediction
% horizon.

predicted_X = ...
    X0 + Vx0 .* prediction_time;

predicted_Y = ...
    Y0 + Vy0 .* prediction_time;

%% =========================================================
% 6. PREDICTED VELOCITY
% ==========================================================

predicted_Vx = ...
    Vx0 .* ones(size(prediction_time));

predicted_Vy = ...
    Vy0 .* ones(size(prediction_time));

%% =========================================================
% 7. ACTUAL FUTURE TRAJECTORY
% ==========================================================

% Calculate the actual cow position after the
% prediction start time.

actual_X = ...
    35 .* ones(size(prediction_time));

actual_Y = ...
    7 - 2 .* (prediction_start_time + prediction_time);

%% =========================================================
% 8. PREDICTION ERROR
% ==========================================================

position_error = sqrt( ...
    (predicted_X - actual_X).^2 + ...
    (predicted_Y - actual_Y).^2);

%% =========================================================
% 9. DISPLAY PREDICTION TABLE
% ==========================================================

fprintf('\n');
fprintf('FUTURE TRAJECTORY PREDICTION\n');
fprintf('---------------------------------------------\n');

fprintf( ...
    'Time(s)     Pred_X(m)     Pred_Y(m)     Error(m)\n');

fprintf( ...
    '---------------------------------------------\n');

for k = 1:length(prediction_time)

    fprintf( ...
        '%5.1f       %8.2f       %8.2f       %7.2f\n', ...
        prediction_time(k), ...
        predicted_X(k), ...
        predicted_Y(k), ...
        position_error(k));

end

%% =========================================================
% 10. PREDICTION PLOT
% ==========================================================

figure('Name','INDRA - CV Motion Prediction');

% Actual trajectory

plot( ...
    actual_X, ...
    actual_Y, ...
    'r--', ...
    'LineWidth', 2);

hold on;

% Predicted trajectory

plot( ...
    predicted_X, ...
    predicted_Y, ...
    'b-', ...
    'LineWidth', 2);

% Current position

plot( ...
    X0, ...
    Y0, ...
    'ko', ...
    'MarkerSize', 8, ...
    'LineWidth', 2);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title( ...
    sprintf( ...
    'INDRA - CV Prediction from t = %.1f s', ...
    prediction_start_time));

legend( ...
    'Actual Future Trajectory', ...
    'CV Predicted Trajectory', ...
    'Current EKF State');

grid on;
axis equal;

%% =========================================================
% 11. PREDICTION ERROR PLOT
% ==========================================================

figure('Name','INDRA - Prediction Error');

plot( ...
    prediction_time, ...
    position_error, ...
    'b-', ...
    'LineWidth', 2);

xlabel('Prediction Horizon (s)');
ylabel('Position Error (m)');

title('INDRA - CV Prediction Error');

grid on;

%% =========================================================
% 12. FINAL PREDICTION
% ==========================================================

fprintf('\n');
fprintf('FINAL PREDICTION\n');
fprintf('---------------------------------------------\n');

fprintf( ...
    'At +%.1f s:\n', ...
    prediction_horizon);

fprintf( ...
    'Predicted X = %.2f m\n', ...
    predicted_X(end));

fprintf( ...
    'Predicted Y = %.2f m\n', ...
    predicted_Y(end));

fprintf( ...
    'Prediction Error = %.2f m\n', ...
    position_error(end));