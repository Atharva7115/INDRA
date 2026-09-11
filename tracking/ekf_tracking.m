%% INDRA - EKF Sensor Fusion and Tracking
%
% State:
%   x = [X; Y; Vx; Vy]
%
% Sensors:
%   Camera -> object detection and bearing
%   LiDAR  -> object X,Y position
%   Radar  -> relative radial velocity
%
% Scenario:
%   Ego vehicle drives forward.
%   Cow crosses the road at X = 35 m.
%
% Coordinate system:
%   X = longitudinal road direction
%   Y = lateral road direction

clear;
clc;
close all;

rng(1);

%% =========================================================
% 1. SIMULATION PARAMETERS
% ==========================================================

dt = 0.05;
T = 8;

t = 0:dt:T;

%% =========================================================
% 2. TRUE WORLD
% ==========================================================

% ----------------------------------------------------------
% Ego vehicle
% ----------------------------------------------------------

ego_speed = 10;                 % m/s

ego_x = ego_speed .* t;
ego_y = zeros(size(t));

% ----------------------------------------------------------
% Cow
% ----------------------------------------------------------

cow_x = 35 * ones(size(t));

cow_start_y = 7;
cow_speed = -2;                 % m/s

cow_y = cow_start_y + cow_speed .* t;

% True cow velocity

cow_vx = zeros(size(t));
cow_vy = cow_speed .* ones(size(t));

%% =========================================================
% 3. RELATIVE POSITION
% ==========================================================

% Object position relative to ego vehicle

rel_x = cow_x - ego_x;
rel_y = cow_y - ego_y;

true_range = sqrt(rel_x.^2 + rel_y.^2);

%% =========================================================
% 4. CAMERA MEASUREMENT
% ==========================================================

% Camera detects the cow within 50 m

camera_detected = true_range < 50;

% True bearing relative to ego

camera_bearing_true = atan2(rel_y, rel_x);

% Add camera noise

camera_bearing = camera_bearing_true + ...
    deg2rad(1.5) .* randn(size(t));

%% =========================================================
% 5. LIDAR MEASUREMENTS
% ==========================================================

% LiDAR measures object X,Y position

lidar_x = cow_x + ...
    0.15 .* randn(size(t));

lidar_y = cow_y + ...
    0.15 .* randn(size(t));

%% =========================================================
% 6. RADAR MEASUREMENT
% ==========================================================
%
% Radar measures relative radial velocity:
%
% z = unit_relative_position' *
%     (object_velocity - ego_velocity)
%
% Radar becomes unreliable at extremely small range.

radar_range_threshold = 1.0;

valid_radar = true_range > radar_range_threshold;

true_range_rate = nan(size(t));

% Relative velocity

rel_vx_true = cow_vx - ego_speed;
rel_vy_true = cow_vy;

% Calculate radial velocity only when range is valid

true_range_rate(valid_radar) = ...
    (rel_x(valid_radar) .* rel_vx_true(valid_radar) + ...
     rel_y(valid_radar) .* rel_vy_true(valid_radar)) ./ ...
     true_range(valid_radar);

% Add radar noise

radar_range_rate = nan(size(t));

radar_range_rate(valid_radar) = ...
    true_range_rate(valid_radar) + ...
    0.10 .* randn(1, sum(valid_radar));

%% =========================================================
% 7. EKF INITIALIZATION
% ==========================================================

% State:
%
% X  -> object longitudinal position
% Y  -> object lateral position
% Vx -> object longitudinal velocity
% Vy -> object lateral velocity

x_est = [ ...
    lidar_x(1);
    lidar_y(1);
    0;
    cow_speed
];

% Initial covariance

P = diag([ ...
    1;
    1;
    4;
    4
]);

% Process noise

Q = diag([ ...
    0.01;
    0.01;
    0.2;
    0.2
]);

% LiDAR measurement noise

R_lidar = diag([ ...
    0.15^2;
    0.15^2
]);

% Radar measurement noise

R_radar = 0.10^2;

%% =========================================================
% 8. STORAGE
% ==========================================================

estimated_state = ...
    zeros(4, length(t));

% NEW:
% Store EKF covariance at every time step.
%
% This will be used by the uncertainty-aware
% risk assessment module.

estimated_covariance = ...
    zeros(4, 4, length(t));

%% =========================================================
% 9. EKF LOOP
% ==========================================================

for k = 1:length(t)

    %% =====================================================
    % PREDICTION
    % ======================================================

    F = [ ...
        1 0 dt 0;
        0 1 0 dt;
        0 0 1  0;
        0 0 0  1
    ];

    % Predicted state

    x_pred = F * x_est;

    % Predicted covariance

    P_pred = F * P * F' + Q;

    %% =====================================================
    % LIDAR UPDATE
    % ======================================================

    % Measurement

    z_lidar = [ ...
        lidar_x(k);
        lidar_y(k)
    ];

    % Measurement model

    H_lidar = [ ...
        1 0 0 0;
        0 1 0 0
    ];

    % Innovation

    innovation = ...
        z_lidar - H_lidar * x_pred;

    % Innovation covariance

    S = ...
        H_lidar * P_pred * H_lidar' + R_lidar;

    % Kalman gain

    K = ...
        P_pred * H_lidar' / S;

    % State update

    x_est = ...
        x_pred + K * innovation;

    % Joseph-form covariance update

    I = eye(4);

    P = ...
        (I - K * H_lidar) * P_pred * ...
        (I - K * H_lidar)' + ...
        K * R_lidar * K';

    %% =====================================================
    % RADAR UPDATE
    % ======================================================
    %
    % Radar direction uses:
    %
    % object position - ego position
    %
    % Radar measurement is relative radial velocity.

    if valid_radar(k) && ...
            ~isnan(radar_range_rate(k))

        %% Current estimated object state

        object_x = x_est(1);
        object_y = x_est(2);

        object_vx = x_est(3);
        object_vy = x_est(4);

        %% Relative position

        relative_x = ...
            object_x - ego_x(k);

        relative_y = ...
            object_y - ego_y(k);

        estimated_range = ...
            sqrt(relative_x^2 + relative_y^2);

        %% Only use radar away from zero range

        if estimated_range > radar_range_threshold

            %% Unit vector from ego to object

            ux = ...
                relative_x / estimated_range;

            uy = ...
                relative_y / estimated_range;

            %% Ego velocity

            ego_vx = ego_speed;
            ego_vy = 0;

            %% Relative velocity

            relative_vx = ...
                object_vx - ego_vx;

            relative_vy = ...
                object_vy - ego_vy;

            %% Predicted radar measurement

            predicted_radar = ...
                ux * relative_vx + ...
                uy * relative_vy;

            %% Actual radar measurement

            z_radar = ...
                radar_range_rate(k);

            %% Innovation

            innovation = ...
                z_radar - predicted_radar;

            %% Radar measurement Jacobian

            H_radar = [ ...
                0 0 ux uy
            ];

            %% Innovation covariance

            S = ...
                H_radar * P * H_radar' + ...
                R_radar;

            %% Kalman gain

            K = ...
                P * H_radar' / S;

            %% State update

            x_est = ...
                x_est + K * innovation;

            %% Joseph-form covariance update

            P = ...
                (I - K * H_radar) * P * ...
                (I - K * H_radar)' + ...
                K * R_radar * K';

        end
    end

    %% =====================================================
    % NUMERICAL SAFETY
    % ======================================================

    if any(isnan(x_est)) || ...
            any(isinf(x_est))

        warning( ...
            'Invalid EKF state at t = %.2f s.', ...
            t(k));

        x_est = x_pred;
        P = P_pred;

    end

    %% =====================================================
    % STORE STATE
    % ======================================================

    estimated_state(:,k) = x_est;

    % NEW:
    % Store covariance for uncertainty analysis

    estimated_covariance(:,:,k) = P;

end

%% =========================================================
% 10. POSITION TRACKING
% ==========================================================

figure('Name','INDRA - EKF Position Tracking');

plot( ...
    cow_x, ...
    cow_y, ...
    'r--', ...
    'LineWidth', 2);

hold on;

plot( ...
    estimated_state(1,:), ...
    estimated_state(2,:), ...
    'b-', ...
    'LineWidth', 2);

plot( ...
    lidar_x, ...
    lidar_y, ...
    '.', ...
    'MarkerSize', 5);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('INDRA - EKF Sensor Fusion');

legend( ...
    'True Cow Position', ...
    'EKF Estimated Position', ...
    'LiDAR Measurements');

grid on;
axis equal;

%% =========================================================
% 11. LONGITUDINAL VELOCITY
% ==========================================================

figure('Name','INDRA - Longitudinal Velocity');

plot( ...
    t, ...
    cow_vx, ...
    'r--', ...
    'LineWidth', 2);

hold on;

plot( ...
    t, ...
    estimated_state(3,:), ...
    'b-', ...
    'LineWidth', 2);

xlabel('Time (s)');
ylabel('Vx (m/s)');

title('INDRA - Longitudinal Velocity Tracking');

legend( ...
    'True Cow Vx', ...
    'EKF Estimated Vx');

grid on;

%% =========================================================
% 12. LATERAL VELOCITY
% ==========================================================

figure('Name','INDRA - Lateral Velocity');

plot( ...
    t, ...
    cow_vy, ...
    'r--', ...
    'LineWidth', 2);

hold on;

plot( ...
    t, ...
    estimated_state(4,:), ...
    'b-', ...
    'LineWidth', 2);

xlabel('Time (s)');
ylabel('Vy (m/s)');

title('INDRA - Lateral Velocity Tracking');

legend( ...
    'True Cow Vy', ...
    'EKF Estimated Vy');

grid on;

%% =========================================================
% 13. POSITION UNCERTAINTY
% ==========================================================

sigma_x = sqrt( ...
    max(squeeze(estimated_covariance(1,1,:)), 0));

sigma_y = sqrt( ...
    max(squeeze(estimated_covariance(2,2,:)), 0));

figure('Name','INDRA - EKF Position Uncertainty');

plot( ...
    t, ...
    sigma_x, ...
    'b-', ...
    'LineWidth', 2);

hold on;

plot( ...
    t, ...
    sigma_y, ...
    'r-', ...
    'LineWidth', 2);

xlabel('Time (s)');
ylabel('Position Standard Deviation (m)');

title('INDRA - EKF Position Uncertainty');

legend( ...
    'Sigma X', ...
    'Sigma Y');

grid on;

%% =========================================================
% 14. DISPLAY RESULTS
% ==========================================================

sample_times = ...
    [1 2 3 3.5 4 5 6 7 8];

fprintf('\n');
fprintf('INDRA EKF TRACKING RESULTS\n');
fprintf('---------------------------------------------\n');

for i = 1:length(sample_times)

    [~, k] = ...
        min(abs(t - sample_times(i)));

    fprintf('\nTime = %.2f s\n', t(k));

    fprintf( ...
        'Estimated X       = %.2f m\n', ...
        estimated_state(1,k));

    fprintf( ...
        'Estimated Y       = %.2f m\n', ...
        estimated_state(2,k));

    fprintf( ...
        'Estimated Vx      = %.2f m/s\n', ...
        estimated_state(3,k));

    fprintf( ...
        'Estimated Vy      = %.2f m/s\n', ...
        estimated_state(4,k));

    fprintf( ...
        'Sigma X           = %.3f m\n', ...
        sigma_x(k));

    fprintf( ...
        'Sigma Y           = %.3f m\n', ...
        sigma_y(k));

    if valid_radar(k)

        fprintf('Radar             = VALID\n');

    else

        fprintf('Radar             = INVALID / SKIPPED\n');

    end

end

%% =========================================================
% 15. TRUE VS FINAL ESTIMATED STATE
% ==========================================================

fprintf('\n');
fprintf('TRUE COW STATE\n');
fprintf('---------------------------------------------\n');

fprintf( ...
    'True X  = %.2f m\n', ...
    cow_x(end));

fprintf( ...
    'True Y  = %.2f m\n', ...
    cow_y(end));

fprintf( ...
    'True Vx = %.2f m/s\n', ...
    cow_vx(end));

fprintf( ...
    'True Vy = %.2f m/s\n', ...
    cow_vy(end));

fprintf('\n');

fprintf('FINAL EKF STATE\n');
fprintf('---------------------------------------------\n');

fprintf( ...
    'X  = %.2f m\n', ...
    estimated_state(1,end));

fprintf( ...
    'Y  = %.2f m\n', ...
    estimated_state(2,end));

fprintf( ...
    'Vx = %.2f m/s\n', ...
    estimated_state(3,end));

fprintf( ...
    'Vy = %.2f m/s\n', ...
    estimated_state(4,end));

fprintf('\n');

fprintf('FINAL POSITION UNCERTAINTY\n');
fprintf('---------------------------------------------\n');

fprintf( ...
    'Sigma X = %.3f m\n', ...
    sigma_x(end));

fprintf( ...
    'Sigma Y = %.3f m\n', ...
    sigma_y(end));