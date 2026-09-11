%% INDRA - Simulated Multi-Sensor Measurements

clear;
clc;
close all;

rng(1);                         % Repeatable noise

%% Simulation parameters
dt = 0.05;
T = 8;
t = 0:dt:T;

%% Ego vehicle
ego_speed = 10;                 % m/s
ego_x = ego_speed .* t;
ego_y = zeros(size(t));

%% Cow ground truth
cow_x = 35 * ones(size(t));

cow_start_y = 7;
cow_speed = -2;
cow_y = cow_start_y + cow_speed .* t;

%% Relative position
rel_x = cow_x - ego_x;
rel_y = cow_y - ego_y;

range_true = sqrt(rel_x.^2 + rel_y.^2);

%% =========================================================
% CAMERA
% Camera gives object class + bearing
% ==========================================================

camera_detected = range_true < 50;

camera_bearing_true = atan2(rel_y, rel_x);

camera_bearing = camera_bearing_true + ...
    deg2rad(1.5) .* randn(size(t));

%% =========================================================
% LiDAR
% LiDAR provides position measurements
% ==========================================================

lidar_x = cow_x + 0.15 .* randn(size(t));
lidar_y = cow_y + 0.15 .* randn(size(t));

%% =========================================================
% RADAR
% Radar provides range + relative radial velocity
% ==========================================================

range_measurement = range_true + ...
    0.20 .* randn(size(t));

% Calculate true range rate
range_rate_true = [0 diff(range_true)] ./ dt;

range_rate_measurement = range_rate_true + ...
    0.10 .* randn(size(t));

%% =========================================================
% Display measurements at selected times
% ==========================================================

sample_times = [1 2 3 3.5 4 5];

fprintf('\nINDRA SENSOR MEASUREMENTS\n');
fprintf('---------------------------------------------\n');

for i = 1:length(sample_times)

    [~, k] = min(abs(t - sample_times(i)));

    fprintf('\nTime = %.2f s\n', t(k));

    if camera_detected(k)
        fprintf('Camera : Cow detected, bearing = %.2f deg\n', ...
            rad2deg(camera_bearing(k)));
    else
        fprintf('Camera : No detection\n');
    end

    fprintf('LiDAR  : X = %.2f m, Y = %.2f m\n', ...
        lidar_x(k), lidar_y(k));

    fprintf('Radar  : Range = %.2f m, Range Rate = %.2f m/s\n', ...
        range_measurement(k), range_rate_measurement(k));
end

%% =========================================================
% Plot sensor measurements
% ==========================================================

figure('Name','INDRA - Sensor Measurements');

plot(ego_x, ego_y, 'b-', 'LineWidth', 2);
hold on;

plot(cow_x, cow_y, 'r--', 'LineWidth', 2);

plot(lidar_x, lidar_y, '.', 'MarkerSize', 8);

xlabel('Road Position X (m)');
ylabel('Lateral Position Y (m)');

title('INDRA - Multi-Sensor Observations');

legend('Ego Vehicle', ...
       'Cow Ground Truth', ...
       'LiDAR Measurements');

grid on;
axis equal;