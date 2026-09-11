%% INDRA - Sudden Cattle Crossing Scenario

clear;
clc;
close all;

%% Simulation parameters
dt = 0.05;
T = 8;
t = 0:dt:T;

%% Ego vehicle
ego_speed = 10;              % m/s
ego_x = ego_speed .* t;
ego_y = zeros(size(t));

%% Cow
% Cow starts beside the road and crosses at x = 35 m
cow_x = 35 * ones(size(t));

cow_start_y = 7;
cow_speed = -2;              % m/s
cow_y = cow_start_y + cow_speed .* t;

%% Create figure
figure('Name','INDRA - Cattle Crossing');

hold on;
grid on;
axis equal;

xlim([0 80]);
ylim([-10 10]);

xlabel('Road Position X (m)');
ylabel('Lateral Position Y (m)');

title('INDRA - Sudden Cattle Crossing');

%% Draw road
plot([0 80], [0 0], 'k--', 'LineWidth', 1.5);

%% Initial objects
ego = plot(ego_x(1), ego_y(1), ...
    'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

cow = plot(cow_x(1), cow_y(1), ...
    'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

legend('Road Center', 'Ego Vehicle', 'Cow');

%% Animation
for k = 1:length(t)

    % Update vehicle position
    set(ego, ...
        'XData', ego_x(k), ...
        'YData', ego_y(k));

    % Update cow position
    set(cow, ...
        'XData', cow_x(k), ...
        'YData', cow_y(k));

    % Display simulation time
    title(sprintf('INDRA - Sudden Cattle Crossing | Time = %.2f s', t(k)));

    drawnow;

    pause(0.02);
end