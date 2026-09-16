function [hFig, sim_result] = indra_live_visualization(scenario_input, varargin)
%% INDRA_LIVE_VISUALIZATION
% Real-time dynamic engineering visualization & SIH demo dashboard for INDRA.
%
% Visualizes the full closed-loop autonomy pipeline timestep-by-timestep:
%   Radar Sensor Detections -> EKF Object Tracking -> Future CV Prediction ->
%   Uncertainty Safety Ellipses -> Risk Assessment -> Local Path Planner ->
%   MPC Lateral/Longitudinal Control -> Kinematic Bicycle Model.
%
% Usage:
%   indra_live_visualization("VILLAGE_CATTLE");
%   indra_live_visualization("URBAN_INTERSECTION", 'playback_speed', 1.5);
%   indra_live_visualization("HIGHWAY_MERGE");
%   indra_live_visualization("DENSE_MARKET");
%   indra_live_visualization("SUDDEN_CATTLE");
%
%   % With scenario struct:
%   sc = scenario_config("VILLAGE_CATTLE");
%   sc.ego_speed = 10;
%   indra_live_visualization(sc);
%
% Options (Name-Value):
%   'playback_speed'  - Playback speed multiplier (default: 1.0, 0 = no delay)
%   'headless'        - Run without opening window (default: false, for test suites)
%   'enable_noise'    - Add realistic radar noise to sensor simulation (default: false)
%   'save_gif'        - Filepath string to export animated GIF (default: "")

% Add paths
addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');
addpath('D:\INDRA\main');
addpath('D:\INDRA\tracking');
addpath('D:\INDRA\prediction');
addpath('D:\INDRA\risk');
addpath('D:\INDRA\planning');
addpath('D:\INDRA\control');

%% 1. Parse Inputs
p = inputParser;
addRequired(p, 'scenario_input');
addParameter(p, 'playback_speed', 1.0, @(x) isnumeric(x) && x >= 0);
addParameter(p, 'headless', false, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'enable_noise', false, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'save_gif', "", @(x) isstring(x) || ischar(x));
parse(p, scenario_input, varargin{:});

playback_speed = p.Results.playback_speed;
headless = logical(p.Results.headless);
enable_noise = logical(p.Results.enable_noise);
save_gif = string(p.Results.save_gif);

%% 2. Run / Obtain Simulation Data
if isstruct(scenario_input) && isfield(scenario_input, 'ego_x') && isfield(scenario_input, 'risk')
    % Passed an already computed result struct
    sim_result = scenario_input;
    if isfield(sim_result, 'scenario')
        scenario = sim_result.scenario;
    else
        scenario = scenario_config(sim_result.scenario_name);
    end
else
    % Run sensor-driven closed loop
    sensor_params = struct('detection_range', 50.0, 'enable_noise', enable_noise);
    sim_result = indra_sensor_closed_loop(scenario_input, sensor_params);
    if isfield(sim_result, 'scenario')
        scenario = sim_result.scenario;
    elseif isstruct(scenario_input)
        scenario = scenario_input;
    else
        scenario = scenario_config(scenario_input);
    end
end

%% 3. Setup Figure & Dashboard Canvas
if headless
    fig_visible = 'off';
else
    fig_visible = 'on';
end

hFig = figure('Name', sprintf('INDRA Autonomy Live Dashboard — %s', scenario.name), ...
              'NumberTitle', 'off', ...
              'Color', [0.08 0.10 0.14], ...
              'Position', [50 50 1400 820], ...
              'Visible', fig_visible);

set(hFig, 'DefaultTextColor', [0.9 0.92 0.95], ...
          'DefaultAxesColor', [0.12 0.14 0.20], ...
          'DefaultAxesXColor', [0.6 0.65 0.75], ...
          'DefaultAxesYColor', [0.6 0.65 0.75]);

% Layout:
% Left: Bird's-Eye-View (BEV) Road Simulation (60% width)
ax_bev = subplot('Position', [0.05 0.36 0.60 0.58]);
hold(ax_bev, 'on');
grid(ax_bev, 'on');
box(ax_bev, 'on');
set(ax_bev, 'GridColor', [0.25 0.30 0.40], 'GridAlpha', 0.4);
title(ax_bev, sprintf('BIRD''S-EYE VIEW — SCENARIO: %s (%s)', scenario.name, scenario.type), ...
      'Color', [0.95 0.95 1.0], 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax_bev, 'Longitudinal Position X (m)', 'Color', [0.7 0.75 0.85]);
ylabel(ax_bev, 'Lateral Position Y (m)', 'Color', [0.7 0.75 0.85]);

% Bottom Left: Real-time Dynamics & Control Telemetry (60% width)
ax_dyn = subplot('Position', [0.05 0.08 0.60 0.22]);
hold(ax_dyn, 'on');
grid(ax_dyn, 'on');
box(ax_dyn, 'on');
set(ax_dyn, 'GridColor', [0.25 0.30 0.40], 'GridAlpha', 0.4);
xlabel(ax_dyn, 'Time (s)', 'Color', [0.7 0.75 0.85]);
ylabel(ax_dyn, 'Speed (m/s) & Steer (deg)', 'Color', [0.7 0.75 0.85]);
title(ax_dyn, 'REAL-TIME CONTROL & RISK PROFILE', 'Color', [0.85 0.9 1.0], 'FontSize', 10, 'FontWeight', 'bold');

% Right Side: Telemetry HUD Panel (30% width)
ax_hud = subplot('Position', [0.68 0.08 0.29 0.86]);
axis(ax_hud, 'off');
set(ax_hud, 'Color', [0.10 0.12 0.17]);

%% 4. Draw Static Road Elements
time_vec = sim_result.time;
N = length(time_vec);
Ts = time_vec(2) - time_vec(1);

road_y = scenario.road_center_y;
max_x = max(sim_result.ego_x) + 40;
road_x = linspace(-10, max_x, 200);

% Road bounds
fill(ax_bev, [-10 max_x max_x -10], [road_y-4 road_y-4 road_y+4 road_y+4], ...
     [0.16 0.18 0.24], 'EdgeColor', 'none', 'DisplayName', 'Road Surface');

% Road Centerline & Boundaries
plot(ax_bev, road_x, road_y * ones(size(road_x)), '--', 'Color', [0.85 0.75 0.2], 'LineWidth', 1.2, 'DisplayName', 'Centerline');
plot(ax_bev, road_x, (road_y + 3.5) * ones(size(road_x)), '-', 'Color', [0.45 0.50 0.60], 'LineWidth', 1.5, 'DisplayName', 'Road Margin');
plot(ax_bev, road_x, (road_y - 3.5) * ones(size(road_x)), '-', 'Color', [0.45 0.50 0.60], 'LineWidth', 1.5, 'DisplayName', 'Road Margin');

% Ego Trajectory line (Past)
h_ego_trail = plot(ax_bev, nan, nan, 'Color', [0.0 0.8 1.0], 'LineWidth', 2.0, 'DisplayName', 'Ego Executed Path');

% Planner Reference Path (Future)
h_plan_path = plot(ax_bev, nan, nan, '--', 'Color', [0.9 0.3 0.9], 'LineWidth', 1.8, 'DisplayName', 'Planner Reference');

% Ego Vehicle Body (Polygon)
ego_L = 4.0; ego_W = 1.8;
h_ego_body = fill(ax_bev, [0 0 0 0], [0 0 0 0], [0.1 0.6 0.95], 'EdgeColor', [1 1 1], 'LineWidth', 1.5, 'DisplayName', 'Ego Vehicle');
h_ego_dir = plot(ax_bev, [0 0], [0 0], 'Color', [1 1 0], 'LineWidth', 2.0, 'HandleVisibility', 'off');

% Dynamic Objects Setup
num_objs = scenario.object_count;
h_obj_truth = gobjects(num_objs, 1);
h_obj_est = gobjects(num_objs, 1);
h_obj_label = gobjects(num_objs, 1);

truth_colors = [0.4 0.4 0.5; 0.4 0.4 0.5; 0.4 0.4 0.5; 0.4 0.4 0.5; 0.4 0.4 0.5];
est_colors = [0.95 0.25 0.25; 0.95 0.55 0.15; 0.85 0.25 0.85; 0.25 0.85 0.45; 0.25 0.75 0.95];

for i = 1:num_objs
    % Ground truth debug marker (faint circle)
    h_obj_truth(i) = plot(ax_bev, nan, nan, 'o', 'Color', [0.55 0.55 0.65], ...
                          'MarkerSize', 8, 'LineWidth', 1.2, 'DisplayName', sprintf('Truth Obj %d', i));
    obj_id_i = scenario.object_id(i);
    obj_type_i = scenario.object_type(min(i, numel(scenario.object_type)));
    % EKF Estimated object marker (solid square)
    h_obj_est(i) = plot(ax_bev, nan, nan, 's', 'MarkerFaceColor', est_colors(mod(i-1, 5)+1, :), ...
                        'MarkerEdgeColor', [1 1 1], 'MarkerSize', 9, 'LineWidth', 1.2, ...
                        'DisplayName', sprintf('EKF Track %d (%s)', obj_id_i, obj_type_i));
    % Text label
    h_obj_label(i) = text(ax_bev, 0, 0, '', 'Color', [1 1 1], 'FontSize', 8, 'FontWeight', 'bold');
end

% Prediction & Uncertainty region for Critical Threat
h_pred_traj = plot(ax_bev, nan, nan, 'm.-', 'LineWidth', 1.8, 'MarkerSize', 7, 'DisplayName', 'Predicted Threat Path');
h_unc_region = fill(ax_bev, [0 0], [0 0], [0.9 0.2 0.2], 'FaceAlpha', 0.2, 'EdgeColor', [0.9 0.3 0.3], ...
                    'LineStyle', ':', 'DisplayName', 'Safety Uncertainty Margin');

% Legend on BEV
legend(ax_bev, [h_ego_body, h_ego_trail, h_plan_path, h_obj_est(1), h_pred_traj, h_unc_region], ...
       'Location', 'northwest', 'TextColor', [0.9 0.9 0.9], 'Color', [0.12 0.14 0.20], 'EdgeColor', [0.3 0.35 0.45]);

%% 5. Setup Strip Chart (Bottom-Left)
plot(ax_dyn, time_vec, sim_result.ego_speed, 'Color', [0.0 0.8 1.0], 'LineWidth', 1.8, 'DisplayName', 'Speed (m/s)');
plot(ax_dyn, time_vec, rad2deg(sim_result.steering), 'Color', [0.95 0.75 0.1], 'LineWidth', 1.5, 'DisplayName', 'Steering (deg)');
plot(ax_dyn, time_vec, sim_result.acceleration, 'Color', [0.9 0.3 0.3], 'LineWidth', 1.5, 'DisplayName', 'Accel (m/s^2)');
h_time_cursor = plot(ax_dyn, [0 0], [-8 15], 'Color', [1 1 1], 'LineWidth', 1.5, 'DisplayName', 'Current Time');
legend(ax_dyn, 'Location', 'northeast', 'TextColor', [0.9 0.9 0.9], 'Color', [0.12 0.14 0.20], 'EdgeColor', [0.3 0.35 0.45]);
ylim(ax_dyn, [-7 16]);
xlim(ax_dyn, [0 time_vec(end)]);

%% 6. Render Initial Telemetry HUD Text Elements
cla(ax_hud);
xlim(ax_hud, [0 1]);
ylim(ax_hud, [0 1]);

% Background Card
rectangle(ax_hud, 'Position', [0.02 0.02 0.96 0.96], 'FaceColor', [0.12 0.14 0.20], ...
          'EdgeColor', [0.3 0.35 0.48], 'Curvature', 0.04, 'LineWidth', 1.5);

% HUD Header
text(ax_hud, 0.50, 0.94, 'INDRA AUTONOMY TELEMETRY', 'Color', [0.0 0.85 1.0], ...
     'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Risk Banner Patch & Text
h_risk_card = rectangle(ax_hud, 'Position', [0.06 0.82 0.88 0.07], 'FaceColor', [0.1 0.6 0.2], ...
                        'EdgeColor', [1 1 1], 'Curvature', 0.15, 'LineWidth', 1.2);
h_risk_text = text(ax_hud, 0.50, 0.855, 'RISK: LOW', 'Color', [1 1 1], ...
                   'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Dynamic Telemetry Text handles
h_txt_time = text(ax_hud, 0.08, 0.77, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);
h_txt_speed = text(ax_hud, 0.08, 0.71, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);
h_txt_control = text(ax_hud, 0.08, 0.65, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);
h_txt_decision = text(ax_hud, 0.08, 0.59, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);
h_txt_planner = text(ax_hud, 0.08, 0.53, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);

h_txt_threat_id = text(ax_hud, 0.08, 0.44, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);
h_txt_ttc = text(ax_hud, 0.08, 0.38, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);
h_txt_distance = text(ax_hud, 0.08, 0.32, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);
h_txt_tracking = text(ax_hud, 0.08, 0.26, '', 'Color', [0.9 0.9 0.9], 'FontSize', 9.5);

% Autonomy Pipeline Status Indicators
text(ax_hud, 0.08, 0.18, 'PIPELINE SUBSYSTEMS:', 'Color', [0.7 0.75 0.85], 'FontSize', 9, 'FontWeight', 'bold');
h_txt_pipeline = text(ax_hud, 0.08, 0.08, ...
    sprintf('Radar Sensor : ACTIVE\nEKF Tracking : ACTIVE\nCV Predictor : ACTIVE\nRisk Engine  : ACTIVE\nMPC Control  : ACTIVE'), ...
    'Color', [0.2 0.85 0.4], 'FontSize', 8.5, 'FontWeight', 'bold');

%% 7. Animation Loop
gif_frames = {};

for k = 1:N
    if ~ishandle(hFig)
        break; % Window closed by user
    end

    t_now = time_vec(k);
    ego_x_k = sim_result.ego_x(k);
    ego_y_k = sim_result.ego_y(k);
    ego_psi_k = sim_result.ego_heading(k);
    ego_spd_k = sim_result.ego_speed(k);
    steer_k = sim_result.steering(k);
    accel_k = sim_result.acceleration(k);
    risk_k = sim_result.risk(k);
    action_k = sim_result.planner_action(k);
    crit_id_k = sim_result.critical_object_id(k);
    ttc_k = sim_result.TTC(k);
    pred_d_k = sim_result.predicted_distance(k);
    margin_k = sim_result.safety_margin(k);
    plan_ref_y_k = sim_result.planner_reference_y(k);

    % Update Ego Trajectory
    set(h_ego_trail, 'XData', sim_result.ego_x(1:k), 'YData', sim_result.ego_y(1:k));

    % Update Ego Vehicle Polygon
    corners_local = [-ego_L/2, -ego_W/2;
                      ego_L/2, -ego_W/2;
                      ego_L/2,  ego_W/2;
                     -ego_L/2,  ego_W/2]';
    R_ego = [cos(ego_psi_k) -sin(ego_psi_k); sin(ego_psi_k) cos(ego_psi_k)];
    corners_global = R_ego * corners_local + [ego_x_k; ego_y_k];
    set(h_ego_body, 'XData', corners_global(1, :), 'YData', corners_global(2, :));

    % Ego Heading Vector
    dir_vec = [ego_x_k, ego_x_k + 3.0 * cos(ego_psi_k); ...
               ego_y_k, ego_y_k + 3.0 * sin(ego_psi_k)];
    set(h_ego_dir, 'XData', dir_vec(1, :), 'YData', dir_vec(2, :));

    % Planner Reference Path (20m ahead)
    x_plan = linspace(ego_x_k, ego_x_k + 25, 30);
    y_plan = linspace(ego_y_k, plan_ref_y_k, 30);
    set(h_plan_path, 'XData', x_plan, 'YData', y_plan);

    % Update Dynamic Objects
    for i = 1:num_objs
        if isfield(sim_result, 'true_x') && size(sim_result.true_x, 1) >= i
            tx_i = sim_result.true_x(i, k);
            ty_i = sim_result.true_y(i, k);
            set(h_obj_truth(i), 'XData', tx_i, 'YData', ty_i);
        end

        if isfield(sim_result, 'est_x') && size(sim_result.est_x, 1) >= i
            ex_i = sim_result.est_x(i, k);
            ey_i = sim_result.est_y(i, k);
            obj_id_i = scenario.object_id(i);
            obj_type_i = scenario.object_type(min(i, numel(scenario.object_type)));
            set(h_obj_est(i), 'XData', ex_i, 'YData', ey_i);
            set(h_obj_label(i), 'Position', [ex_i + 0.8, ey_i + 0.8, 0], ...
                'String', sprintf('Obj %d: %s', obj_id_i, obj_type_i));
        end
    end

    % Update Critical Threat Prediction & Uncertainty Region
    if crit_id_k > 0 && isfield(sim_result, 'est_x') && size(sim_result.est_x, 1) >= crit_id_k
        est_px = sim_result.est_x(crit_id_k, k);
        est_py = sim_result.est_y(crit_id_k, k);
        est_pvx = sim_result.est_vx(crit_id_k, k);
        est_pvy = sim_result.est_vy(crit_id_k, k);

        fut_t = (1:15) * 0.2;
        pred_x_pts = est_px + est_pvx * fut_t;
        pred_y_pts = est_py + est_pvy * fut_t;
        set(h_pred_traj, 'XData', [est_px pred_x_pts], 'YData', [est_py pred_y_pts]);

        % Uncertainty buffer polygon
        unc_buf = max(sim_result.uncertainty_margin(k), 1.0);
        theta_cir = linspace(0, 2*pi, 24);
        cir_x = pred_x_pts(end) + unc_buf * cos(theta_cir);
        cir_y = pred_y_pts(end) + unc_buf * sin(theta_cir);
        set(h_unc_region, 'XData', cir_x, 'YData', cir_y);
    else
        set(h_pred_traj, 'XData', nan, 'YData', nan);
        set(h_unc_region, 'XData', [0 0], 'YData', [0 0]);
    end

    % Adjust BEV camera window around Ego
    xlim(ax_bev, [ego_x_k - 12, ego_x_k + 48]);
    ylim(ax_bev, [road_y - 6, road_y + 6]);

    % Update Strip Chart cursor
    set(h_time_cursor, 'XData', [t_now t_now]);

    % Update HUD Risk Card Color
    if risk_k == "HIGH"
        set(h_risk_card, 'FaceColor', [0.85 0.15 0.15]);
        set(h_risk_text, 'String', 'RISK: HIGH (EMERGENCY)', 'Color', [1 1 1]);
    elseif risk_k == "MEDIUM"
        set(h_risk_card, 'FaceColor', [0.92 0.55 0.05]);
        set(h_risk_text, 'String', 'RISK: MEDIUM (AVOIDANCE)', 'Color', [1 1 1]);
    else
        set(h_risk_card, 'FaceColor', [0.10 0.65 0.25]);
        set(h_risk_text, 'String', 'RISK: LOW (CLEAR)', 'Color', [1 1 1]);
    end

    % Update HUD Telemetry text
    set(h_txt_time, 'String', sprintf('Time : %.2f s / %.2f s (Step %d/%d)', t_now, time_vec(end), k, N));
    set(h_txt_speed, 'String', sprintf('Ego Speed : %.2f m/s (%.1f km/h) | Cmd: %.1f m/s', ...
                                      ego_spd_k, ego_spd_k*3.6, sim_result.target_speed(k)));
    set(h_txt_control, 'String', sprintf('Steering  : %+.2f deg | Accel: %+.2f m/s^2', ...
                                        rad2deg(steer_k), accel_k));
    set(h_txt_decision, 'String', sprintf('Behavior  : %s', sim_result.behavior(k)));
    set(h_txt_planner, 'String', sprintf('Action    : %s', action_k));

    if crit_id_k > 0
        crit_type_k = scenario.object_type(min(crit_id_k, numel(scenario.object_type)));
        set(h_txt_threat_id, 'String', sprintf('Critical Threat : Obj ID %d (%s)', crit_id_k, crit_type_k));
        if isinf(ttc_k)
            ttc_str = 'Inf';
        else
            ttc_str = sprintf('%.2f s', ttc_k);
        end
        set(h_txt_ttc, 'String', sprintf('Time to Collision (TTC) : %s', ttc_str));
        set(h_txt_distance, 'String', sprintf('Pred Dist : %.2f m | Margin: %+.2f m', pred_d_k, margin_k));
        set(h_txt_tracking, 'String', sprintf('EKF Uncertainty 2σ : %.2f m', sim_result.uncertainty_margin(k)));
    else
        set(h_txt_threat_id, 'String', 'Critical Threat : NONE');
        set(h_txt_ttc, 'String', 'Time to Collision (TTC) : Inf');
        set(h_txt_distance, 'String', 'Pred Dist : Clear');
        set(h_txt_tracking, 'String', 'EKF Uncertainty 2σ : 0.00 m');
    end

    drawnow;

    % Optional GIF export
    if save_gif ~= "" && mod(k, 2) == 1
        frame = getframe(hFig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if k == 1
            imwrite(imind, cm, char(save_gif), 'gif', 'Loopcount', inf, 'DelayTime', Ts*2);
        else
            imwrite(imind, cm, char(save_gif), 'gif', 'WriteMode', 'append', 'DelayTime', Ts*2);
        end
    end

    % Timing control
    if playback_speed > 0 && ~headless
        pause(Ts / playback_speed);
    end
end

if ishandle(hFig)
    % Final summary note on HUD
    text(ax_hud, 0.50, 0.02, 'SIMULATION COMPLETE', 'Color', [0.0 0.85 1.0], ...
         'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    drawnow;
end

end
