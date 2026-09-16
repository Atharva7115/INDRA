function [tracks, track_estimates] = sensor_to_ekf_adapter(observations, ego_state, tracks, dt, tracker_params)
% SENSOR_TO_EKF_ADAPTER
% Measurement adapter and multi-object EKF tracking manager for INDRA.
%
% Bridges synthetic sensor observations (range, bearing, range rate)
% to the INDRA Extended Kalman Filter (EKF) in world coordinates [X, Y, Vx, Vy].
%
% Inputs:
%   observations    - Struct produced by synthetic_sensor_model:
%                     .timestamp, .num_detections, .object_id, .object_type,
%                     .range, .bearing_rad, .range_rate, .is_valid
%   ego_state       - Struct with ego pose:
%                     .x (or .ego_x), .y (or .ego_y), .speed (or .ego_speed), .heading (or .ego_heading)
%   tracks          - (Optional) Existing tracks struct array (or empty [] on first step)
%   dt              - (Optional) Time step (s, default: 0.1)
%   tracker_params  - (Optional) Configuration struct:
%                     .Q        : Process noise covariance (4x4, default: diag([0.01 0.01 0.1 0.1]))
%                     .R_pos    : Position measurement noise covariance (2x2, default: diag([0.15^2 0.15^2]))
%                     .R_radar  : Radar range rate noise variance (scalar, default: 0.20^2)
%                     .P_init   : Initial covariance (4x4, default: diag([1 1 4 4]))
%                     .max_missed: Max missed steps before deactivating (default: 20)
%
% Outputs:
%   tracks          - Updated struct array of tracked objects with internal EKF states:
%                     .object_id, .object_type, .x_est, .P, .last_timestamp, .missed_count, .is_active
%   track_estimates - Clean summary struct for downstream autonomy consumption:
%                     .object_count, .object_id, .object_type,
%                     .x, .y, .vx, .vy, .P, .uncertainty_x, .uncertainty_y

% -------------------------------------------------------------
% 1. Default Parameters & Inputs
% -------------------------------------------------------------
if nargin < 3 || isempty(tracks)
    tracks = struct( ...
        'object_id', {}, ...
        'object_type', {}, ...
        'x_est', {}, ...
        'P', {}, ...
        'last_timestamp', {}, ...
        'missed_count', {}, ...
        'is_active', {});
end

if nargin < 4 || isempty(dt)
    dt = 0.1;
end

if nargin < 5 || isempty(tracker_params)
    tracker_params = struct();
end

if ~isfield(tracker_params, 'Q'),        tracker_params.Q = diag([0.01, 0.01, 0.1, 0.1]); end
if ~isfield(tracker_params, 'R_pos'),    tracker_params.R_pos = diag([0.15^2, 0.15^2]); end
if ~isfield(tracker_params, 'R_radar'),  tracker_params.R_radar = 0.20^2; end
if ~isfield(tracker_params, 'P_init'),   tracker_params.P_init = diag([1.0, 1.0, 4.0, 4.0]); end
if ~isfield(tracker_params, 'max_missed'), tracker_params.max_missed = 20; end

Q = tracker_params.Q;
R_pos = tracker_params.R_pos;
R_radar = tracker_params.R_radar;
P_init = tracker_params.P_init;

% Extract ego pose
if isfield(ego_state, 'ego_x'), ego_x = ego_state.ego_x; else, ego_x = ego_state.x; end
if isfield(ego_state, 'ego_y'), ego_y = ego_state.ego_y; else, ego_y = ego_state.y; end
if isfield(ego_state, 'ego_speed'), ego_spd = ego_state.ego_speed; else, ego_spd = ego_state.speed; end
if isfield(ego_state, 'ego_heading'), ego_psi = ego_state.ego_heading; else, ego_psi = ego_state.heading; end

ego_vx = ego_spd * cos(ego_psi);
ego_vy = ego_spd * sin(ego_psi);

% State transition matrix (Constant Velocity model in world frame)
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1  0;
     0 0 0  1];

H_pos = [1 0 0 0;
         0 1 0 0];

I4 = eye(4);

% -------------------------------------------------------------
% 2. EKF Prediction Step for All Active Tracks
% -------------------------------------------------------------
num_existing = numel(tracks);
for k = 1:num_existing
    if tracks(k).is_active
        tracks(k).x_est = F * tracks(k).x_est;
        tracks(k).P = F * tracks(k).P * F' + Q;
        tracks(k).missed_count = tracks(k).missed_count + 1;
    end
end

% -------------------------------------------------------------
% 3. Measurement Conversion & EKF Update Step
% -------------------------------------------------------------
num_dets = observations.num_detections;
tracked_ids = [];
if num_existing > 0
    tracked_ids = [tracks.object_id];
end

for d = 1:num_dets
    if ~observations.is_valid(d)
        continue;
    end

    obj_id = observations.object_id(d);
    obj_type = observations.object_type(d);
    r = observations.range(d);
    bearing = observations.bearing_rad(d);
    range_rate = observations.range_rate(d);

    % Convert relative polar measurement in body frame to world coordinates
    rel_x_body = r * cos(bearing);
    rel_y_body = r * sin(bearing);

    % Rotate from ego body frame to world frame and add ego position
    cos_psi = cos(ego_psi);
    sin_psi = sin(ego_psi);

    world_meas_x = ego_x + (rel_x_body * cos_psi - rel_y_body * sin_psi);
    world_meas_y = ego_y + (rel_x_body * sin_psi + rel_y_body * cos_psi);
    z_pos = [world_meas_x; world_meas_y];

    % Find existing track or initialize a new one
    track_idx = find(tracked_ids == obj_id, 1);

    if isempty(track_idx)
        % Initialize new track
        new_track = struct();
        new_track.object_id = obj_id;
        new_track.object_type = obj_type;
        new_track.x_est = [world_meas_x; world_meas_y; 0; 0];
        new_track.P = P_init;
        new_track.last_timestamp = observations.timestamp;
        new_track.missed_count = 0;
        new_track.is_active = true;

        tracks(end + 1) = new_track; %#ok<AGROW>
        track_idx = numel(tracks);
        tracked_ids = [tracks.object_id]; %#ok<AGROW>
    else
        % Position measurement update
        x_pred = tracks(track_idx).x_est;
        P_pred = tracks(track_idx).P;

        innovation_pos = z_pos - H_pos * x_pred;
        S_pos = H_pos * P_pred * H_pos' + R_pos;
        K_pos = (P_pred * H_pos') / S_pos;

        x_upd = x_pred + K_pos * innovation_pos;
        P_upd = (I4 - K_pos * H_pos) * P_pred * (I4 - K_pos * H_pos)' + K_pos * R_pos * K_pos';

        % Optional Radar Range-Rate Update
        if ~isnan(range_rate) && r > 1.0
            est_x = x_upd(1);
            est_y = x_upd(2);

            rel_est_x = est_x - ego_x;
            rel_est_y = est_y - ego_y;
            est_range = hypot(rel_est_x, rel_est_y);

            if est_range > 1.0
                ux = rel_est_x / est_range;
                uy = rel_est_y / est_range;

                ego_radial_vel = ux * ego_vx + uy * ego_vy;
                radar_abs_radial_vel = range_rate + ego_radial_vel;

                H_radar = [0, 0, ux, uy];
                predicted_radar = H_radar * x_upd;
                innovation_radar = radar_abs_radial_vel - predicted_radar;

                S_radar = H_radar * P_upd * H_radar' + R_radar;
                K_radar = (P_upd * H_radar') / S_radar;

                x_upd = x_upd + K_radar * innovation_radar;
                P_upd = (I4 - K_radar * H_radar) * P_upd * (I4 - K_radar * H_radar)' + K_radar * R_radar * K_radar';
            end
        end

        % Numerical sanity check
        if any(isnan(x_upd)) || any(isinf(x_upd))
            tracks(track_idx).x_est = x_pred;
            tracks(track_idx).P = P_pred;
        else
            tracks(track_idx).x_est = x_upd;
            tracks(track_idx).P = P_upd;
        end

        tracks(track_idx).last_timestamp = observations.timestamp;
        tracks(track_idx).missed_count = 0;
        tracks(track_idx).is_active = true;
    end
end

% -------------------------------------------------------------
% 4. Manage Track Inactivity
% -------------------------------------------------------------
for k = 1:numel(tracks)
    if tracks(k).missed_count > tracker_params.max_missed
        tracks(k).is_active = false;
    end
end

% -------------------------------------------------------------
% 5. Assemble Output Track Estimates
% -------------------------------------------------------------
active_mask = [tracks.is_active];
active_tracks = tracks(active_mask);
M = numel(active_tracks);

track_estimates = struct();
track_estimates.object_count = M;
track_estimates.object_id = zeros(1, M);
track_estimates.object_type = strings(1, M);
track_estimates.x = zeros(1, M);
track_estimates.y = zeros(1, M);
track_estimates.vx = zeros(1, M);
track_estimates.vy = zeros(1, M);
track_estimates.P = zeros(4, 4, M);
track_estimates.uncertainty_x = zeros(1, M);
track_estimates.uncertainty_y = zeros(1, M);

for m = 1:M
    track_estimates.object_id(m) = active_tracks(m).object_id;
    track_estimates.object_type(m) = string(active_tracks(m).object_type);
    track_estimates.x(m) = active_tracks(m).x_est(1);
    track_estimates.y(m) = active_tracks(m).x_est(2);
    track_estimates.vx(m) = active_tracks(m).x_est(3);
    track_estimates.vy(m) = active_tracks(m).x_est(4);
    track_estimates.P(:, :, m) = active_tracks(m).P;
    track_estimates.uncertainty_x(m) = sqrt(max(active_tracks(m).P(1, 1), 0));
    track_estimates.uncertainty_y(m) = sqrt(max(active_tracks(m).P(2, 2), 0));
end

end
