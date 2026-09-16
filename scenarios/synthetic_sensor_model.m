function [observations, object_detections] = synthetic_sensor_model(world, ego_state, sensor_params, current_time)
% SYNTHETIC_SENSOR_MODEL
% Mathematical radar-like synthetic sensor observation model for INDRA.
%
% Converts ground-truth scenario_world state and ego vehicle pose into
% realistic relative sensor observations in the vehicle body coordinate frame
% (x: forward, y: left).
%
% Usage:
%   obs = synthetic_sensor_model(world, ego_state)
%   obs = synthetic_sensor_model(world, ego_state, sensor_params)
%   [obs, object_dets] = synthetic_sensor_model(world, ego_state, sensor_params, current_time)
%
% Inputs:
%   world          - Struct with multi-object ground-truth state:
%                    .object_count, .object_id, .object_type, .x, .y, .vx, .vy
%   ego_state      - Struct with ego pose:
%                    .x (or .ego_x), .y (or .ego_y), .speed (or .ego_speed), .heading (or .ego_heading)
%   sensor_params  - (Optional) Configuration struct:
%                    .detection_range       : max detection distance (m, default: 50.0)
%                    .fov_azimuth_deg       : azimuth FOV [min, max] (deg, default: [-90, 90])
%                    .enable_noise          : enable Gaussian noise (bool, default: false)
%                    .range_noise_sigma     : range std dev (m, default: 0.15)
%                    .bearing_noise_sigma_deg: bearing std dev (deg, default: 0.3)
%                    .range_rate_noise_sigma: range rate std dev (m/s, default: 0.20)
%                    .detection_probability : probability of detection (0-1, default: 1.0)
%                    .rng_seed              : random generator seed (optional)
%   current_time   - (Optional) Simulation timestamp (s, default: 0.0)
%
% Outputs:
%   observations   - Struct with structured radar-like measurements:
%                    .timestamp, .num_detections, .object_id, .object_type,
%                    .range, .bearing_rad, .bearing_deg, .range_rate,
%                    .relative_pos_x, .relative_pos_y, .relative_vel_x, .relative_vel_y,
%                    .is_valid, .confidence, .noise_covariance, .sensor_params
%   object_detections - (Optional) Cell array of MATLAB objectDetection objects
%                       (if Sensor Fusion and Tracking Toolbox is available)

% -------------------------------------------------------------
% 1. Parse & Validate Inputs
% -------------------------------------------------------------
if nargin < 1 || isempty(world)
    error("INDRA:InvalidInput", "world structure must be provided.");
end

if nargin < 2 || isempty(ego_state)
    error("INDRA:InvalidInput", "ego_state structure must be provided.");
end

if nargin < 3 || isempty(sensor_params)
    sensor_params = struct();
end

if nargin < 4 || isempty(current_time)
    current_time = 0.0;
end

% Set default sensor parameters
if ~isfield(sensor_params, 'detection_range'),        sensor_params.detection_range = 50.0; end
if ~isfield(sensor_params, 'fov_azimuth_deg'),        sensor_params.fov_azimuth_deg = [-90.0, 90.0]; end
if ~isfield(sensor_params, 'enable_noise'),           sensor_params.enable_noise = false; end
if ~isfield(sensor_params, 'range_noise_sigma'),      sensor_params.range_noise_sigma = 0.15; end
if ~isfield(sensor_params, 'bearing_noise_sigma_deg'),sensor_params.bearing_noise_sigma_deg = 0.30; end
if ~isfield(sensor_params, 'range_rate_noise_sigma'), sensor_params.range_rate_noise_sigma = 0.20; end
if ~isfield(sensor_params, 'detection_probability'),  sensor_params.detection_probability = 1.0; end

if isfield(sensor_params, 'rng_seed') && ~isempty(sensor_params.rng_seed)
    rng(sensor_params.rng_seed);
end

% Extract ego pose
if isfield(ego_state, 'ego_x'), ego_x = ego_state.ego_x; else, ego_x = ego_state.x; end
if isfield(ego_state, 'ego_y'), ego_y = ego_state.ego_y; else, ego_y = ego_state.y; end
if isfield(ego_state, 'ego_speed'), ego_spd = ego_state.ego_speed; else, ego_spd = ego_state.speed; end
if isfield(ego_state, 'ego_heading'), ego_psi = ego_state.ego_heading; else, ego_psi = ego_state.heading; end

ego_vx = ego_spd * cos(ego_psi);
ego_vy = ego_spd * sin(ego_psi);

% -------------------------------------------------------------
% 2. Process Multi-Object Ground Truth
% -------------------------------------------------------------
N = world.object_count;
detected_indices = [];

raw_range = zeros(1, N);
raw_bearing_rad = zeros(1, N);
raw_range_rate = zeros(1, N);
raw_rel_x = zeros(1, N);
raw_rel_y = zeros(1, N);
raw_rel_vx = zeros(1, N);
raw_rel_vy = zeros(1, N);

fov_rad = deg2rad(sensor_params.fov_azimuth_deg);

for i = 1:N
    % World relative offset
    dx = world.x(i) - ego_x;
    dy = world.y(i) - ego_y;
    dvx = world.vx(i) - ego_vx;
    dvy = world.vy(i) - ego_vy;

    % Transform to ego body coordinate frame (x: forward, y: left)
    cos_psi = cos(ego_psi);
    sin_psi = sin(ego_psi);

    rel_x = dx * cos_psi + dy * sin_psi;
    rel_y = -dx * sin_psi + dy * cos_psi;

    rel_vx = dvx * cos_psi + dvy * sin_psi;
    rel_vy = -dvx * sin_psi + dvy * cos_psi;

    % Spherical / Polar radar coordinates
    r = hypot(rel_x, rel_y);
    if r < 1e-6
        bearing = 0.0;
        range_rate = 0.0;
    else
        bearing = atan2(rel_y, rel_x);
        range_rate = (rel_x * rel_vx + rel_y * rel_vy) / r;
    end

    raw_range(i) = r;
    raw_bearing_rad(i) = bearing;
    raw_range_rate(i) = range_rate;
    raw_rel_x(i) = rel_x;
    raw_rel_y(i) = rel_y;
    raw_rel_vx(i) = rel_vx;
    raw_rel_vy(i) = rel_vy;

    % Check Field of View and Detection Range
    in_range = (r <= sensor_params.detection_range);
    in_fov = (bearing >= fov_rad(1)) && (bearing <= fov_rad(2));
    is_detected = (rand() <= sensor_params.detection_probability);

    if in_range && in_fov && is_detected
        detected_indices = [detected_indices, i]; %#ok<AGROW>
    end
end

num_dets = numel(detected_indices);

% -------------------------------------------------------------
% 3. Apply Noise (if enabled)
% -------------------------------------------------------------
bearing_sigma_rad = deg2rad(sensor_params.bearing_noise_sigma_deg);
noise_cov = diag([ ...
    sensor_params.range_noise_sigma^2, ...
    bearing_sigma_rad^2, ...
    sensor_params.range_rate_noise_sigma^2]);

if num_dets > 0
    det_ids = world.object_id(detected_indices);
    det_types = world.object_type(detected_indices);

    meas_range = raw_range(detected_indices);
    meas_bearing_rad = raw_bearing_rad(detected_indices);
    meas_range_rate = raw_range_rate(detected_indices);
    meas_rel_vx = raw_rel_vx(detected_indices);
    meas_rel_vy = raw_rel_vy(detected_indices);

    if sensor_params.enable_noise
        meas_range = meas_range + randn(1, num_dets) * sensor_params.range_noise_sigma;
        meas_bearing_rad = meas_bearing_rad + randn(1, num_dets) * bearing_sigma_rad;
        meas_range_rate = meas_range_rate + randn(1, num_dets) * sensor_params.range_rate_noise_sigma;
    end

    meas_bearing_deg = rad2deg(meas_bearing_rad);
    meas_rel_x = meas_range .* cos(meas_bearing_rad);
    meas_rel_y = meas_range .* sin(meas_bearing_rad);

    meas_valid = true(1, num_dets);
    meas_confidence = ones(1, num_dets);
else
    det_ids = [];
    det_types = strings(1, 0);
    meas_range = [];
    meas_bearing_rad = [];
    meas_bearing_deg = [];
    meas_range_rate = [];
    meas_rel_x = [];
    meas_rel_y = [];
    meas_rel_vx = [];
    meas_rel_vy = [];
    meas_valid = false(1, 0);
    meas_confidence = [];
end

% -------------------------------------------------------------
% 4. Assemble Observations Structure
% -------------------------------------------------------------
observations = struct();
observations.timestamp = current_time;
observations.num_detections = num_dets;
observations.object_id = det_ids;
observations.object_type = det_types;
observations.range = meas_range;
observations.bearing_rad = meas_bearing_rad;
observations.bearing_deg = meas_bearing_deg;
observations.range_rate = meas_range_rate;
observations.relative_pos_x = meas_rel_x;
observations.relative_pos_y = meas_rel_y;
observations.relative_vel_x = meas_rel_vx;
observations.relative_vel_y = meas_rel_vy;
observations.is_valid = meas_valid;
observations.confidence = meas_confidence;
observations.noise_covariance = noise_cov;
observations.sensor_params = sensor_params;

% -------------------------------------------------------------
% 5. (Optional) Create objectDetection Objects for Sensor Fusion
% -------------------------------------------------------------
if nargout >= 2
    object_detections = cell(num_dets, 1);
    has_obj_det = ~isempty(which('objectDetection'));

    if has_obj_det
        for k = 1:num_dets
            meas_vec = [meas_range(k); meas_bearing_rad(k); meas_range_rate(k)];
            object_detections{k} = objectDetection( ...
                current_time, ...
                meas_vec, ...
                'MeasurementNoise', noise_cov, ...
                'SensorIndex', 1, ...
                'ObjectClassID', det_ids(k), ...
                'MeasurementParameters', struct('Frame', 'Spherical'));
        end
    end
end

end
