%% TEST_SENSOR_TO_EKF
% Comprehensive validation script for Phase 6.3 — Synthetic Sensor -> EKF Tracking Integration.

addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');
addpath('D:\INDRA\tracking');

fprintf('\n===============================================================\n');
fprintf('     INDRA PHASE 6.3 — SENSOR TO EKF TRACKING VALIDATION\n');
fprintf('===============================================================\n\n');

test_passed = true;

try
    % =========================================================
    % 1. Single-Object Tracking (VILLAGE_CATTLE)
    % =========================================================
    fprintf('>>> Test 1: Single-Object Tracking (VILLAGE_CATTLE) <<<\n');
    sc_cattle = scenario_config("VILLAGE_CATTLE");
    dt = 0.1;
    t_sim = 0:dt:8.0;
    N_steps = numel(t_sim);

    % Sensor configuration with realistic noise
    sensor_params_noisy = struct( ...
        'detection_range', sc_cattle.sensor_range, ...
        'enable_noise', true, ...
        'range_noise_sigma', 0.15, ...
        'bearing_noise_sigma_deg', 0.30, ...
        'range_rate_noise_sigma', 0.20, ...
        'rng_seed', 101);

    tracks = [];
    true_x_hist = zeros(1, N_steps);
    true_y_hist = zeros(1, N_steps);
    true_vx_hist = zeros(1, N_steps);
    true_vy_hist = zeros(1, N_steps);

    est_x_hist = zeros(1, N_steps);
    est_y_hist = zeros(1, N_steps);
    est_vx_hist = zeros(1, N_steps);
    est_vy_hist = zeros(1, N_steps);

    % Ego motion (constant forward speed)
    ego_speed = sc_cattle.ego_speed;

    % Evaluate while cattle is within front sensor FOV (t = 0 to 3.5 s)
    active_steps = round(3.5 / dt);

    for k = 1:N_steps
        current_time = t_sim(k);
        ego_state = struct('x', ego_speed * current_time, 'y', 0.0, 'speed', ego_speed, 'heading', 0.0);
        world = scenario_world(sc_cattle, current_time);

        true_x_hist(k) = world.x(1);
        true_y_hist(k) = world.y(1);
        true_vx_hist(k) = world.vx(1);
        true_vy_hist(k) = world.vy(1);

        % Generate synthetic observations
        obs = synthetic_sensor_model(world, ego_state, sensor_params_noisy, current_time);

        % Feed to EKF adapter
        [tracks, est] = sensor_to_ekf_adapter(obs, ego_state, tracks, dt);

        if k <= active_steps
            assert(est.object_count == 1, 'Expected 1 tracked object while in FOV.');
            assert(est.object_id(1) == 1, 'Track ID mismatch.');
            assert(all(isfinite([est.x(1), est.y(1), est.vx(1), est.vy(1)])), 'Non-finite EKF state encountered.');

            est_x_hist(k) = est.x(1);
            est_y_hist(k) = est.y(1);
            est_vx_hist(k) = est.vx(1);
            est_vy_hist(k) = est.vy(1);
        end
    end

    % Quantitative error metrics over FOV active tracking window (skip initial step k=1)
    eval_range = 2:active_steps;
    pos_err = hypot(est_x_hist(eval_range) - true_x_hist(eval_range), est_y_hist(eval_range) - true_y_hist(eval_range));
    vel_err = hypot(est_vx_hist(eval_range) - true_vx_hist(eval_range), est_vy_hist(eval_range) - true_vy_hist(eval_range));

    pos_rmse = sqrt(mean(pos_err.^2));
    vel_rmse = sqrt(mean(vel_err.^2));
    max_pos_err = max(pos_err);
    max_vel_err = max(vel_err);

    fprintf('    Position RMSE        : %.3f m\n', pos_rmse);
    fprintf('    Velocity RMSE        : %.3f m/s\n', vel_rmse);
    fprintf('    Max Position Error   : %.3f m\n', max_pos_err);
    fprintf('    Max Velocity Error   : %.3f m/s\n', max_vel_err);
    fprintf('    Track ID Maintained  : ID = %d (Type = %s)\n', sc_cattle.object_id(1), sc_cattle.object_type(1));

    assert(pos_rmse < 0.50, 'Position RMSE exceeds allowable threshold.');
    assert(vel_rmse < 0.80, 'Velocity RMSE exceeds allowable threshold.');
    fprintf('    [PASS] Single-object tracking accuracy verified.\n\n');

    % =========================================================
    % 2. Deterministic Repeatability Check
    % =========================================================
    fprintf('>>> Test 2: Deterministic Sensor Repeatability <<<\n');
    sensor_params_det = struct('detection_range', sc_cattle.sensor_range, 'enable_noise', false);

    tracks_a = [];
    tracks_b = [];
    for k = 1:20
        t_curr = (k - 1) * dt;
        ego_state = struct('x', ego_speed * t_curr, 'y', 0.0, 'speed', ego_speed, 'heading', 0.0);
        world = scenario_world(sc_cattle, t_curr);

        obs_a = synthetic_sensor_model(world, ego_state, sensor_params_det, t_curr);
        obs_b = synthetic_sensor_model(world, ego_state, sensor_params_det, t_curr);

        [tracks_a, est_a] = sensor_to_ekf_adapter(obs_a, ego_state, tracks_a, dt);
        [tracks_b, est_b] = sensor_to_ekf_adapter(obs_b, ego_state, tracks_b, dt);

        assert(isequal(est_a.x, est_b.x) && isequal(est_a.y, est_b.y), 'Deterministic runs must produce identical states.');
    end
    fprintf('    [PASS] Deterministic repeatability confirmed.\n\n');

    % =========================================================
    % 3. Missed Detection / Coasting & Re-acquisition Test
    % =========================================================
    fprintf('>>> Test 3: Missed Detection & Track Coasting <<<\n');
    tracks_miss = [];
    coasting_cov_grew = false;

    for k = 1:50
        t_curr = (k - 1) * dt;
        ego_state = struct('x', ego_speed * t_curr, 'y', 0.0, 'speed', ego_speed, 'heading', 0.0);
        world = scenario_world(sc_cattle, t_curr);

        obs = synthetic_sensor_model(world, ego_state, sensor_params_det, t_curr);

        % Simulate sensor blackout between t = 1.5s and t = 2.5s (steps 16 to 25)
        if t_curr >= 1.5 && t_curr <= 2.5
            obs.num_detections = 0;
            obs.is_valid = [];
            obs.object_id = [];
        end

        [tracks_miss, est_miss] = sensor_to_ekf_adapter(obs, ego_state, tracks_miss, dt);

        assert(est_miss.object_count == 1, 'Track should remain active and coast during temporary dropout.');
        assert(all(isfinite([est_miss.x(1), est_miss.y(1), est_miss.vx(1), est_miss.vy(1)])), 'State became non-finite during coasting.');

        if abs(t_curr - 2.5) < 1e-4
            % At end of dropout, uncertainty should have grown
            coasting_cov_grew = (est_miss.uncertainty_x(1) > 0.5);
        end
    end

    assert(coasting_cov_grew, 'Uncertainty did not increase as expected during coasting.');
    % After resuming measurements (at t=5.0s), track should be re-locked closely to truth
    true_final_y = sc_cattle.object_initial_y + sc_cattle.object_vy * 4.9;
    assert(abs(est_miss.y(1) - true_final_y) < 0.20, 'Track failed to re-converge after blackout.');
    fprintf('    [PASS] Missed detection coasting & re-acquisition verified.\n\n');

    % =========================================================
    % 4. Multi-Object Tracking (URBAN_INTERSECTION)
    % =========================================================
    fprintf('>>> Test 4: Multi-Object Tracking (URBAN_INTERSECTION) <<<\n');
    sc_urban = scenario_config("URBAN_INTERSECTION");
    tracks_multi = [];

    for k = 1:40
        t_curr = (k - 1) * dt;
        ego_state_multi = struct('x', sc_urban.ego_speed * t_curr, 'y', 0.0, 'speed', sc_urban.ego_speed, 'heading', 0.0);
        world_multi = scenario_world(sc_urban, t_curr);

        obs_multi = synthetic_sensor_model(world_multi, ego_state_multi, sensor_params_det, t_curr);
        [tracks_multi, est_multi] = sensor_to_ekf_adapter(obs_multi, ego_state_multi, tracks_multi, dt);

        assert(est_multi.object_count == 2, 'Expected 2 distinct tracked objects.');
        assert(all(ismember([1, 2], est_multi.object_id)), 'Expected track IDs [1, 2].');
        assert(all(isfinite([est_multi.x, est_multi.y, est_multi.vx, est_multi.vy])), 'Non-finite multi-object state.');
    end

    % Verify separate EKF states
    idx1 = find(est_multi.object_id == 1);
    idx2 = find(est_multi.object_id == 2);
    assert(est_multi.x(idx1) ~= est_multi.x(idx2), 'Object 1 and 2 states must be distinct.');
    fprintf('    Object 1 (ID 1): Est Pos (%.2f, %.2f) m, Est Vel (%.2f, %.2f) m/s\n', ...
        est_multi.x(idx1), est_multi.y(idx1), est_multi.vx(idx1), est_multi.vy(idx1));
    fprintf('    Object 2 (ID 2): Est Pos (%.2f, %.2f) m, Est Vel (%.2f, %.2f) m/s\n', ...
        est_multi.x(idx2), est_multi.y(idx2), est_multi.vx(idx2), est_multi.vy(idx2));
    fprintf('    [PASS] Multi-object independent EKF tracking confirmed.\n\n');

catch ME
    fprintf('FAIL: %s\n', ME.message);
    test_passed = false;
end

fprintf('---------------------------------------------------------------\n');
if test_passed
    fprintf('PHASE 6.3 TEST RESULT: PASS\n');
else
    fprintf('PHASE 6.3 TEST RESULT: FAIL\n');
end
fprintf('===============================================================\n\n');
