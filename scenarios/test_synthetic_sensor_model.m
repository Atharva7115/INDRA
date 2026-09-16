%% TEST_SYNTHETIC_SENSOR_MODEL
% Validation script for Phase 6.2 — Synthetic Sensor Simulation for INDRA.

addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');

fprintf('\n===============================================================\n');
fprintf('     INDRA PHASE 6.2 — SYNTHETIC SENSOR MODEL VALIDATION\n');
fprintf('===============================================================\n\n');

test_passed = true;

try
    % ---------------------------------------------------------
    % 1. Load VILLAGE_CATTLE scenario and ground truth at t=0
    % ---------------------------------------------------------
    sc = scenario_config("VILLAGE_CATTLE");
    world = scenario_world(sc, 0.0);
    ego_state = struct('x', sc.ego_x, 'y', sc.ego_y, 'speed', sc.ego_speed, 'heading', sc.ego_heading);

    fprintf('[1] Scenario Config Loaded: %s\n', sc.name);
    fprintf('    Ego: Pos (%.1f, %.1f) m, Speed %.1f m/s, Heading %.1f rad\n', ...
        ego_state.x, ego_state.y, ego_state.speed, ego_state.heading);
    fprintf('    Cattle Ground Truth: Pos (%.1f, %.1f) m, Vel (%.1f, %.1f) m/s\n', ...
        world.x(1), world.y(1), world.vx(1), world.vy(1));

    % ---------------------------------------------------------
    % 2. Test Execution & Deterministic Output (No Noise)
    % ---------------------------------------------------------
    sensor_params_det = struct('detection_range', 50.0, 'enable_noise', false);
    [obs_det1, dets1] = synthetic_sensor_model(world, ego_state, sensor_params_det, 0.0);
    [obs_det2, ~]     = synthetic_sensor_model(world, ego_state, sensor_params_det, 0.0);

    assert(obs_det1.num_detections == 1, 'Expected exactly 1 cattle detection.');
    assert(obs_det1.object_id(1) == 1, 'Detected object ID mismatch.');
    assert(strcmp(obs_det1.object_type(1), "CATTLE"), 'Detected object type mismatch.');
    fprintf('[2] Detection Successful: 1 dynamic object identified (Type: %s, ID: %d)\n', ...
        obs_det1.object_type(1), obs_det1.object_id(1));

    % ---------------------------------------------------------
    % 3. Relative Position Verification
    % ---------------------------------------------------------
    expected_rel_x = 35.0;
    expected_rel_y = 7.0;
    assert(abs(obs_det1.relative_pos_x(1) - expected_rel_x) < 1e-4, 'Relative X mismatch.');
    assert(abs(obs_det1.relative_pos_y(1) - expected_rel_y) < 1e-4, 'Relative Y mismatch.');
    fprintf('[3] Relative Position Verified: (%.2f, %.2f) m in ego body frame.\n', ...
        obs_det1.relative_pos_x(1), obs_det1.relative_pos_y(1));

    % ---------------------------------------------------------
    % 4. Relative Velocity & Radial Velocity (Range Rate)
    % ---------------------------------------------------------
    expected_rel_vx = -10.0; % cattle 0 m/s - ego 10 m/s
    expected_rel_vy = -2.0;  % cattle -2 m/s - ego 0 m/s
    expected_range = hypot(expected_rel_x, expected_rel_y); % ~35.693 m
    expected_bearing_rad = atan2(expected_rel_y, expected_rel_x); % ~0.1974 rad (11.31 deg)
    expected_range_rate = (expected_rel_x * expected_rel_vx + expected_rel_y * expected_rel_vy) / expected_range; % ~ -10.198 m/s

    assert(abs(obs_det1.range(1) - expected_range) < 1e-4, 'Range calculation mismatch.');
    assert(abs(obs_det1.bearing_rad(1) - expected_bearing_rad) < 1e-4, 'Bearing calculation mismatch.');
    assert(abs(obs_det1.range_rate(1) - expected_range_rate) < 1e-4, 'Range rate calculation mismatch.');
    assert(abs(obs_det1.relative_vel_x(1) - expected_rel_vx) < 1e-4, 'Relative Vx mismatch.');
    assert(abs(obs_det1.relative_vel_y(1) - expected_rel_vy) < 1e-4, 'Relative Vy mismatch.');

    fprintf('[4] Relative & Radial Velocities Verified:\n');
    fprintf('    Range: %.3f m, Bearing: %.2f deg (%.4f rad)\n', ...
        obs_det1.range(1), obs_det1.bearing_deg(1), obs_det1.bearing_rad(1));
    fprintf('    Range Rate: %.3f m/s, Rel Vel: (%.2f, %.2f) m/s\n', ...
        obs_det1.range_rate(1), obs_det1.relative_vel_x(1), obs_det1.relative_vel_y(1));

    % ---------------------------------------------------------
    % 5. Range & Bearing Finiteness
    % ---------------------------------------------------------
    assert(isfinite(obs_det1.range(1)), 'Range is not finite.');
    assert(isfinite(obs_det1.bearing_rad(1)), 'Bearing is not finite.');
    assert(isfinite(obs_det1.range_rate(1)), 'Range rate is not finite.');
    fprintf('[5] Range, bearing, and range rate finiteness confirmed.\n');

    % ---------------------------------------------------------
    % 6. Detection Range Filtering
    % ---------------------------------------------------------
    sensor_params_short = struct('detection_range', 20.0, 'enable_noise', false);
    obs_short = synthetic_sensor_model(world, ego_state, sensor_params_short, 0.0);
    assert(obs_short.num_detections == 0, 'Object beyond detection range should not be detected.');
    fprintf('[6] Detection range filtering verified (Range limit 20m correctly filtered out target at %.1fm).\n', expected_range);

    % ---------------------------------------------------------
    % 7. Measurement Noise Verification
    % ---------------------------------------------------------
    sensor_params_noisy = struct( ...
        'detection_range', 50.0, ...
        'enable_noise', true, ...
        'range_noise_sigma', 0.2, ...
        'bearing_noise_sigma_deg', 0.5, ...
        'range_rate_noise_sigma', 0.25, ...
        'rng_seed', 42);
    obs_noisy = synthetic_sensor_model(world, ego_state, sensor_params_noisy, 0.0);
    assert(obs_noisy.num_detections == 1, 'Noisy detection failed.');
    assert(abs(obs_noisy.range(1) - expected_range) < 3 * 0.2, 'Noisy range exceeds 3-sigma bound.');
    assert(abs(obs_noisy.bearing_rad(1) - expected_bearing_rad) < 3 * deg2rad(0.5), 'Noisy bearing exceeds 3-sigma bound.');
    assert(abs(obs_noisy.range_rate(1) - expected_range_rate) < 3 * 0.25, 'Noisy range rate exceeds 3-sigma bound.');
    fprintf('[7] Noise characteristics verified within 3-sigma statistical bounds.\n');
    fprintf('    Noisy Range: %.3f m (dev: %+.3f m)\n', obs_noisy.range(1), obs_noisy.range(1) - expected_range);
    fprintf('    Noisy Bearing: %.2f deg (dev: %+.2f deg)\n', obs_noisy.bearing_deg(1), obs_noisy.bearing_deg(1) - rad2deg(expected_bearing_rad));
    fprintf('    Noisy Range Rate: %.3f m/s (dev: %+.3f m/s)\n', obs_noisy.range_rate(1), obs_noisy.range_rate(1) - expected_range_rate);

    % ---------------------------------------------------------
    % 8. Deterministic Repeatability Check
    % ---------------------------------------------------------
    assert(isequal(obs_det1.range, obs_det2.range), 'Deterministic runs must be identical.');
    assert(isequal(obs_det1.bearing_rad, obs_det2.bearing_rad), 'Deterministic bearings must be identical.');
    assert(isequal(obs_det1.range_rate, obs_det2.range_rate), 'Deterministic range rates must be identical.');
    fprintf('[8] Deterministic repeatability confirmed (identical output across runs).\n');

    % ---------------------------------------------------------
    % 9. Multi-Object Input Support Structure
    % ---------------------------------------------------------
    sc_multi = scenario_config("URBAN_INTERSECTION");
    world_multi = scenario_world(sc_multi, 0.0);
    ego_multi = struct('x', sc_multi.ego_x, 'y', sc_multi.ego_y, 'speed', sc_multi.ego_speed, 'heading', sc_multi.ego_heading);
    obs_multi = synthetic_sensor_model(world_multi, ego_multi, sensor_params_det, 0.0);
    assert(obs_multi.num_detections == 2, 'Multi-object scenario should detect all visible objects.');
    assert(numel(obs_multi.object_id) == 2, 'Multi-object ID vector length mismatch.');
    fprintf('[9] Multi-object input structure verified (%d objects detected in URBAN_INTERSECTION).\n', obs_multi.num_detections);

    % ---------------------------------------------------------
    % 10. Sensor Fusion objectDetection Object Integration (if available)
    % ---------------------------------------------------------
    if ~isempty(dets1)
        assert(numel(dets1) == 1, 'Expected 1 objectDetection object.');
        assert(isa(dets1{1}, 'objectDetection'), 'Output element must be of class objectDetection.');
        fprintf('[10] objectDetection handle integration with Sensor Fusion Toolbox confirmed.\n');
    else
        fprintf('[10] objectDetection export skipped (Toolbox class not loaded).\n');
    end

catch ME
    fprintf('\nFAIL: Exception caught during testing: %s\n', ME.message);
    test_passed = false;
end

fprintf('\n---------------------------------------------------------------\n');
if test_passed
    fprintf('PHASE 6.2 TEST RESULT: PASS\n');
else
    fprintf('PHASE 6.2 TEST RESULT: FAIL\n');
end
fprintf('===============================================================\n\n');
