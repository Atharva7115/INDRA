%% TEST_SENSOR_CLOSED_LOOP
% Comprehensive validation script for Phase 6.4 — Sensor-Driven Closed-Loop Integration.

addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');
addpath('D:\INDRA\main');
addpath('D:\INDRA\tracking');

fprintf('\n===============================================================\n');
fprintf('     INDRA PHASE 6.4 — SENSOR CLOSED-LOOP VALIDATION\n');
fprintf('===============================================================\n\n');

test_passed = true;

try
    % =========================================================
    % 1. Execute Sensor-Driven Closed Loop (VILLAGE_CATTLE - Deterministic)
    % =========================================================
    fprintf('>>> Test 1: Sensor Closed-Loop Execution (Deterministic Mode) <<<\n');
    sensor_params_det = struct('detection_range', 50.0, 'enable_noise', false);
    r_det = indra_sensor_closed_loop("VILLAGE_CATTLE", sensor_params_det);

    % Sanity checks
    assert(all(isfinite(r_det.ego_x)), 'Non-finite ego_x encountered.');
    assert(all(isfinite(r_det.ego_y)), 'Non-finite ego_y encountered.');
    assert(all(isfinite(r_det.ego_speed)), 'Non-finite ego_speed encountered.');
    assert(all(isfinite(r_det.ego_heading)), 'Non-finite ego_heading encountered.');
    assert(all(isfinite(r_det.steering)), 'Non-finite steering command encountered.');
    assert(all(isfinite(r_det.acceleration)), 'Non-finite acceleration command encountered.');
    assert(~r_det.collision_detected, 'Collision detected in deterministic run.');
    assert(r_det.first_intervention_time <= 1.0, 'Intervention time exceeds safety bound.');
    assert(min(r_det.acceleration) <= -4.0, 'Braking intervention was not executed.');

    fprintf('    [PASS] Deterministic closed-loop run succeeded.\n\n');

    % =========================================================
    % 2. Execute Sensor-Driven Closed Loop (VILLAGE_CATTLE - Noisy Mode)
    % =========================================================
    fprintf('>>> Test 2: Sensor Closed-Loop Execution (Realistic Noise Mode) <<<\n');
    sensor_params_noisy = struct( ...
        'detection_range', 50.0, ...
        'enable_noise', true, ...
        'range_noise_sigma', 0.15, ...
        'bearing_noise_sigma_deg', 0.30, ...
        'range_rate_noise_sigma', 0.20, ...
        'rng_seed', 42);
    r_noisy = indra_sensor_closed_loop("VILLAGE_CATTLE", sensor_params_noisy);

    assert(all(isfinite(r_noisy.ego_x)), 'Non-finite ego_x in noisy run.');
    assert(~r_noisy.collision_detected, 'Collision detected in noisy run.');
    assert(r_noisy.first_intervention_time <= 1.0, 'Intervention time in noisy run exceeds safety bound.');
    assert(min(r_noisy.acceleration) <= -4.0, 'Braking intervention in noisy run was not executed.');

    fprintf('    [PASS] Noisy sensor closed-loop run succeeded.\n\n');

    % =========================================================
    % 3. Comparison with Frozen Baseline (VILLAGE_CATTLE)
    % =========================================================
    fprintf('>>> Test 3: Metric Comparison vs Frozen Baseline <<<\n');
    r_base = scenario_runner("VILLAGE_CATTLE");

    fprintf('\n----------------------------------------------------------------------\n');
    fprintf('%-25s | %-15s | %-15s\n', 'Metric', 'Phase C Baseline', 'Sensor Closed-Loop');
    fprintf('----------------------------------------------------------------------\n');
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'Final Speed (m/s)', r_base.ego_speed(end), r_det.ego_speed(end));
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'Final X (m)', r_base.ego_x(end), r_det.ego_x(end));
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'Final Y (m)', r_base.ego_y(end), r_det.ego_y(end));
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'Final Heading (deg)', rad2deg(r_base.ego_heading(end)), rad2deg(r_det.ego_heading(end)));
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'First MEDIUM (s)', r_base.first_medium_risk_time, r_det.first_medium_risk_time);
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'First Intervention (s)', r_base.first_intervention_time, r_det.first_intervention_time);
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'Max Steering (deg)', rad2deg(max(abs(r_base.steering))), rad2deg(max(abs(r_det.steering))));
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'Max Braking (m/s^2)', min(r_base.acceleration), min(r_det.acceleration));
    fprintf('%-25s | %-15.2f | %-15.2f\n', 'Min True Distance (m)', min(r_base.actual_distance), r_det.min_true_distance);
    fprintf('%-25s | %-15s | %-15s\n', 'Collision Detected', 'false', mat2str(r_det.collision_detected));
    fprintf('----------------------------------------------------------------------\n\n');

catch ME
    fprintf('\nFAIL: Exception caught during testing: %s\n', ME.message);
    test_passed = false;
end

fprintf('---------------------------------------------------------------\n');
if test_passed
    fprintf('PHASE 6.4 TEST RESULT: PASS\n');
else
    fprintf('PHASE 6.4 TEST RESULT: FAIL\n');
end
fprintf('===============================================================\n\n');
