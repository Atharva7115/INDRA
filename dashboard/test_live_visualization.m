%% TEST_LIVE_VISUALIZATION
% Comprehensive automated validation for Phase 6.5 — INDRA Live Visualization.

addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');
addpath('D:\INDRA\main');
addpath('D:\INDRA\tracking');
addpath('D:\INDRA\prediction');
addpath('D:\INDRA\risk');
addpath('D:\INDRA\planning');
addpath('D:\INDRA\control');
addpath('D:\INDRA\dashboard');

fprintf('\n===============================================================\n');
fprintf('     INDRA PHASE 6.5 — LIVE VISUALIZATION VALIDATION\n');
fprintf('===============================================================\n\n');

test_passed = true;

try
    % ---------------------------------------------------------
    % Test 1: VILLAGE_CATTLE Live Visualization (Headless execution)
    % ---------------------------------------------------------
    fprintf('>>> Test 1: VILLAGE_CATTLE Live Visualization Execution <<<\n');
    [hFig1, sim_vc] = indra_live_visualization("VILLAGE_CATTLE", 'headless', true, 'playback_speed', 0);
    
    assert(isvalid(hFig1), 'Figure handle is invalid.');
    assert(all(isfinite(sim_vc.ego_x)), 'Non-finite ego_x in simulation result.');
    assert(all(isfinite(sim_vc.steering)), 'Non-finite steering in simulation result.');
    assert(any(sim_vc.risk == "MEDIUM"), 'Risk change to MEDIUM was not captured in simulation.');
    assert(min(sim_vc.acceleration) <= -3.0, 'Braking intervention was not reflected in telemetry.');
    close(hFig1);
    fprintf('    [PASS] VILLAGE_CATTLE visualization initialized and completed cleanly.\n\n');

    % ---------------------------------------------------------
    % Test 2: Multi-Object Scenarios Compatibility
    % ---------------------------------------------------------
    fprintf('>>> Test 2: Multi-Object Scenarios Verification <<<\n');
    scenarios_to_test = ["URBAN_INTERSECTION", "HIGHWAY_MERGE", "DENSE_MARKET", "SUDDEN_CATTLE"];
    
    for s = 1:numel(scenarios_to_test)
        sc_name = scenarios_to_test(s);
        fprintf('    Testing scenario visualization: %s ...\n', sc_name);
        [hFig_s, sim_s] = indra_live_visualization(sc_name, 'headless', true, 'playback_speed', 0);
        assert(isvalid(hFig_s), sprintf('Figure handle invalid for %s', sc_name));
        assert(~isempty(sim_s.ego_x), sprintf('Empty trajectory for %s', sc_name));
        assert(all(isfinite(sim_s.ego_x)), sprintf('Non-finite values in %s', sc_name));
        close(hFig_s);
    end
    fprintf('    [PASS] All 4 multi-object scenarios visualized successfully.\n\n');

    % ---------------------------------------------------------
    % Test 3: Structured Scenario Input Compatibility
    % ---------------------------------------------------------
    fprintf('>>> Test 3: Dynamic Composer Scenario Struct Input <<<\n');
    sc_custom = scenario_config("VILLAGE_CATTLE");
    sc_custom.ego_speed = 12.0;
    [hFig_custom, sim_custom] = indra_live_visualization(sc_custom, 'headless', true, 'playback_speed', 0);
    assert(isvalid(hFig_custom), 'Custom struct figure invalid.');
    assert(sim_custom.ego_speed(1) == 12.0, 'Initial speed not respected.');
    close(hFig_custom);
    fprintf('    [PASS] Custom scenario struct executed and visualized correctly.\n\n');

    % ---------------------------------------------------------
    % Test 4: Regression Protection (Phase C, 6.2, 6.3, 6.4)
    % ---------------------------------------------------------
    fprintf('>>> Test 4: Frozen Phase Regressions Protection <<<\n');
    
    % Phase 6.2 test
    run('D:\INDRA\scenarios\test_synthetic_sensor_model.m');
    
    % Phase 6.3 test
    run('D:\INDRA\scenarios\test_sensor_to_ekf.m');
    
    % Phase 6.4 test
    run('D:\INDRA\scenarios\test_sensor_closed_loop.m');
    
    % Phase C 5-scenario baseline regression
    run('D:\INDRA\scenarios\test_all_scenarios.m');
    
    fprintf('\n    [PASS] All frozen baseline regression suites passed.\n\n');

catch ME
    fprintf('\nFAIL: Exception caught during testing: %s\n', ME.message);
    for k = 1:length(ME.stack)
        fprintf('    in %s at line %d\n', ME.stack(k).file, ME.stack(k).line);
    end
    test_passed = false;
end

fprintf('---------------------------------------------------------------\n');
if test_passed
    fprintf('PHASE 6.5 TEST RESULT: PASS\n');
else
    fprintf('PHASE 6.5 TEST RESULT: FAIL\n');
end
fprintf('===============================================================\n\n');
