%% TEST_DRIVING_SCENARIO_WORLD
% Validation script for Phase 6 Step 6.1: MATLAB drivingScenario environment for VILLAGE_CATTLE.

addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');

fprintf('\n===============================================================\n');
fprintf('     INDRA PHASE 6.1 — DRIVING SCENARIO WORLD VALIDATION\n');
fprintf('===============================================================\n\n');

test_passed = true;

% 1. Load scenario configuration
sc = scenario_config("VILLAGE_CATTLE");
fprintf('[1] Scenario Config Loaded: %s (%s)\n', sc.name, sc.type);

% 2. Check Automated Driving Toolbox presence
has_adt = (exist('drivingScenario', 'class') == 8) || ~isempty(which('drivingScenario'));
fprintf('[2] Automated Driving Toolbox Available: %s\n', mat2str(has_adt));

if has_adt
    try
        % 3. Instantiate drivingScenario environment
        driving_world = driving_scenario_world(sc);
        fprintf('[3] driving_scenario_world successfully created drivingScenario object.\n');

        % 4. Verify return structure fields
        required_fields = {'scenario', 'ego', 'actors', 'config', 'sample_time', 'stop_time', 'road_center_y', 'object_count'};
        for i = 1:numel(required_fields)
            assert(isfield(driving_world, required_fields{i}), ...
                "Missing field '%s' in driving_world structure.", required_fields{i});
        end
        fprintf('[4] Richer driving_world structure fields verified.\n');

        % 5. Verify initial actor positions and velocities
        assert(isequal(driving_world.ego.Position, [0, 0, 0]), 'Ego initial position mismatch.');
        assert(isequal(driving_world.ego.Velocity, [10, 0, 0]), 'Ego initial velocity mismatch.');
        assert(isequal(driving_world.actors(1).Position, [35, 7, 0]), 'Cattle initial position mismatch.');
        assert(isequal(driving_world.actors(1).Velocity, [0, -2, 0]), 'Cattle initial velocity mismatch.');
        fprintf('[5] Initial positions and velocities match scenario_config exactly.\n');

        % 6. Advance scenario by one timestep (dt = 0.1 s)
        advance(driving_world.scenario);
        dt = driving_world.sample_time;
        expected_cattle_pos = [35, 7 + (-2)*dt, 0];
        expected_ego_pos = [10*dt, 0, 0];

        assert(max(abs(driving_world.actors(1).Position - expected_cattle_pos)) < 1e-4, ...
            'Cattle position did not advance consistently with velocity.');
        assert(max(abs(driving_world.ego.Position - expected_ego_pos)) < 1e-4, ...
            'Ego position did not advance consistently with velocity.');
        fprintf('[6] Single-step simulation advance (dt=0.1s) validated.\n');
        fprintf('    Ego moved to: (%.2f, %.2f) m\n', driving_world.ego.Position(1), driving_world.ego.Position(2));
        fprintf('    Cattle moved to: (%.2f, %.2f) m\n', driving_world.actors(1).Position(1), driving_world.actors(1).Position(2));

        % 7. Verify unsupported scenario error handling
        unsupported_threw_error = false;
        try
            sc_urban = scenario_config("URBAN_INTERSECTION");
            driving_scenario_world(sc_urban);
        catch ME
            if strcmp(ME.identifier, "INDRA:UnsupportedDrivingScenarioType")
                unsupported_threw_error = true;
            end
        end
        assert(unsupported_threw_error, 'Unsupported scenario type did not throw expected error.');
        fprintf('[7] Unsupported scenario error handling verified.\n');

    catch ME
        fprintf('FAIL: %s\n', ME.message);
        test_passed = false;
    end
else
    fprintf('[3] Static validation mode: Automated Driving Toolbox not present on this machine.\n');
    
    % Verify error handling when ADT is missing
    caught_expected = false;
    try
        driving_scenario_world(sc);
    catch ME
        if strcmp(ME.identifier, "INDRA:MissingAutomatedDrivingToolbox")
            caught_expected = true;
            fprintf('    Verified driving_scenario_world throws INDRA:MissingAutomatedDrivingToolbox cleanly.\n');
        else
            fprintf('    Unexpected error: %s (%s)\n', ME.message, ME.identifier);
        end
    end
    
    if ~caught_expected
        test_passed = false;
    end
    
    % Verify static configuration consistency
    assert(sc.ego_x == 0 && sc.ego_y == 0 && sc.ego_speed == 10 && sc.ego_heading == 0, 'Static Ego mismatch');
    assert(sc.object_initial_x == 35 && sc.object_initial_y == 7, 'Static Cattle position mismatch');
    assert(sc.object_vx == 0 && sc.object_vy == -2, 'Static Cattle velocity mismatch');
    fprintf('[4] Static parameter and coordinate consistency verified.\n');
end

fprintf('\n---------------------------------------------------------------\n');
if test_passed
    fprintf('PHASE 6.1 TEST RESULT: PASS\n');
else
    fprintf('PHASE 6.1 TEST RESULT: FAIL\n');
end
fprintf('===============================================================\n\n');
