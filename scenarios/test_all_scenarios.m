%% TEST_ALL_SCENARIOS
% Test script to execute and validate all five INDRA scenarios sequentially
% via scenario_runner, confirming clean state isolation, metrics, and pipeline integrity.

addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');

scenarios = { ...
    'VILLAGE_CATTLE', ...
    'URBAN_INTERSECTION', ...
    'HIGHWAY_MERGE', ...
    'DENSE_MARKET', ...
    'SUDDEN_CATTLE'};

fprintf('\n===============================================================\n');
fprintf('         INDRA SCENARIO RUNNER — ALL SCENARIOS TEST\n');
fprintf('===============================================================\n\n');

results = cell(numel(scenarios), 1);

for i = 1:numel(scenarios)
    s_name = scenarios{i};
    fprintf('\n>>> Running Scenario: %s <<<\n', s_name);
    results{i} = scenario_runner(s_name);
end

fprintf('\n\n====================================================================================================================================\n');
fprintf('                                                 INDRA MULTI-SCENARIO SUMMARY TABLE\n');
fprintf('====================================================================================================================================\n');
fprintf('%-20s | %-5s | %-12s | %-12s | %-12s | %-10s | %-10s | %-9s | %-8s | %-8s | %-9s | %-8s\n', ...
    'Scenario', 'Objs', '1st MEDIUM', '1st HIGH', '1st Interv', 'Max Steer', 'Max Brake', 'Final Spd', 'Final X', 'Final Y', 'Final Psi', 'Pipeline');
fprintf('------------------------------------------------------------------------------------------------------------------------------------\n');

for i = 1:numel(scenarios)
    r = results{i};
    sc_name = scenarios{i};
    sc_cfg = scenario_config(sc_name);
    
    first_med_str = sprintf('%.2f s', r.first_medium_risk_time);
    if isinf(r.first_medium_risk_time) || isnan(r.first_medium_risk_time)
        first_med_str = 'NOT REACHED';
    end
    
    first_high_str = sprintf('%.2f s', r.first_high_risk_time);
    if isinf(r.first_high_risk_time) || isnan(r.first_high_risk_time)
        first_high_str = 'NOT REACHED';
    end
    
    first_int_str = sprintf('%.2f s', r.first_intervention_time);
    if isinf(r.first_intervention_time) || isnan(r.first_intervention_time)
        first_int_str = 'NOT REACHED';
    end
    
    max_steer_deg = rad2deg(max(abs(r.steering)));
    max_brake_val = min(r.acceleration);
    final_spd = r.ego_speed(end);
    final_x = r.ego_x(end);
    final_y = r.ego_y(end);
    final_psi_deg = rad2deg(r.ego_heading(end));
    fprintf('%-20s | %-5d | %-12s | %-12s | %-12s | %-7.2f deg | %-10.2f | %-9.2f | %-8.2f | %-8.2f | %-7.2f deg | %-8s\n', ...
        sc_name, sc_cfg.object_count, first_med_str, first_high_str, first_int_str, ...
        max_steer_deg, max_brake_val, final_spd, final_x, final_y, final_psi_deg, 'ACTIVE');
end
fprintf('====================================================================================================================================\n\n');
