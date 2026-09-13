function result = scenario_runner(scenario_input)
% SCENARIO_RUNNER
% Central experiment runner for the INDRA autonomous driving stack.
%
% Usage:
%   result = scenario_runner("VILLAGE_CATTLE")
%   result = scenario_runner("URBAN_INTERSECTION")
%   result = scenario_runner("HIGHWAY_MERGE")
%   result = scenario_runner("DENSE_MARKET")
%   result = scenario_runner("SUDDEN_CATTLE")
%
%   % Dynamic composer usage:
%   sc = scenario_config("SUDDEN_CATTLE");
%   sc.ego_speed = 10;
%   sc.sensor_range = 60;
%   result = scenario_runner(sc);

% Ensure INDRA paths are available
addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');

% 1. Parse scenario
if ischar(scenario_input) || isstring(scenario_input)
    scenario_struct = scenario_config(scenario_input);
elseif isstruct(scenario_input)
    scenario_struct = scenario_input;
else
    error("INDRA:InvalidInput", ...
        "scenario_runner expects a scenario name (string/char) or a scenario struct.");
end

% 2. Safety and integrity checks (Composer validation)
validate_scenario(scenario_struct);

% 3. Reset persistent controller state to guarantee clean isolation between runs
clear functions;

% 4. Execute INDRA closed-loop simulation
result = indra_main(scenario_struct);

end
