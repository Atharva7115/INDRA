function driving_world = driving_scenario_world(scenario_input)
% DRIVING_SCENARIO_WORLD
% MATLAB Automated Driving Toolbox drivingScenario simulation environment
% for INDRA autonomous driving scenarios.
%
% Usage:
%   driving_world = driving_scenario_world("VILLAGE_CATTLE")
%   driving_world = driving_scenario_world(scenario_struct)
%
% Output structure (driving_world):
%   .scenario       : MATLAB drivingScenario object (or mock/diagnostic if ADT not present)
%   .ego            : Ego vehicle actor
%   .actors         : Dynamic actors array (e.g. cattle, vehicles, etc.)
%   .config         : Validated scenario configuration struct
%   .sample_time    : Simulation sample time (s)
%   .stop_time      : Simulation stop time (s)
%   .road_center_y  : Road center Y coordinate (m)
%   .object_count   : Number of dynamic objects

% 1. Ensure path setup
addpath(genpath('D:\INDRA'));
addpath('D:\INDRA\scenarios');

% 2. Resolve scenario input
if ischar(scenario_input) || isstring(scenario_input)
    scenario = scenario_config(scenario_input);
elseif isstruct(scenario_input)
    scenario = scenario_input;
else
    error("INDRA:InvalidInput", ...
        "driving_scenario_world expects a scenario name (string/char) or a scenario struct.");
end

% 3. Validate scenario configuration
validate_scenario(scenario);

% 4. Simulation parameters
sample_time = 0.1;
stop_time = 8.0;

% 5. Check for Automated Driving Toolbox drivingScenario availability
has_adt = (exist('drivingScenario', 'class') == 8) || ~isempty(which('drivingScenario'));

if ~has_adt
    error("INDRA:MissingAutomatedDrivingToolbox", ...
        "Automated Driving Toolbox is required to instantiate MATLAB drivingScenario objects.");
end

% 6. Construct drivingScenario environment based on scenario type
switch upper(string(scenario.type))

    case "CATTLE_CROSSING"
        % Create drivingScenario object
        sc_obj = drivingScenario('SampleTime', sample_time, 'StopTime', stop_time);

        % Add straight unmarked village road
        road_centers = [0, scenario.road_center_y, 0; 100, scenario.road_center_y, 0];
        road_width = 7.0; % standard two-lane unmarked village road
        road(sc_obj, road_centers, road_width);

        % Create Ego Vehicle
        ego_pos = [scenario.ego_x, scenario.ego_y, 0];
        ego_vx = scenario.ego_speed * cos(scenario.ego_heading);
        ego_vy = scenario.ego_speed * sin(scenario.ego_heading);
        ego_vel = [ego_vx, ego_vy, 0];
        ego_yaw = rad2deg(scenario.ego_heading);

        ego_actor = vehicle(sc_obj, ...
            'ClassID', 1, ...
            'Position', ego_pos, ...
            'Velocity', ego_vel, ...
            'Yaw', ego_yaw, ...
            'Length', 4.5, ...
            'Width', 1.8, ...
            'Height', 1.5);

        % Create Cattle Actor
        cattle_pos = [scenario.object_initial_x(1), scenario.object_initial_y(1), 0];
        cattle_vel = [scenario.object_vx(1), scenario.object_vy(1), 0];

        cattle_actor = actor(sc_obj, ...
            'ClassID', 4, ...
            'Position', cattle_pos, ...
            'Velocity', cattle_vel, ...
            'Length', 2.0, ...
            'Width', 0.8, ...
            'Height', 1.4);

        % Populate return structure
        driving_world = struct();
        driving_world.scenario = sc_obj;
        driving_world.ego = ego_actor;
        driving_world.actors = cattle_actor;
        driving_world.config = scenario;
        driving_world.sample_time = sample_time;
        driving_world.stop_time = stop_time;
        driving_world.road_center_y = scenario.road_center_y;
        driving_world.object_count = scenario.object_count;

    otherwise
        error("INDRA:UnsupportedDrivingScenarioType", ...
            "driving_scenario_world does not yet support scenario type '%s'. Currently supported: CATTLE_CROSSING", ...
            scenario.type);

end

end
