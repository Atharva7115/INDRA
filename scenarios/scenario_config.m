function scenario = scenario_config(scenario_name)
% SCENARIO_CONFIG
% Central configuration for INDRA simulation scenarios.
%
% Currently implemented:
%   VILLAGE_CATTLE
%
% The autonomy stack should NOT depend on the scenario name.
% Scenario-specific information stays inside the scenario engine.

switch upper(string(scenario_name))

    %% ============================================================
    %  VILLAGE ROAD - CATTLE CROSSING
    %  This matches the currently validated INDRA scenario.
    % =============================================================

    case "VILLAGE_CATTLE"

        scenario.name = "Village Road - Cattle Crossing";
        scenario.type = "CATTLE_CROSSING";

        % ---------------------------------------------------------
        % Ego vehicle initial state
        % ---------------------------------------------------------
        scenario.ego_x = 0;
        scenario.ego_y = 0;
        scenario.ego_speed = 10;
        scenario.ego_heading = 0;

        % ---------------------------------------------------------
        % Road configuration
        % ---------------------------------------------------------
        scenario.road_center_y = 0;
        scenario.max_avoidance_offset = 1.5;

        % ---------------------------------------------------------
        % Object configuration
        % ---------------------------------------------------------
        scenario.object_count = 1;
        scenario.object_id = 1;
        scenario.object_type = "CATTLE";

        scenario.object_initial_x = 35;
        scenario.object_initial_y = 7;

        scenario.object_vx = 0;
        scenario.object_vy = -2;

        % ---------------------------------------------------------
        % Sensor configuration
        % ---------------------------------------------------------
        scenario.sensor_range = 50;

        % ---------------------------------------------------------
        % Display information
        % ---------------------------------------------------------
        scenario.description = ...
            "Unmarked village road with sudden cattle crossing.";

    %% ============================================================
    %  URBAN INTERSECTION
    %  Busy unsignalized urban intersection with crossing traffic.
    % =============================================================

    case "URBAN_INTERSECTION"

        scenario.name = "Urban Intersection";
        scenario.type = "INTERSECTION";

        % ---------------------------------------------------------
        % Ego vehicle initial state
        % ---------------------------------------------------------
        scenario.ego_x = 0;
        scenario.ego_y = 0;
        scenario.ego_speed = 8;
        scenario.ego_heading = 0;

        % ---------------------------------------------------------
        % Road configuration
        % ---------------------------------------------------------
        scenario.road_center_y = 0;
        scenario.max_avoidance_offset = 1.5;

        % ---------------------------------------------------------
        % Object configuration
        % ---------------------------------------------------------
        scenario.object_count = 2;
        scenario.object_id = [1, 2];
        scenario.object_type = ["VEHICLE", "VEHICLE"];

        scenario.object_initial_x = [25, 18];
        scenario.object_initial_y = [8, -10];

        scenario.object_vx = [0, 0];
        scenario.object_vy = [-4, 4];

        % ---------------------------------------------------------
        % Sensor configuration
        % ---------------------------------------------------------
        scenario.sensor_range = 50;

        % ---------------------------------------------------------
        % Display information
        % ---------------------------------------------------------
        scenario.description = ...
            "Busy unsignalized urban intersection with crossing traffic.";

    %% ============================================================
    %  HIGHWAY MERGE
    %  Highway merge with a slow vehicle and an incoming merging vehicle.
    % =============================================================

    case "HIGHWAY_MERGE"

        scenario.name = "Highway Merge";
        scenario.type = "HIGHWAY_MERGE";

        % ---------------------------------------------------------
        % Ego vehicle initial state
        % ---------------------------------------------------------
        scenario.ego_x = 0;
        scenario.ego_y = 0;
        scenario.ego_speed = 15;
        scenario.ego_heading = 0;

        % ---------------------------------------------------------
        % Road configuration
        % ---------------------------------------------------------
        scenario.road_center_y = 0;
        scenario.max_avoidance_offset = 1.5;

        % ---------------------------------------------------------
        % Object configuration
        % ---------------------------------------------------------
        scenario.object_count = 3;
        scenario.object_id = [1, 2, 3];
        scenario.object_type = ["VEHICLE", "VEHICLE", "VEHICLE"];

        scenario.object_initial_x = [35, 20, -15];
        scenario.object_initial_y = [0, 4, 0];

        scenario.object_vx = [8, 12, 16];
        scenario.object_vy = [0, -1.5, 0];

        % ---------------------------------------------------------
        % Sensor configuration
        % ---------------------------------------------------------
        scenario.sensor_range = 60;

        % ---------------------------------------------------------
        % Display information
        % ---------------------------------------------------------
        scenario.description = ...
            "Highway merge with a slow vehicle and an incoming merging vehicle.";

    %% ============================================================
    %  DENSE MARKET
    %  Dense Indian market road with pedestrians, motorcycles, and vehicles.
    % =============================================================

    case "DENSE_MARKET"

        scenario.name = "Dense Market";
        scenario.type = "DENSE_MARKET";

        % ---------------------------------------------------------
        % Ego vehicle initial state
        % ---------------------------------------------------------
        scenario.ego_x = 0;
        scenario.ego_y = 0;
        scenario.ego_speed = 6;
        scenario.ego_heading = 0;

        % ---------------------------------------------------------
        % Road configuration
        % ---------------------------------------------------------
        scenario.road_center_y = 0;
        scenario.max_avoidance_offset = 1.5;

        % ---------------------------------------------------------
        % Object configuration
        % ---------------------------------------------------------
        scenario.object_count = 5;
        scenario.object_id = [1, 2, 3, 4, 5];
        scenario.object_type = ["PEDESTRIAN", "MOTORCYCLE", "VEHICLE", "PEDESTRIAN", "MOTORCYCLE"];

        scenario.object_initial_x = [16, 24, 32, 20, 12];
        scenario.object_initial_y = [2.5, -2.5, 0, 4, -4];

        scenario.object_vx = [-0.5, -1, 3, 0, 2];
        scenario.object_vy = [-0.8, 1.0, 0, -1.2, 1.0];

        % ---------------------------------------------------------
        % Sensor configuration
        % ---------------------------------------------------------
        scenario.sensor_range = 40;

        % ---------------------------------------------------------
        % Display information
        % ---------------------------------------------------------
        scenario.description = ...
            "Dense Indian market road with pedestrians, motorcycles, and vehicles.";

    %% ============================================================
    %  SUDDEN CATTLE CROSSING
    %  Sudden cattle crossing from roadside with limited reaction time.
    % =============================================================

    case "SUDDEN_CATTLE"

        scenario.name = "Sudden Cattle Crossing";
        scenario.type = "SUDDEN_CATTLE";

        % ---------------------------------------------------------
        % Ego vehicle initial state
        % ---------------------------------------------------------
        scenario.ego_x = 0;
        scenario.ego_y = 0;
        scenario.ego_speed = 12;
        scenario.ego_heading = 0;

        % ---------------------------------------------------------
        % Road configuration
        % ---------------------------------------------------------
        scenario.road_center_y = 0;
        scenario.max_avoidance_offset = 1.5;

        % ---------------------------------------------------------
        % Object configuration
        % ---------------------------------------------------------
        scenario.object_count = 2;
        scenario.object_id = [1, 2];
        scenario.object_type = ["CATTLE", "CATTLE"];

        scenario.object_initial_x = [42, 55];
        scenario.object_initial_y = [5, -6];

        scenario.object_vx = [0, 0];
        scenario.object_vy = [-3, 2.5];

        % ---------------------------------------------------------
        % Sensor configuration
        % ---------------------------------------------------------
        scenario.sensor_range = 50;

        % ---------------------------------------------------------
        % Display information
        % ---------------------------------------------------------
        scenario.description = ...
            "Sudden cattle crossing from roadside with limited reaction time.";

    otherwise

        error("Unknown scenario '%s'. " + ...
            "Available scenarios: VILLAGE_CATTLE, URBAN_INTERSECTION, HIGHWAY_MERGE, DENSE_MARKET, SUDDEN_CATTLE", ...
            scenario_name);

end

end