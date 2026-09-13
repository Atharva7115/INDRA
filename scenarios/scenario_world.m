function world = scenario_world(scenario, current_time)
% SCENARIO_WORLD
% Dynamic world generator for INDRA simulation scenarios.
%
% Standardized multi-object interface:
%   world.object_count : number of dynamic objects in the world
%   world.object_id    : [1 x N] vector of unique object IDs
%   world.object_type  : [1 x N] string array of object types
%   world.x            : [1 x N] vector of X positions (m)
%   world.y            : [1 x N] vector of Y positions (m)
%   world.vx           : [1 x N] vector of X velocities (m/s)
%   world.vy           : [1 x N] vector of Y velocities (m/s)

switch upper(string(scenario.type))

    case {"CATTLE_CROSSING", "INTERSECTION", "HIGHWAY_MERGE", "DENSE_MARKET", "SUDDEN_CATTLE"}

        world.object_count = scenario.object_count;
        world.object_id = scenario.object_id;
        world.object_type = scenario.object_type;

        world.x = scenario.object_initial_x + ...
                  scenario.object_vx * current_time;

        world.y = scenario.object_initial_y + ...
                  scenario.object_vy * current_time;

        world.vx = scenario.object_vx;
        world.vy = scenario.object_vy;

    otherwise

        error("Unknown scenario type '%s'.", scenario.type);

end

end
