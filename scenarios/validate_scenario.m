function validate_scenario(scenario)
% VALIDATE_SCENARIO
% Performs comprehensive safety and integrity checks on an INDRA scenario struct.
%
% Validates:
%   - Struct type and required fields
%   - Finite numeric ego state
%   - Positive sensor range
%   - Valid object count and matching array dimensions
%   - Finite dynamic object positions and velocities
%   - Road boundaries and avoidance constraints

if ~isstruct(scenario)
    error("INDRA:InvalidScenario", "Scenario must be a MATLAB struct.");
end

required_fields = { ...
    'ego_x', 'ego_y', 'ego_speed', 'ego_heading', ...
    'road_center_y', 'max_avoidance_offset', ...
    'object_count', 'object_id', 'object_type', ...
    'object_initial_x', 'object_initial_y', ...
    'object_vx', 'object_vy', 'sensor_range'};

for k = 1:numel(required_fields)
    fld = required_fields{k};
    if ~isfield(scenario, fld)
        error("INDRA:MissingField", "Scenario struct is missing required field '%s'.", fld);
    end
end

% Ego state validation
if ~isnumeric(scenario.ego_x) || ~isscalar(scenario.ego_x) || ~isfinite(scenario.ego_x)
    error("INDRA:InvalidEgoState", "scenario.ego_x must be a finite numeric scalar.");
end
if ~isnumeric(scenario.ego_y) || ~isscalar(scenario.ego_y) || ~isfinite(scenario.ego_y)
    error("INDRA:InvalidEgoState", "scenario.ego_y must be a finite numeric scalar.");
end
if ~isnumeric(scenario.ego_speed) || ~isscalar(scenario.ego_speed) || ~isfinite(scenario.ego_speed) || scenario.ego_speed < 0
    error("INDRA:InvalidEgoState", "scenario.ego_speed must be a non-negative finite numeric scalar.");
end
if ~isnumeric(scenario.ego_heading) || ~isscalar(scenario.ego_heading) || ~isfinite(scenario.ego_heading)
    error("INDRA:InvalidEgoState", "scenario.ego_heading must be a finite numeric scalar.");
end

% Road validation
if ~isnumeric(scenario.road_center_y) || ~isscalar(scenario.road_center_y) || ~isfinite(scenario.road_center_y)
    error("INDRA:InvalidRoadParam", "scenario.road_center_y must be a finite numeric scalar.");
end
if ~isnumeric(scenario.max_avoidance_offset) || ~isscalar(scenario.max_avoidance_offset) || ...
        ~isfinite(scenario.max_avoidance_offset) || scenario.max_avoidance_offset <= 0
    error("INDRA:InvalidRoadParam", "scenario.max_avoidance_offset must be a positive finite numeric scalar.");
end

% Sensor range validation
if ~isnumeric(scenario.sensor_range) || ~isscalar(scenario.sensor_range) || ...
        ~isfinite(scenario.sensor_range) || scenario.sensor_range <= 0
    error("INDRA:InvalidSensorRange", "scenario.sensor_range must be a positive finite numeric scalar.");
end

% Object count validation
N = scenario.object_count;
if ~isnumeric(N) || ~isscalar(N) || ~isfinite(N) || N < 0 || floor(N) ~= N
    error("INDRA:InvalidObjectCount", "scenario.object_count must be a non-negative integer scalar.");
end

% Object arrays validation
arrays_to_check = { ...
    'object_id', 'object_type', 'object_initial_x', 'object_initial_y', 'object_vx', 'object_vy'};

for k = 1:numel(arrays_to_check)
    fld = arrays_to_check{k};
    val = scenario.(fld);
    if numel(val) ~= N
        error("INDRA:ArrayDimensionMismatch", ...
            "Length of '%s' (%d) does not match scenario.object_count (%d).", ...
            fld, numel(val), N);
    end
end

if N > 0
    if ~isnumeric(scenario.object_id) || any(~isfinite(scenario.object_id))
        error("INDRA:InvalidObjectData", "scenario.object_id must contain finite numeric values.");
    end
    if ~isnumeric(scenario.object_initial_x) || any(~isfinite(scenario.object_initial_x))
        error("INDRA:InvalidObjectData", "scenario.object_initial_x must contain finite numeric values.");
    end
    if ~isnumeric(scenario.object_initial_y) || any(~isfinite(scenario.object_initial_y))
        error("INDRA:InvalidObjectData", "scenario.object_initial_y must contain finite numeric values.");
    end
    if ~isnumeric(scenario.object_vx) || any(~isfinite(scenario.object_vx))
        error("INDRA:InvalidObjectData", "scenario.object_vx must contain finite numeric values.");
    end
    if ~isnumeric(scenario.object_vy) || any(~isfinite(scenario.object_vy))
        error("INDRA:InvalidObjectData", "scenario.object_vy must contain finite numeric values.");
    end
end

end
