%% INDRA - DECISION LOGIC
% ---------------------------------------------------------
% Converts uncertainty-aware risk into a vehicle behavior.
%
% Risk levels:
%   LOW    -> CRUISE
%   MEDIUM -> SLOW / YIELD
%   HIGH   -> STOP / AVOID
%
% Output:
%   decision_result structure
% ---------------------------------------------------------

clear;
clc;
close all;

fprintf('\n');
fprintf('=====================================================\n');
fprintf('              INDRA DECISION LOGIC\n');
fprintf('=====================================================\n');

%% =====================================================
% 1. RUN UNCERTAINTY-AWARE RISK ASSESSMENT
% ======================================================

fprintf('\nRunning uncertainty-aware risk assessment...\n');

run('D:\INDRA\risk\risk_uncertainty.m');

fprintf('Risk assessment completed.\n');

%% =====================================================
% 2. READ RISK RESULT
% ======================================================

fprintf('\n');
fprintf('-----------------------------------------------------\n');
fprintf('RISK INPUT\n');
fprintf('-----------------------------------------------------\n');

fprintf('Risk Level = %s\n', risk_level);

fprintf('TTC = %.2f s\n', TTC);

fprintf('Minimum predicted distance = %.2f m\n', ...
    min_distance);

fprintf('Minimum adjusted distance = %.2f m\n', ...
    min_adjusted_distance);

%% =====================================================
% 3. VEHICLE PARAMETERS
% ======================================================

cruise_speed = 10.0;      % m/s
medium_speed = 5.0;       % m/s
stop_speed   = 0.0;       % m/s

%% =====================================================
% 4. DECISION LOGIC
% ======================================================

switch upper(string(risk_level))

    case "LOW"

        behavior = "CRUISE";

        target_speed = cruise_speed;

        brake_command = 0.0;

        avoid_command = false;

        planning_mode = "NORMAL";


    case "MEDIUM"

        behavior = "SLOW / YIELD";

        target_speed = medium_speed;

        brake_command = 0.5;

        avoid_command = false;

        planning_mode = "CAUTION";


    case "HIGH"

        behavior = "STOP / AVOID";

        target_speed = stop_speed;

        brake_command = 1.0;

        avoid_command = true;

        planning_mode = "EMERGENCY";


    otherwise

        % Fail-safe behavior

        behavior = "EMERGENCY STOP";

        target_speed = stop_speed;

        brake_command = 1.0;

        avoid_command = false;

        planning_mode = "FAILSAFE";

end

%% =====================================================
% 5. PRINT DECISION
% ======================================================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('              INDRA DECISION RESULT\n');
fprintf('=====================================================\n');

fprintf('\nRisk Level       : %s\n', risk_level);

fprintf('Behavior         : %s\n', behavior);

fprintf('Planning Mode    : %s\n', planning_mode);

fprintf('Target Speed     : %.2f m/s\n', target_speed);

fprintf('Brake Command    : %.2f\n', brake_command);


if avoid_command

    fprintf('Avoidance        : REQUIRED\n');

else

    fprintf('Avoidance        : NOT REQUIRED\n');

end

fprintf('\n');

fprintf('-----------------------------------------------------\n');

switch upper(string(risk_level))

    case "LOW"

        fprintf('ACTION: Continue normal driving.\n');


    case "MEDIUM"

        fprintf('ACTION: Reduce speed and prepare to yield.\n');


    case "HIGH"

        fprintf('ACTION: Emergency intervention required.\n');

        fprintf('ACTION: Attempt safe stop or path avoidance.\n');


    otherwise

        fprintf('ACTION: Fail-safe emergency stop.\n');

end

fprintf('-----------------------------------------------------\n');

%% =====================================================
% 6. CREATE DECISION OUTPUT STRUCTURE
% ======================================================
%
% IMPORTANT:
% Use "decision_result" instead of "decision".
% risk_uncertainty.m already creates a variable called
% "decision", so using the same name causes a conflict.

decision_result = struct();

decision_result.risk_level = string(risk_level);

decision_result.behavior = behavior;

decision_result.planning_mode = planning_mode;

decision_result.target_speed = target_speed;

decision_result.brake_command = brake_command;

decision_result.avoid_command = avoid_command;

decision_result.TTC = TTC;

decision_result.minimum_distance = min_distance;

decision_result.minimum_adjusted_distance = ...
    min_adjusted_distance;

fprintf('\nDecision output structure created successfully.\n');

%% =====================================================
% 7. DISPLAY STRUCTURE
% ======================================================

fprintf('\n');
fprintf('-----------------------------------------------------\n');
fprintf('DECISION OUTPUT STRUCTURE\n');
fprintf('-----------------------------------------------------\n');

disp(decision_result);

fprintf('=====================================================\n');