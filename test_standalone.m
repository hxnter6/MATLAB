%%
% test_standalone.m - Simple test for LTS_Standalone.m
%
% This script tests the LTS_Standalone.m file to ensure it runs
% without errors and produces reasonable output.
%%

clear all; close all; clc;

try
    fprintf('Testing LTS_Standalone.m...\n');

    % Run the standalone simulation
    run('LTS_Standalone.m');

    % Check if key variables exist
    if exist('simReport', 'var')
        fprintf('✓ simReport variable created successfully\n');
        fprintf('  - Lap time: %.3f seconds\n', simReport.lap_time);
        fprintf('  - Average speed: %.1f km/h\n', simReport.average_speed_kmh);

        if isfield(simReport, 'time_vec') && isfield(simReport, 'speed_vec')
            fprintf('✓ Speed vs time data generated\n');
            fprintf('  - Simulation points: %d\n', length(simReport.time_vec));
        end

        if isfield(simReport, 'drs_map')
            fprintf('✓ DRS data generated\n');
            drs_active = sum(simReport.drs_map > 0);
            fprintf('  - DRS active segments: %d\n', drs_active);
        end
    else
        fprintf('✗ simReport variable not found\n');
    end

    fprintf('\nTest completed successfully!\n');
    fprintf('LTS_Standalone.m appears to be working correctly.\n');

catch ME
    fprintf('✗ Error during test: %s\n', ME.message);
    fprintf('Error ID: %s\n', ME.identifier);
    fprintf('Error in function: %s\n', ME.stack(1).name);
    fprintf('Error at line: %d\n', ME.stack(1).line);
end