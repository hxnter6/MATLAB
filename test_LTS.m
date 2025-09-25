%% Test script for LTS_Complete_Independent
% Simple test to verify the code runs without errors

try
    fprintf('Testing LTS_Complete_Independent...\n')
    LTS_Complete_Independent();
    fprintf('Test completed successfully!\n')
catch ME
    fprintf('Error occurred: %s\n', ME.message)
    fprintf('Stack trace:\n')
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line)
    end
end