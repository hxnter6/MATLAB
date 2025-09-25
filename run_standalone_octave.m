addpath('/workspace');
try
  s = LTS_Standalone(struct('plot', false));
  save('-mat', '/workspace/simReport.mat', 's');
  fprintf('OK\n');
catch ME
  fprintf('ERROR: %s\n', ME.message);
  rethrow(ME);
end

