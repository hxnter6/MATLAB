%% Solver_Output_Setup.m

if batch == 0
    i_max = 1;
    j_max = 1;
elseif batch == 1
    i_max = numel(ind_var1);
    j_max = 1;
    fprintf('Iterating %s.\n',var1)
elseif batch == 2
    i_max = numel(ind_var1);
    j_max = i_max;
    fprintf('Iterating %s and %s.\n',var1,var2)
else
    error('var "batch" is messed up')
end

run = 0;