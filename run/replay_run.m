% REPLAY_RUN Visualize a v2 result directory after simulation completes.
% Set resultDirectory to a folder created by run_theory_suite, then press Run.
resultDirectory = '';
exportFile = '';
if isempty(resultDirectory)
    error('replay_run:ResultDirectory', 'Pass the result directory created by run_theory_suite.');
end
startup;
payload = agc.io.loadRun(resultDirectory);
agc.viz.replay3D(payload.run, struct('exportFile', exportFile));
