% REPLAY_RUN Visualize one saved result directory after simulation completes.
%
% Set resultDirectory to a folder created by run_theory_suite, then press Run.

%% User settings

resultDirectory = '';

exportFile = '';

%% Load the selected saved run

if isempty(resultDirectory)
    error('replay_run:ResultDirectory', 'Pass the result directory created by run_theory_suite.');
end

startup;

payload = agc.io.loadRun(resultDirectory);

%% Render or export the 3-D replay

agc.viz.replay3D(payload.run, exportFile=exportFile);
