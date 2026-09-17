% GENERATE_PAPER_FIGURES Export paper figures from one saved theory-suite run.
% Set suiteDirectory to a results/<timestamp> directory, then press Run.

%% User settings

suiteDirectory = 'results';
outputDirectory = '';
showFigures = false;

%% Load the project and validate the selected result suite

startup;
if isempty(suiteDirectory)
    error('generate_paper_figures:SuiteDirectory', ...
        'Set suiteDirectory to a saved results/<timestamp> directory.');
end

%% Export static PNG and PDF publication assets

options = struct('outputDirectory', outputDirectory, 'visible', showFigures);
output = agc.viz.paperFigures(suiteDirectory, options);
for k = 1:numel(output)
    fprintf('Wrote %s\n', output(k).files.png);
    fprintf('Wrote %s\n', output(k).files.pdf);
end
