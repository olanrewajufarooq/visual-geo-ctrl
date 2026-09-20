% GENERATE_PAPER_FIGURES Export figures from one saved theory-suite run.
% By default, comparisons are grouped by study in
% <suite>/comparisons/{nominal,euclidean,bregman,euclidean_v_bregman,performance}
% and each standalone diagnostic is stored beside its run.mat in
% <variant>/figures.

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

options = struct('outputDirectory', outputDirectory, ...
    'visible', showFigures);

output = agc.viz.paperFigures(suiteDirectory, options);

for k = 1:numel(output)
    fprintf('Wrote %s\n', output(k).files.png);

    fprintf('Wrote %s\n', output(k).files.pdf);
end
