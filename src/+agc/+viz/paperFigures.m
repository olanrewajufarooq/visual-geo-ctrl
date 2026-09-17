function output = paperFigures(suiteDirectory, options)
%PAPERFIGURES Export static paper figures from one saved theory-suite folder.
%
% The suite must contain nominal, Euclidean, and Bregman runs, each using
% C1 and C2. Rendering is post-processing only: simulations never plot.

if nargin < 2 || isempty(options), options = struct(); end
if ~isfield(options, 'outputDirectory'), options.outputDirectory = ''; end
if ~isfield(options, 'visible'), options.visible = false; end
validateattributes(suiteDirectory, {'char', 'string'}, {'nonempty'});
validateattributes(options.outputDirectory, {'char', 'string'}, {'scalartext'});
validateattributes(options.visible, {'logical', 'numeric'}, {'scalar'});
suiteDirectory = agc.io.resolveResultSuite(suiteDirectory);
options.outputDirectory = char(options.outputDirectory);
options.visible = logical(options.visible);

%% Discover and validate the complete controller comparison matrix

if isempty(options.outputDirectory)
    outputDirectory = fullfile(suiteDirectory, 'figures');
else
    outputDirectory = options.outputDirectory;
end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end

entries = loadSuite(suiteDirectory);
modes = ["nominal", "euclidean", "bregman"];
forms = ["c1", "c2"];
requireVariants(entries, modes, forms);

%% Export the fixed publication figure set

specifications = [ ...
    struct('name', 'trajectory', 'draw', @() drawTrajectory(entries, modes, forms, options.visible)); ...
    struct('name', 'tracking', 'draw', @() drawTracking(entries, modes, forms, options.visible)); ...
    struct('name', 'transverse', 'draw', @() drawTransverse(entries, modes, forms, options.visible)); ...
    struct('name', 'adaptive_comparison', 'draw', @() drawAdaptiveComparison(entries, forms, options.visible)); ...
    struct('name', 'performance_summary', 'draw', @() drawPerformance(entries, modes, forms, options.visible)); ...
    struct('name', 'adaptation', 'draw', @() drawAdaptation(entries, forms, options.visible))];

output = repmat(struct('name', '', 'directory', outputDirectory, 'files', struct()), numel(specifications), 1);
for k = 1:numel(specifications)
    figureHandle = specifications(k).draw();
    output(k) = exportStatic(figureHandle, specifications(k).name, outputDirectory);
    close(figureHandle);
end
end

function entries = loadSuite(suiteDirectory)
%LOADSUITE Read every direct child result directory containing run.mat.

children = dir(suiteDirectory);
entries = struct('mode', {}, 'coriolis', {}, 'scenario', {}, 'run', {}, 'metrics', {});
for k = 1:numel(children)
    if ~children(k).isdir || startsWith(children(k).name, '.')
        continue;
    end
    resultDirectory = fullfile(suiteDirectory, children(k).name);
    if ~isfile(fullfile(resultDirectory, 'run.mat'))
        continue;
    end
    payload = agc.io.loadRun(resultDirectory);
    entries(end + 1) = struct('mode', string(payload.run.mode), ... %#ok<AGROW>
        'coriolis', string(payload.run.coriolis), 'scenario', payload.scenario, ...
        'run', payload.run, 'metrics', payload.metrics);
end
end

function requireVariants(entries, modes, forms)
%REQUIREVARIANTS Fail early when a paper comparison would be incomplete.

for mode = modes
    for form = forms
        if isempty(selectEntry(entries, mode, form))
            error('agc:viz:paperFigures:MissingVariant', ...
                'Missing saved run for %s/%s.', mode, upper(form));
        end
    end
end
end

function entry = selectEntry(entries, mode, form)
%SELECTENTRY Return one saved run by controller mode and C-factorization.

matches = find([entries.mode] == mode & [entries.coriolis] == form);
if numel(matches) ~= 1
    entry = [];
else
    entry = entries(matches);
end
end

function figureHandle = drawTrajectory(entries, modes, forms, visible)
figureHandle = newFigure('Trajectory tracking', visible);
layout = tiledlayout(1, numel(modes), 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(modes)
    ax = nexttile(layout); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
    first = selectEntry(entries, modes(k), forms(1));
    desired = positions(first.run.Hdesired);
    plot3(ax, desired(:,1), desired(:,2), desired(:,3), 'k--', 'LineWidth', 1.2, 'DisplayName', 'desired');
    for form = forms
        entry = selectEntry(entries, modes(k), form);
        actual = positions(entry.run.H);
        plot3(ax, actual(:,1), actual(:,2), actual(:,3), lineSpec(modes(k), form), ...
            'Color', modeColor(modes(k)), 'LineWidth', 1.5, 'DisplayName', upper(form));
    end
    title(ax, capitalize(modes(k))); xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
    legend(ax, 'Location', 'best');
end
end

function figureHandle = drawTracking(entries, modes, forms, visible)
figureHandle = newFigure('Tracking errors', visible);
layout = tiledlayout(numel(modes), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(modes)
    axPosition = nexttile(layout); hold(axPosition, 'on'); grid(axPosition, 'on');
    axAttitude = nexttile(layout); hold(axAttitude, 'on'); grid(axAttitude, 'on');
    for form = forms
        entry = selectEntry(entries, modes(k), form);
        signal = runSignals(entry);
        plot(axPosition, entry.run.t, signal.positionError, lineSpec(modes(k), form), ...
            'Color', modeColor(modes(k)), 'LineWidth', 1.4, 'DisplayName', upper(form));
        plot(axAttitude, entry.run.t, signal.attitudeError, lineSpec(modes(k), form), ...
            'Color', modeColor(modes(k)), 'LineWidth', 1.4, 'DisplayName', upper(form));
    end
    title(axPosition, sprintf('%s: position', capitalize(modes(k)))); ylabel(axPosition, '||p-p_d|| (m)');
    title(axAttitude, sprintf('%s: attitude', capitalize(modes(k)))); ylabel(axAttitude, 'theta_R (rad)');
    if k == numel(modes), xlabel(axPosition, 'time (s)'); xlabel(axAttitude, 'time (s)'); end
    legend(axPosition, 'Location', 'best'); legend(axAttitude, 'Location', 'best');
end
end

function figureHandle = drawTransverse(entries, modes, forms, visible)
figureHandle = newFigure('Transverse convergence', visible);
layout = tiledlayout(numel(modes), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(modes)
    axS = nexttile(layout); hold(axS, 'on'); grid(axS, 'on');
    axVs = nexttile(layout); hold(axVs, 'on'); grid(axVs, 'on');
    for form = forms
        entry = selectEntry(entries, modes(k), form);
        signal = runSignals(entry);
        semilogy(axS, entry.run.t, floorPositive(signal.slidingNorm), lineSpec(modes(k), form), ...
            'Color', modeColor(modes(k)), 'LineWidth', 1.4, 'DisplayName', upper(form));
        semilogy(axVs, entry.run.t, floorPositive(entry.run.Vs), lineSpec(modes(k), form), ...
            'Color', modeColor(modes(k)), 'LineWidth', 1.4, 'DisplayName', upper(form));
    end
    title(axS, sprintf('%s: sliding residual', capitalize(modes(k)))); ylabel(axS, '||s||_{Lambda^{-1}}');
    title(axVs, sprintf('%s: transverse energy', capitalize(modes(k)))); ylabel(axVs, 'V_s');
    if k == numel(modes), xlabel(axS, 'time (s)'); xlabel(axVs, 'time (s)'); end
    legend(axS, 'Location', 'best'); legend(axVs, 'Location', 'best');
end
end

function figureHandle = drawAdaptiveComparison(entries, forms, visible)
figureHandle = newFigure('Euclidean versus Bregman adaptation', visible);
layout = tiledlayout(2, numel(forms), 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(forms)
    axTracking = nexttile(layout); hold(axTracking, 'on'); grid(axTracking, 'on');
    axSliding = nexttile(layout); hold(axSliding, 'on'); grid(axSliding, 'on');
    for mode = ["euclidean", "bregman"]
        entry = selectEntry(entries, mode, forms(k));
        signal = runSignals(entry);
        plot(axTracking, entry.run.t, signal.positionError, '-', 'Color', modeColor(mode), ...
            'LineWidth', 1.5, 'DisplayName', capitalize(mode));
        semilogy(axSliding, entry.run.t, floorPositive(signal.slidingNorm), '-', 'Color', modeColor(mode), ...
            'LineWidth', 1.5, 'DisplayName', capitalize(mode));
    end
    title(axTracking, sprintf('%s: tracking', upper(forms(k)))); ylabel(axTracking, '||p-p_d|| (m)');
    title(axSliding, sprintf('%s: sliding', upper(forms(k)))); ylabel(axSliding, '||s||_{Lambda^{-1}}'); xlabel(axSliding, 'time (s)');
    legend(axTracking, 'Location', 'best'); legend(axSliding, 'Location', 'best');
end
end

function figureHandle = drawPerformance(entries, modes, forms, visible)
figureHandle = newFigure('Performance summary', visible);
layout = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
labels = strings(1, numel(modes) * numel(forms));
position = zeros(size(labels)); attitude = zeros(size(labels)); effort = zeros(size(labels)); colors = zeros(numel(labels), 3);
index = 0;
for mode = modes
    for form = forms
        index = index + 1;
        entry = selectEntry(entries, mode, form);
        labels(index) = capitalize(mode) + " " + upper(form);
        position(index) = entry.metrics.positionRMSE;
        attitude(index) = entry.metrics.attitudeRMSE;
        effort(index) = entry.metrics.wrenchRMS;
        colors(index,:) = modeColor(mode);
    end
end
drawBars(nexttile(layout), position, labels, colors, 'Position RMSE (m)');
drawBars(nexttile(layout), attitude, labels, colors, 'Attitude RMSE (rad)');
drawBars(nexttile(layout), effort, labels, colors, 'Wrench RMS');
end

function figureHandle = drawAdaptation(entries, forms, visible)
figureHandle = newFigure('Adaptive estimator behavior', visible);
layout = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
axEuclidean = nexttile(layout); hold(axEuclidean, 'on'); grid(axEuclidean, 'on');
axBregman = nexttile(layout); hold(axBregman, 'on'); grid(axBregman, 'on');
axSPD = nexttile(layout); hold(axSPD, 'on'); grid(axSPD, 'on');
for form = forms
    euclidean = selectEntry(entries, "euclidean", form);
    bregman = selectEntry(entries, "bregman", form);
    eSignal = runSignals(euclidean);
    bSignal = runSignals(bregman);
    plot(axEuclidean, euclidean.run.t, eSignal.parameterMismatch, lineSpec("euclidean", form), ...
        'Color', modeColor("euclidean"), 'LineWidth', 1.4, 'DisplayName', upper(form));
    plot(axBregman, bregman.run.t, bSignal.parameterMismatch, lineSpec("bregman", form), ...
        'Color', modeColor("bregman"), 'LineWidth', 1.4, 'DisplayName', upper(form));
    semilogy(axSPD, bregman.run.t, floorPositive(bregman.run.minPseudoEigenvalue), lineSpec("bregman", form), ...
        'Color', modeColor("bregman"), 'LineWidth', 1.4, 'DisplayName', upper(form));
end
title(axEuclidean, 'Euclidean mismatch'); ylabel(axEuclidean, 'normalized ||piHat-pi||'); xlabel(axEuclidean, 'time (s)'); legend(axEuclidean, 'Location', 'best');
title(axBregman, 'Bregman mismatch'); ylabel(axBregman, 'normalized ||piHat-pi||'); xlabel(axBregman, 'time (s)'); legend(axBregman, 'Location', 'best');
title(axSPD, 'Bregman SPD margin'); ylabel(axSPD, 'min eig(JHat)'); xlabel(axSPD, 'time (s)'); yline(axSPD, eps, 'k:', 'positive boundary'); legend(axSPD, 'Location', 'best');
end

function signal = runSignals(entry)
%RUNSIGNALS Derive plot-ready paper diagnostics from a saved run.

run = entry.run;
n = numel(run.t);
positionError = zeros(n,1);
attitudeError = zeros(n,1);
for k = 1:n
    H = run.H(:,:,k); Hd = run.Hdesired(:,:,k);
    positionError(k) = norm(H(1:3,4) - Hd(1:3,4));
    Re = Hd(1:3,1:3).' * H(1:3,1:3);
    attitudeError(k) = acos(max(-1, min(1, (trace(Re) - 1) / 2)));
end

if isfield(entry.scenario, 'controller') && isfield(entry.scenario.controller, 'Lambda')
    LambdaInverse = entry.scenario.controller.Lambda \ eye(6);
else
    LambdaInverse = eye(6);
end
slidingNorm = sqrt(max(0, sum((run.s * LambdaInverse) .* run.s, 2)));

piTrue = entry.scenario.plantPi(:).';
scale = max(abs(piTrue), 1e-6 * max(1, max(abs(piTrue))));
parameterMismatch = sqrt(mean(((run.estimatePi - piTrue) ./ scale).^2, 2));

signal = struct('positionError', positionError, 'attitudeError', attitudeError, ...
    'slidingNorm', slidingNorm, 'parameterMismatch', parameterMismatch);
end

function drawBars(ax, values, labels, colors, ylabelText)
barHandle = bar(ax, values, 0.75);
barHandle.FaceColor = 'flat';
barHandle.CData = colors;
grid(ax, 'on'); ylabel(ax, ylabelText); xticks(ax, 1:numel(labels));
xticklabels(ax, labels); xtickangle(ax, 35);
end

function output = exportStatic(figureHandle, name, directory)
pngFile = fullfile(directory, [name, '.png']);
pdfFile = fullfile(directory, [name, '.pdf']);
exportgraphics(figureHandle, pngFile, 'Resolution', 300);
exportgraphics(figureHandle, pdfFile, 'ContentType', 'vector');
output = struct('name', name, 'directory', directory, 'files', struct('png', pngFile, 'pdf', pdfFile));
end

function figureHandle = newFigure(name, visible)
visibility = 'off';
if visible, visibility = 'on'; end
figureHandle = figure('Name', name, 'Color', 'w', 'Visible', visibility, 'Position', [100, 100, 1300, 720]);
end

function values = positions(H)
values = squeeze(H(1:3,4,:)).';
end

function value = floorPositive(value)
value = max(value, 1e-12);
end

function color = modeColor(mode)
switch lower(string(mode))
    case "nominal"
        color = [0.00, 0.45, 0.74];
    case "euclidean"
        color = [0.85, 0.33, 0.10];
    case "bregman"
        color = [0.49, 0.18, 0.56];
end
end

function style = lineSpec(~, form)
if strcmpi(form, 'c1'), style = '--'; else, style = '-'; end
end

function value = capitalize(value)
value = char(string(value));
value(1) = upper(value(1));
end
