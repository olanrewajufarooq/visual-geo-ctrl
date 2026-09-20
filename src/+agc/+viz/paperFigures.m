function output = paperFigures(suiteDirectory, options)
%PAPERFIGURES Export comparison and standalone figures from a saved suite.
%
% Figures are post-processing only. By default, comparisons are written to
% <suite>/comparisons and a run's diagnostics stay beside its run.mat file
% in <suite>/<mode>_<coriolis>/figures.

if nargin < 2 || isempty(options), options = struct(); end
options = defaultOptions(options);
suiteDirectory = agc.io.resolveResultSuite(suiteDirectory);
entries = loadSuite(suiteDirectory);
modes = ["nominal", "euclidean", "bregman"];
forms = ["c1", "c2"];
if isempty(entries)
    error('agc:viz:paperFigures:NoSavedRuns', ...
        'No saved runs were found in: %s', suiteDirectory);
end

output = struct('name', {}, 'directory', {}, 'files', {});

%% Direct comparisons: only signals with the same units share an axes

if options.exportComparisons
    comparisonRoot = resultDirectory(suiteDirectory, options, 'comparisons');
    comparisonDirectory = comparisonDirectories(comparisonRoot);
    comparisonNames = fieldnames(comparisonDirectory);
    for k = 1:numel(comparisonNames)
        directory = comparisonDirectory.(comparisonNames{k});
        if ~isfolder(directory), mkdir(directory); end
    end

    for mode = modes
        c1 = selectEntry(entries, mode, "c1");
        c2 = selectEntry(entries, mode, "c2");
        if isempty(c1) || isempty(c2), continue; end
        prefix = char(mode + "_");
        directory = comparisonDirectory.(char(mode));
        output(end + 1) = savePlot([prefix 'trajectory_c1_c2'], directory, options, ...
            @() plotTrajectoryComparison(c1, c2)); %#ok<AGROW>
        output(end + 1) = savePlot([prefix 'position_error_c1_c2'], directory, options, ...
            @() plotScalarComparison(c1, c2, 'positionError', 'Position error', '||p-p_d|| (m)', false)); %#ok<AGROW>
        output(end + 1) = savePlot([prefix 'attitude_error_c1_c2'], directory, options, ...
            @() plotScalarComparison(c1, c2, 'attitudeError', 'Attitude error', 'theta_R (rad)', false)); %#ok<AGROW>
        output(end + 1) = savePlot([prefix 'sliding_norm_c1_c2'], directory, options, ...
            @() plotScalarComparison(c1, c2, 'slidingNorm', 'Sliding residual', '||s||_{Lambda^{-1}}', true)); %#ok<AGROW>
        output(end + 1) = savePlot([prefix 'transverse_energy_c1_c2'], directory, options, ...
            @() plotScalarComparison(c1, c2, 'Vs', 'Transverse energy', 'V_s', true)); %#ok<AGROW>
        output(end + 1) = savePlot([prefix 'wrench_norm_c1_c2'], directory, options, ...
            @() plotScalarComparison(c1, c2, 'wrenchNorm', 'Control-wrench norm', '||W_c||', false)); %#ok<AGROW>
    end

    for form = forms
        euclidean = selectEntry(entries, "euclidean", form);
        bregman = selectEntry(entries, "bregman", form);
        if isempty(euclidean) || isempty(bregman), continue; end
        suffix = ['_' char(form)];
        output(end + 1) = savePlot(['euclidean_vs_bregman_position_error' suffix], comparisonDirectory.euclidean_v_bregman, options, ...
            @() plotAdaptiveComparison(euclidean, bregman, 'positionError', 'Position error', '||p-p_d|| (m)', false)); %#ok<AGROW>
        output(end + 1) = savePlot(['euclidean_vs_bregman_attitude_error' suffix], comparisonDirectory.euclidean_v_bregman, options, ...
            @() plotAdaptiveComparison(euclidean, bregman, 'attitudeError', 'Attitude error', 'theta_R (rad)', false)); %#ok<AGROW>
        output(end + 1) = savePlot(['euclidean_vs_bregman_sliding_norm' suffix], comparisonDirectory.euclidean_v_bregman, options, ...
            @() plotAdaptiveComparison(euclidean, bregman, 'slidingNorm', 'Sliding residual', '||s||_{Lambda^{-1}}', true)); %#ok<AGROW>
        output(end + 1) = savePlot(['euclidean_vs_bregman_parameter_error' suffix], comparisonDirectory.euclidean_v_bregman, options, ...
            @() plotAdaptiveComparison(euclidean, bregman, 'parameterError', 'Parameter error', 'normalized ||piHat-pi||', false)); %#ok<AGROW>
        output(end + 1) = savePlot(['euclidean_vs_bregman_pseudo_inertia_margin' suffix], comparisonDirectory.euclidean_v_bregman, options, ...
            @() plotAdaptiveComparison(euclidean, bregman, 'pseudoMargin', 'Pseudo-inertia margin', 'min eig(JHat)', false)); %#ok<AGROW>
    end

    % Batch metrics remain useful, but each metric is a separate figure so its
    % scale and units are never conflated with an unrelated performance measure.
    output(end + 1) = savePlot('performance_position_rmse', comparisonDirectory.performance, options, ...
        @() plotMetricSummary(entries, modes, forms, 'positionRMSE', 'Position RMSE', 'RMSE (m)'));
    output(end + 1) = savePlot('performance_attitude_rmse', comparisonDirectory.performance, options, ...
        @() plotMetricSummary(entries, modes, forms, 'attitudeRMSE', 'Attitude RMSE', 'RMSE (rad)'));
    output(end + 1) = savePlot('performance_wrench_rms', comparisonDirectory.performance, options, ...
        @() plotMetricSummary(entries, modes, forms, 'wrenchRMS', 'Control-wrench RMS', 'RMS wrench'));
end

%% Standalone diagnostics: every signal can be inspected without a composite

for k = 1:numel(entries)
    entry = entries(k);
    variant = sprintf('%s_%s', entry.mode, entry.coriolis);
    directory = diagnosticDirectory(entry, suiteDirectory, options, variant);
    if ~isfolder(directory), mkdir(directory); end
    output = [output, exportRunDiagnostics(entry, variant, directory, options)]; %#ok<AGROW>
end
end

function options = defaultOptions(options)
%DEFAULTOPTIONS Normalize public exporter options.

if ~isfield(options, 'outputDirectory'), options.outputDirectory = ''; end
if ~isfield(options, 'visible'), options.visible = false; end
if ~isfield(options, 'exportComparisons'), options.exportComparisons = true; end
validateattributes(options.outputDirectory, {'char', 'string'}, {'scalartext'});
validateattributes(options.visible, {'logical', 'numeric'}, {'scalar'});
validateattributes(options.exportComparisons, {'logical', 'numeric'}, {'scalar'});
options.outputDirectory = char(options.outputDirectory);
options.visible = logical(options.visible);
options.exportComparisons = logical(options.exportComparisons);
end

function entries = loadSuite(suiteDirectory)
%LOADSUITE Load every immediate saved variant from a result suite.

children = dir(suiteDirectory);
entries = repmat(struct('mode', "", 'coriolis', "", 'directory', '', ...
    'scenario', struct(), 'run', struct(), 'metrics', struct()), numel(children), 1);
count = 0;
for k = 1:numel(children)
    if ~children(k).isdir || startsWith(children(k).name, '.'), continue; end
    directory = fullfile(suiteDirectory, children(k).name);
    if ~isfile(fullfile(directory, 'run.mat')), continue; end
    payload = agc.io.loadRun(directory);
    count = count + 1;
    entries(count) = struct('mode', string(payload.run.mode), ...
        'coriolis', string(payload.run.coriolis), 'directory', directory, ...
        'scenario', payload.scenario, ...
        'run', payload.run, 'metrics', payload.metrics);
end

entries = entries(1:count);
end

function directory = resultDirectory(suiteDirectory, options, name)
%RESULTDIRECTORY Resolve the suite-level comparison destination.

if isempty(options.outputDirectory)
    directory = fullfile(suiteDirectory, name);
else
    directory = fullfile(suiteDirectory, options.outputDirectory, name);
end
end

function directories = comparisonDirectories(root)
%COMPARISONDIRECTORIES Group suite-level comparisons by scientific question.

directories = struct('nominal', fullfile(root, 'nominal'), ...
    'euclidean', fullfile(root, 'euclidean'), ...
    'bregman', fullfile(root, 'bregman'), ...
    'euclidean_v_bregman', fullfile(root, 'euclidean_v_bregman'), ...
    'performance', fullfile(root, 'performance'));
end

function directory = diagnosticDirectory(entry, suiteDirectory, options, variant)
%DIAGNOSTICDIRECTORY Keep diagnostics with their run unless redirected.

if isempty(options.outputDirectory)
    directory = fullfile(entry.directory, 'figures');
else
    directory = fullfile(suiteDirectory, options.outputDirectory, 'diagnostics', variant);
end
end

function entry = selectEntry(entries, mode, form)
%SELECTENTRY Return one unambiguous mode/factorization result.

matches = find([entries.mode] == mode & [entries.coriolis] == form);
if numel(matches) ~= 1, entry = []; else, entry = entries(matches); end
end

function output = exportRunDiagnostics(entry, variant, directory, options)
%EXPORTRUNDIAGNOSTICS Export readable figures for exactly one saved run.

output = struct('name', {}, 'directory', {}, 'files', {});
signal = diagnostics(entry);
specification = [ ...
    struct('name', 'trajectory_3d', 'draw', @() plotTrajectory(entry)); ...
    struct('name', 'trajectory_xy', 'draw', @() plotXY(entry)); ...
    struct('name', 'altitude', 'draw', @() plotChannels(entry, signal.position(:,3), signal.desiredPosition(:,3), 'Altitude', 'z (m)', {'z'})); ...
    struct('name', 'position', 'draw', @() plotChannels(entry, signal.position, signal.desiredPosition, 'Cartesian position', 'p (m)', {'x', 'y', 'z'})); ...
    struct('name', 'attitude', 'draw', @() plotChannels(entry, signal.rpy, signal.desiredRpy, 'Attitude', 'angle (rad)', {'roll', 'pitch', 'yaw'})); ...
    struct('name', 'linear_velocity', 'draw', @() plotChannels(entry, entry.run.V(:,4:6), entry.run.Vdesired(:,4:6), 'Body linear velocity', 'v (m/s)', {'v_x', 'v_y', 'v_z'})); ...
    struct('name', 'angular_velocity', 'draw', @() plotChannels(entry, entry.run.V(:,1:3), entry.run.Vdesired(:,1:3), 'Body angular velocity', 'omega (rad/s)', {'omega_x', 'omega_y', 'omega_z'})); ...
    struct('name', 'wrench_force', 'draw', @() plotWrench(entry, 4:6, 'Control force', 'force (N)', {'F_x', 'F_y', 'F_z'})); ...
    struct('name', 'wrench_torque', 'draw', @() plotWrench(entry, 1:3, 'Control torque', 'torque (Nm)', {'tau_x', 'tau_y', 'tau_z'})); ...
    struct('name', 'sliding_norm', 'draw', @() plotSignal(entry, signal.slidingNorm, 'Sliding residual', '||s||_{Lambda^{-1}}', true)); ...
    struct('name', 'potential', 'draw', @() plotSignal(entry, entry.run.Psi, 'Configuration potential', 'Psi', false)); ...
    struct('name', 'transverse_energy', 'draw', @() plotSignal(entry, entry.run.Vs, 'Transverse energy', 'V_s', true))];

if entry.mode ~= "nominal"
    adaptive = [ ...
        struct('name', 'parameter_error', 'draw', @() plotSignal(entry, signal.parameterError, 'Parameter error', 'normalized ||piHat-pi||', false)); ...
        struct('name', 'pseudo_inertia_margin', 'draw', @() plotPseudoMargin(entry, signal.pseudoMargin)); ...
        struct('name', 'mass', 'draw', @() plotEstimate(entry, signal.estimatePi(:,1), signal.truePi(:,1), 'Mass estimate', 'mass (kg)', {'estimate', 'true'})); ...
        struct('name', 'cog', 'draw', @() plotCog(entry, signal)); ...
        struct('name', 'inertia_principal', 'draw', @() plotInertia(entry, signal, 1:3, 'Principal inertia', 'inertia (kg m^2)', {'I_{xx}', 'I_{yy}', 'I_{zz}'})); ...
        struct('name', 'inertia_off_diagonal', 'draw', @() plotInertia(entry, signal, 4:6, 'Off-diagonal inertia', 'inertia (kg m^2)', {'I_{xy}', 'I_{xz}', 'I_{yz}'}))];
    specification = [specification; adaptive];
end

for k = 1:numel(specification)
    output(end + 1) = savePlot([variant '_' specification(k).name], directory, options, specification(k).draw); %#ok<AGROW>
end
end

function output = savePlot(name, directory, options, draw)
%SAVEPLOT Render one scientific claim and export it as PNG and high-res PDF.

figureHandle = draw();
set(figureHandle, 'Visible', onOff(options.visible));
output = exportStatic(figureHandle, name, directory);
close(figureHandle);
end

function figureHandle = plotTrajectoryComparison(c1, c2)
figureHandle = newFigure('C1/C2 trajectory comparison'); ax = axes(figureHandle); hold(ax, 'on');
desired = diagnostics(c1).desiredPosition;
plot3(ax, desired(:,1), desired(:,2), desired(:,3), 'k--', 'LineWidth', 1.3, 'DisplayName', 'desired');
plot3(ax, diagnostics(c1).position(:,1), diagnostics(c1).position(:,2), diagnostics(c1).position(:,3), '--', 'Color', formColor("c1"), 'LineWidth', 1.5, 'DisplayName', 'C1');
plot3(ax, diagnostics(c2).position(:,1), diagnostics(c2).position(:,2), diagnostics(c2).position(:,3), '-', 'Color', formColor("c2"), 'LineWidth', 1.5, 'DisplayName', 'C2');
style3D(ax, 'Trajectory tracking');
end

function figureHandle = plotScalarComparison(c1, c2, field, titleText, ylabelText, useLog)
figureHandle = newFigure(titleText); ax = axes(figureHandle); hold(ax, 'on');
s1 = diagnostics(c1); s2 = diagnostics(c2);
plotScalar(ax, c1.run.t, s1.(field), '--', formColor("c1"), 'C1', useLog);
plotScalar(ax, c2.run.t, s2.(field), '-', formColor("c2"), 'C2', useLog);
styleTime(ax, titleText, ylabelText);
markPayloadRelease(ax, c1.scenario);
end

function figureHandle = plotAdaptiveComparison(euclidean, bregman, field, titleText, ylabelText, useLog)
figureHandle = newFigure(titleText); ax = axes(figureHandle); hold(ax, 'on');
se = diagnostics(euclidean); sb = diagnostics(bregman);
plotScalar(ax, euclidean.run.t, se.(field), '-', modeColor("euclidean"), 'Euclidean', useLog);
plotScalar(ax, bregman.run.t, sb.(field), '-', modeColor("bregman"), 'Bregman', useLog);
if strcmp(field, 'pseudoMargin'), yline(ax, 0, 'k:', 'zero boundary'); end
styleTime(ax, titleText, ylabelText);
markPayloadRelease(ax, euclidean.scenario);
end

function figureHandle = plotMetricSummary(entries, modes, forms, field, titleText, ylabelText)
%PLOTMETRICSUMMARY Export one batch metric without combining units.

labels = strings(1, 0);
values = zeros(1, 0);
colors = zeros(0, 3);
for mode = modes
    for form = forms
        entry = selectEntry(entries, mode, form);
        if isempty(entry), continue; end
        labels(end + 1) = upper(mode) + " " + upper(form); %#ok<AGROW>
        values(end + 1) = entry.metrics.(field); %#ok<AGROW>
        colors(end + 1,:) = modeColor(mode); %#ok<AGROW>
    end
end
figureHandle = newFigure(titleText); ax = axes(figureHandle);
bars = bar(ax, values, 0.75); bars.FaceColor = 'flat'; bars.CData = colors;
grid(ax, 'on'); box(ax, 'on'); ylabel(ax, ylabelText); title(ax, titleText);
xticks(ax, 1:numel(labels)); xticklabels(ax, labels); xtickangle(ax, 25);
end

function figureHandle = plotTrajectory(entry)
figureHandle = newFigure('Trajectory tracking'); ax = axes(figureHandle); hold(ax, 'on');
signal = diagnostics(entry);
plot3(ax, signal.desiredPosition(:,1), signal.desiredPosition(:,2), signal.desiredPosition(:,3), 'k--', 'LineWidth', 1.3, 'DisplayName', 'desired');
plot3(ax, signal.position(:,1), signal.position(:,2), signal.position(:,3), '-', 'Color', modeColor(entry.mode), 'LineWidth', 1.5, 'DisplayName', 'actual');
style3D(ax, sprintf('%s %s trajectory', upper(entry.mode), upper(entry.coriolis)));
end

function figureHandle = plotXY(entry)
figureHandle = newFigure('XY tracking'); ax = axes(figureHandle); hold(ax, 'on');
signal = diagnostics(entry);
plot(ax, signal.desiredPosition(:,1), signal.desiredPosition(:,2), 'k--', 'LineWidth', 1.3, 'DisplayName', 'desired');
plot(ax, signal.position(:,1), signal.position(:,2), '-', 'Color', modeColor(entry.mode), 'LineWidth', 1.5, 'DisplayName', 'actual');
axis(ax, 'equal'); grid(ax, 'on'); xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); title(ax, 'XY trajectory tracking'); legend(ax, 'Location', 'best');
end

function figureHandle = plotChannels(entry, actual, desired, titleText, ylabelText, labels)
figureHandle = newFigure(titleText); ax = axes(figureHandle); hold(ax, 'on');
colors = lines(size(actual,2));
for k = 1:size(actual,2)
    plot(ax, entry.run.t, actual(:,k), '-', 'Color', colors(k,:), 'LineWidth', 1.4, 'DisplayName', labels{k});
    plot(ax, entry.run.t, desired(:,k), '--', 'Color', colors(k,:), 'LineWidth', 1.1, 'DisplayName', [labels{k} ' desired']);
end
styleTime(ax, titleText, ylabelText);
markPayloadRelease(ax, entry.scenario);
end

function figureHandle = plotWrench(entry, index, titleText, ylabelText, labels)
figureHandle = newFigure(titleText); ax = axes(figureHandle); hold(ax, 'on');
colors = lines(numel(index));
for k = 1:numel(index)
    plot(ax, entry.run.t, entry.run.wrench(:,index(k)), 'Color', colors(k,:), 'LineWidth', 1.4, 'DisplayName', labels{k});
end
styleTime(ax, titleText, ylabelText);
markPayloadRelease(ax, entry.scenario);
end

function figureHandle = plotSignal(entry, values, titleText, ylabelText, useLog)
figureHandle = newFigure(titleText); ax = axes(figureHandle); hold(ax, 'on');
plotScalar(ax, entry.run.t, values, '-', modeColor(entry.mode), upper(entry.coriolis), useLog);
styleTime(ax, titleText, ylabelText);
markPayloadRelease(ax, entry.scenario);
end

function figureHandle = plotPseudoMargin(entry, values)
figureHandle = newFigure('Pseudo-inertia margin'); ax = axes(figureHandle); hold(ax, 'on');
plot(ax, entry.run.t, values, '-', 'Color', modeColor(entry.mode), 'LineWidth', 1.4, 'DisplayName', 'min eig(JHat)');
yline(ax, 0, 'k:', 'zero boundary'); styleTime(ax, 'Pseudo-inertia margin', 'min eig(JHat)');
markPayloadRelease(ax, entry.scenario);
end

function figureHandle = plotEstimate(entry, estimate, truth, titleText, ylabelText, labels)
figureHandle = newFigure(titleText); ax = axes(figureHandle); hold(ax, 'on');
plot(ax, entry.run.t, estimate, '-', 'Color', modeColor(entry.mode), 'LineWidth', 1.4, 'DisplayName', labels{1});
plotTruth(ax, entry.run.t, truth, labels{2}); styleTime(ax, titleText, ylabelText);
markPayloadRelease(ax, entry.scenario);
end

function figureHandle = plotCog(entry, signal)
figureHandle = newFigure('Center-of-mass estimate'); ax = axes(figureHandle); hold(ax, 'on');
colors = lines(3); names = 'xyz';
for k = 1:3
    plot(ax, entry.run.t, signal.cog(:,k), '-', 'Color', colors(k,:), 'LineWidth', 1.4, 'DisplayName', sprintf('c_%s', names(k)));
    plotTruth(ax, entry.run.t, signal.trueCog(:,k), sprintf('c_%s true', names(k)), colors(k,:));
end
styleTime(ax, 'Center-of-mass estimate', 'c_m (m)');
markPayloadRelease(ax, entry.scenario);
end

function figureHandle = plotInertia(entry, signal, index, titleText, ylabelText, labels)
figureHandle = newFigure(titleText); ax = axes(figureHandle); hold(ax, 'on');
colors = lines(numel(index));
for k = 1:numel(index)
    parameter = index(k) + 4;
    plot(ax, entry.run.t, signal.estimatePi(:,parameter), '-', 'Color', colors(k,:), 'LineWidth', 1.4, 'DisplayName', labels{k});
    plotTruth(ax, entry.run.t, signal.truePi(:,parameter), [labels{k} ' true'], colors(k,:));
end
styleTime(ax, titleText, ylabelText);
markPayloadRelease(ax, entry.scenario);
end

function signal = diagnostics(entry)
%DIAGNOSTICS Derive all plot signals from the immutable saved run.

run = entry.run;
n = numel(run.t);
position = squeeze(run.H(1:3,4,:)).';
desiredPosition = squeeze(run.Hdesired(1:3,4,:)).';
rpy = zeros(n,3); desiredRpy = zeros(n,3); attitudeError = zeros(n,1);
for k = 1:n
    rpy(k,:) = fliplr(rotm2eul(run.H(1:3,1:3,k), 'ZYX'));
    desiredRpy(k,:) = fliplr(rotm2eul(run.Hdesired(1:3,1:3,k), 'ZYX'));
    Re = run.Hdesired(1:3,1:3,k).' * run.H(1:3,1:3,k);
    attitudeError(k) = acos(max(-1, min(1, (trace(Re) - 1) / 2)));
end
LambdaInverse = eye(6);
if isfield(entry.scenario, 'controller') && isfield(entry.scenario.controller, 'Lambda')
    LambdaInverse = entry.scenario.controller.Lambda \ eye(6);
end
piTrue = activePlantParameters(entry.scenario, run);
estimatePi = run.estimatePi;
pseudoMargin = zeros(n,1);
for k = 1:n, pseudoMargin(k) = min(eig(agc.math.pseudoFromPi(estimatePi(k,:)))); end
% A global scale avoids making numerically negligible true parameters
% dominate the plot through component-wise division.
parameterError = vecnorm(estimatePi - piTrue, 2, 2) / max(norm(piTrue(1,:)), 1);
signal = struct('position', position, 'desiredPosition', desiredPosition, ...
    'rpy', rpy, 'desiredRpy', desiredRpy, ...
    'positionError', vecnorm(position - desiredPosition, 2, 2), ...
    'attitudeError', attitudeError, ...
    'slidingNorm', sqrt(max(0, sum((run.s * LambdaInverse) .* run.s, 2))), ...
    'Vs', run.Vs, 'wrenchNorm', vecnorm(run.wrench, 2, 2), ...
    'estimatePi', estimatePi, 'truePi', piTrue, ...
    'parameterError', parameterError, ...
    'pseudoMargin', pseudoMargin, ...
    'cog', estimatePi(:,2:4) ./ estimatePi(:,1), ...
    'trueCog', piTrue(:,2:4) ./ piTrue(:,1));
end

function pi = activePlantParameters(scenario, run)
%ACTIVEPLANTPARAMETERS Support legacy fixed runs and payload-drop histories.

if isfield(run, 'activePlantPi')
    pi = run.activePlantPi;
else
    pi = repmat(scenario.plantPi(:).', numel(run.t), 1);
end
end

function plotScalar(ax, time, values, style, color, label, useLog)
if useLog
    semilogy(ax, time, max(values, 1e-12), style, 'Color', color, 'LineWidth', 1.4, 'DisplayName', label);
else
    plot(ax, time, values, style, 'Color', color, 'LineWidth', 1.4, 'DisplayName', label);
end
end

function style3D(ax, titleText)
grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3); xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)'); title(ax, titleText); legend(ax, 'Location', 'best');
end

function styleTime(ax, titleText, ylabelText)
grid(ax, 'on'); box(ax, 'on'); xlabel(ax, 'time (s)'); ylabel(ax, ylabelText); title(ax, titleText); legend(ax, 'Location', 'best');
end

function markPayloadRelease(ax, scenario)
%MARKPAYLOADRELEASE Annotate time-domain diagnostics for hybrid experiments.

if isfield(scenario, 'payloadDrop')
    xline(ax, scenario.payloadDrop.releaseTime, 'k:', 'payload release', ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
end

function plotTruth(ax, time, truth, label, color)
%PLOTTRUTH Draw scalar or sampled physical truth with consistent styling.

if nargin < 5, color = [0, 0, 0]; end
if isscalar(truth)
    yline(ax, truth, 'k--', label);
else
    plot(ax, time, truth, '--', 'Color', color, 'LineWidth', 1.1, 'DisplayName', label);
end
end

function output = exportStatic(figureHandle, name, directory)
pngFile = fullfile(directory, [name '.png']); pdfFile = fullfile(directory, [name '.pdf']);
exportgraphics(figureHandle, pngFile, 'Resolution', 300);
exportgraphics(figureHandle, pdfFile, 'ContentType', 'image', 'Resolution', 300);
output = struct('name', name, 'directory', directory, 'files', struct('png', pngFile, 'pdf', pdfFile));
end

function figureHandle = newFigure(name)
figureHandle = figure('Name', name, 'Color', 'w', 'Visible', 'off', 'Position', [100, 100, 900, 550]);
end

function color = formColor(form)
if form == "c1", color = [0.00, 0.45, 0.74]; else, color = [0.85, 0.33, 0.10]; end
end

function color = modeColor(mode)
switch lower(string(mode))
    case "nominal", color = [0.00, 0.45, 0.74];
    case "euclidean", color = [0.85, 0.33, 0.10];
    case "bregman", color = [0.49, 0.18, 0.56];
end
end

function value = onOff(condition)
if condition, value = 'on'; else, value = 'off'; end
end
