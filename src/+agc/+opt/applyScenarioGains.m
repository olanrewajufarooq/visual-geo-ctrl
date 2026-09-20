function scenarios = applyScenarioGains(candidate, scenarios)
%APPLYSCENARIOGAINS Decode one log-space all-gain candidate into scenarios.
%
% Positive gain order: KRdiag(3), Kxidiag(3), LambdaDiag(6), kd, ks,
% then gammaE(10) for Euclidean or gammaB(1) for Bregman. Alpha is direct.

if ~iscell(scenarios), scenarios = {scenarios}; end
if isempty(scenarios)
    error('agc:opt:applyScenarioGains:EmptyScenarios', 'At least one scenario is required.');
end

mode = lower(char(string(scenarios{1}.controller.mode)));
for k = 2:numel(scenarios)
    if ~strcmpi(scenarios{k}.controller.mode, mode)
        error('agc:opt:applyScenarioGains:MixedModes', 'All scenarios must use one controller mode.');
    end
end

[positive, alpha, adaptationGain] = decode(candidate, mode);
for k = 1:numel(scenarios)
    controller = scenarios{k}.controller;
    controller.KR = diag(positive(1:3));
    controller.Kxi = diag(positive(4:6));
    controller.Lambda = diag(positive(7:12));
    controller.kd = positive(13);
    controller.ks = positive(14);
    controller.alpha = alpha;
    if strcmp(mode, 'euclidean')
        controller.gammaE = adaptationGain;
    elseif strcmp(mode, 'bregman')
        controller.gammaB = adaptationGain;
    end
    scenarios{k}.controller = controller;
end
end

function [positive, alpha, adaptationGain] = decode(candidate, mode)
candidate = candidate(:).';
switch mode
    case 'nominal'
        expectedLength = 15;
    case 'euclidean'
        expectedLength = 25;
    case 'bregman'
        expectedLength = 16;
end
validateattributes(candidate, {'numeric'}, {'real', 'finite', 'numel', expectedLength});

positive = 10 .^ candidate(1:14);
alpha = candidate(15);
if alpha <= 0 || alpha >= 1
    error('agc:opt:applyScenarioGains:Alpha', 'alpha must lie strictly between zero and one.');
end
if positive(13) <= 0.5
    error('agc:opt:applyScenarioGains:Kd', 'kd must be strictly greater than one half.');
end

adaptationGain = [];
if strcmp(mode, 'euclidean')
    adaptationGain = (10 .^ candidate(16:25)).';
elseif strcmp(mode, 'bregman')
    adaptationGain = 10 ^ candidate(16);
end
end
