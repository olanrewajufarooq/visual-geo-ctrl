function record = evaluateScenarioCandidate(candidate, baseScenario, weights, useParallel, label)
%EVALUATESCENARIOCANDIDATE Round and score one complete gain candidate.

if nargin < 4 || isempty(useParallel), useParallel = false; end
if nargin < 5 || isempty(label), label = 'candidate'; end
[publishedCandidate, gains] = roundedCandidate(candidate, baseScenario);
[cost, detail] = agc.opt.objective(publishedCandidate, {baseScenario}, ...
    @agc.opt.applyScenarioGains, weights, useParallel);
record = struct('rawCandidate', candidate(:).', 'candidate', publishedCandidate, 'gains', gains, ...
    'cost', cost, 'failed', detail.failed || ~isfinite(cost), 'label', char(label), 'detail', detail);
end

function [candidate, gains] = roundedCandidate(rawCandidate, baseScenario)
scenario = agc.opt.applyScenarioGains(rawCandidate, {baseScenario});
controller = scenario{1}.controller;
gains = struct('KRdiag', diag(controller.KR).', 'Kxidiag', diag(controller.Kxi).', ...
    'LambdaDiag', diag(controller.Lambda).', 'kd', controller.kd, 'ks', controller.ks, ...
    'alpha', controller.alpha, 'gammaE', controller.gammaE(:), 'gammaB', controller.gammaB);
gains = agc.opt.roundGains(gains, 4);
controller.KR = diag(gains.KRdiag);
controller.Kxi = diag(gains.Kxidiag);
controller.Lambda = diag(gains.LambdaDiag);
controller.kd = gains.kd;
controller.ks = gains.ks;
controller.alpha = gains.alpha;
controller.gammaE = gains.gammaE;
controller.gammaB = gains.gammaB;
scenario{1}.controller = controller;
candidate = agc.opt.encodeScenarioGains(scenario{1});
end
