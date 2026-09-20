function metrics = metrics(run)
%METRICS Compute stable, unit-aware tracking and theory diagnostics.
n = numel(run.t);
positionError = zeros(n,3);
attitudeError = zeros(n,1);
for k = 1:n
    H = run.H(:,:,k); Hd = run.Hdesired(:,:,k);
    positionError(k,:) = (H(1:3,4) - Hd(1:3,4)).';
    Re = Hd(1:3,1:3).' * H(1:3,1:3);
    attitudeError(k) = acos(max(-1, min(1, (trace(Re) - 1) / 2)));
end
metrics = struct();
metrics.positionRMSE = sqrt(mean(sum(positionError.^2, 2)));
metrics.attitudeRMSE = sqrt(mean(attitudeError.^2));
metrics.maxPositionError = max(vecnorm(positionError, 2, 2));
metrics.wrenchRMS = sqrt(mean(sum(run.wrench.^2, 2)));
metrics.finalSlidingNorm = norm(run.s(end,:));
metrics.maxPsi = max(run.Psi);
metrics.maxVs = max(run.Vs);
if all(isnan(run.minPseudoEigenvalue))
    metrics.minimumPseudoEigenvalue = NaN;
else
    metrics.minimumPseudoEigenvalue = min(run.minPseudoEigenvalue, [], 'omitnan');
end
end
