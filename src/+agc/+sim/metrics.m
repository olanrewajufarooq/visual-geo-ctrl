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
velocityError = run.V - run.Vdesired;
metrics.angularVelocityRMSE = sqrt(mean(sum(velocityError(:,1:3).^2, 2)));
metrics.linearVelocityRMSE = sqrt(mean(sum(velocityError(:,4:6).^2, 2)));
metrics.maxPositionError = max(vecnorm(positionError, 2, 2));
metrics.wrenchRMS = sqrt(mean(sum(run.wrench.^2, 2)));
% Use one initial true-parameter scale so all ten inertial coordinates enter
% the adaptation diagnostic without component-wise division artifacts.
if isfield(run, 'activePlantPi') && isfield(run, 'estimatePi')
    truePi = run.activePlantPi;
    estimatePi = run.estimatePi;

    % Report physically meaningful groups separately. Mass and center of mass
    % describe the payload-induced change; rotational inertia is kept as its
    % own lower-priority identification term in the optimizer.
    massScale = max(abs(truePi(1,1)), 1);
    massError = (estimatePi(:,1) - truePi(:,1)) / massScale;
    trueMass = max(abs(truePi(:,1)), eps);
    estimateMass = max(abs(estimatePi(:,1)), eps);
    trueCog = truePi(:,2:4) ./ trueMass;
    estimateCog = estimatePi(:,2:4) ./ estimateMass;
    cogScale = max(norm(trueCog(1,:)), 0.1);
    cogError = vecnorm(estimateCog - trueCog, 2, 2) / cogScale;
    inertiaScale = max(norm(truePi(1,5:10)), 1);
    inertiaError = vecnorm(estimatePi(:,5:10) - truePi(:,5:10), 2, 2) / inertiaScale;

    metrics.massEstimationRMSE = sqrt(mean(massError.^2));
    metrics.centerOfMassEstimationRMSE = sqrt(mean(cogError.^2));
    metrics.massCogEstimationRMSE = sqrt(mean(0.5 * (massError.^2 + cogError.^2)));
    metrics.inertiaEstimationRMSE = sqrt(mean(inertiaError.^2));

    parameterScale = max(norm(truePi(1,:)), 1);
    parameterError = vecnorm(estimatePi - truePi, 2, 2) / parameterScale;
    metrics.parameterEstimationRMSE = sqrt(mean(parameterError.^2));
else
    metrics.massEstimationRMSE = NaN;
    metrics.centerOfMassEstimationRMSE = NaN;
    metrics.massCogEstimationRMSE = NaN;
    metrics.inertiaEstimationRMSE = NaN;
    metrics.parameterEstimationRMSE = NaN;
end
metrics.finalSlidingNorm = norm(run.s(end,:));
metrics.maxPsi = max(run.Psi);
metrics.maxVs = max(run.Vs);
if all(isnan(run.minPseudoEigenvalue))
    metrics.minimumPseudoEigenvalue = NaN;
else
    metrics.minimumPseudoEigenvalue = min(run.minPseudoEigenvalue, [], 'omitnan');
end
end
