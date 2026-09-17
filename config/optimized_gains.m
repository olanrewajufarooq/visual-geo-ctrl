function gains = optimized_gains(mode, coriolis)
%OPTIMIZED_GAINS Automatically maintained per-scenario gain registry.
% Last promotion: 2026-09-17 14:38:28.
key = sprintf('%s_%s', lower(char(string(mode))), lower(char(string(coriolis))));
switch key
    case 'nominal_c1'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'nominal_c2'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'euclidean_c1'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'euclidean_c2'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'bregman_c1'
        gains = struct('KRdiag', [0.030225641349407324 0.090019221077913492 0.68464570320998086], 'Kxidiag', [100 4.1332328550601893 6.380017022159242], 'LambdaDiag', [5.2906513399210811 16.282133280817376 21.446370147614065 0.21649563223641727 0.49558180162048154 1.0462427157163474], 'kd', 2.1233070118586017, 'ks', 18.26299782729577, 'alpha', 0.69680147526869429, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 7.717274538032889e-07);
    case 'bregman_c2'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    otherwise
        error('agc:config:optimized_gains:UnknownScenario', 'Unknown scenario key: %s.', key);
end
end
