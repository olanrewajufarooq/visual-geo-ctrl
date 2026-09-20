function gains = optimized_gains(mode, coriolis)
%OPTIMIZED_GAINS Automatically maintained per-scenario gain registry.
% Last promotion: 2026-09-20 21:01:37.
key = sprintf('%s_%s', lower(char(string(mode))), lower(char(string(coriolis))));
switch key
    case 'nominal_c1'
        gains = struct('KRdiag', [0.1819 0.01 2.596], 'Kxidiag', [85.36 3.784 100], 'LambdaDiag', [38.73 26.23 5.068 0.1638 0.2499 0.1605], 'kd', 24.6, 'ks', 1.651, 'alpha', 0.1439, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'nominal_c2'
        gains = struct('KRdiag', [2.197 0.09943 2.517], 'Kxidiag', [39.74 8.85 7.891], 'LambdaDiag', [4.315 9.267 3.333 0.2074 3.552 3.315], 'kd', 0.6467, 'ks', 15.62, 'alpha', 0.5587, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'euclidean_c1'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'euclidean_c2'
        gains = struct('KRdiag', [0.007179 0.6227 1.506], 'Kxidiag', [16.89 11.55 53.48], 'LambdaDiag', [23.9 34.43 38.99 0.9233 2.249 0.2175], 'kd', 39.17, 'ks', 3.503, 'alpha', 0.643, 'gammaE', [0.1126;0.1436;0.1544;0.01352;0.03343;0.0006179;0.0023;0.002673;0.09363;0.01932], 'gammaB', 0.001);
    case 'bregman_c1'
        gains = struct('KRdiag', [0.8752 0.1478 3.518], 'Kxidiag', [100 43.03 20.77], 'LambdaDiag', [1.405 4.572 3.305 0.1037 0.04545 0.4213], 'kd', 0.5001, 'ks', 10.52, 'alpha', 0.9, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 1e-05);
    case 'bregman_c2'
        gains = struct('KRdiag', [3.378 0.5416 1.794], 'Kxidiag', [78.73 19.6 16.54], 'LambdaDiag', [7.413 9.499 3.643 0.1438 0.5261 0.4154], 'kd', 9.157, 'ks', 9.923, 'alpha', 0.6201, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.01585);
    otherwise
        error('agc:config:optimized_gains:UnknownScenario', 'Unknown scenario key: %s.', key);
end
end
