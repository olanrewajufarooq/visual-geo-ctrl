function gains = optimized_gains(mode, coriolis)
%OPTIMIZED_GAINS Automatically maintained per-scenario gain registry.
% Last promotion: 2026-09-21 20:20:47.
key = sprintf('%s_%s', lower(char(string(mode))), lower(char(string(coriolis))));
switch key
    case 'nominal_c1'
        gains = struct('KRdiag', [2.197 0.09943 2.517], 'Kxidiag', [39.74 8.85 7.891], 'LambdaDiag', [4.315 9.267 3.333 0.2074 3.552 3.315], 'kd', 0.6467, 'ks', 15.62, 'alpha', 0.5587, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'nominal_c2'
        gains = struct('KRdiag', [0.1819 0.01 2.596], 'Kxidiag', [85.36 3.784 100], 'LambdaDiag', [38.73 26.23 5.068 0.1638 0.2499 0.1605], 'kd', 24.6, 'ks', 1.651, 'alpha', 0.1439, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'euclidean_c1'
        gains = struct('KRdiag', [0.01926 0.7074 0.7917], 'Kxidiag', [3.283 69.9 5.843], 'LambdaDiag', [20.49 16.85 81.68 0.3738 0.5646 0.4835], 'kd', 6.38, 'ks', 13.01, 'alpha', 0.5104, 'gammaE', [1;0.3032;0.2151;0.1401;1e-05;1.046e-05;0.03055;1.319e-05;0.001107;0.001085], 'gammaB', 0.001);
    case 'euclidean_c2'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'bregman_c1'
        gains = struct('KRdiag', [0.9401 0.01072 1.282], 'Kxidiag', [11.31 3.765 5.802], 'LambdaDiag', [18.28 18.77 14.41 0.7082 0.3636 0.7342], 'kd', 6.108, 'ks', 12.93, 'alpha', 0.4534, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.09003);
    case 'bregman_c2'
        gains = struct('KRdiag', [0.01887 0.00132 0.001561], 'Kxidiag', [11.71 11.34 299.5], 'LambdaDiag', [18.56 34.92 11.97 0.08278 0.1263 0.4643], 'kd', 16.66, 'ks', 0.001917, 'alpha', 0.1386, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.08016);
    otherwise
        error('agc:config:optimized_gains:UnknownScenario', 'Unknown scenario key: %s.', key);
end
end
