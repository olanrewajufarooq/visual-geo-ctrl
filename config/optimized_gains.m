function gains = optimized_gains(mode, coriolis)
%OPTIMIZED_GAINS Automatically maintained per-scenario gain registry.
% Last promotion: 2026-09-20 15:14:47.
key = sprintf('%s_%s', lower(char(string(mode))), lower(char(string(coriolis))));
switch key
    case 'nominal_c1'
        gains = struct('KRdiag', [0.1819 0.01 2.596], 'Kxidiag', [85.36 3.784 100], 'LambdaDiag', [38.73 26.23 5.068 0.1638 0.2499 0.1605], 'kd', 24.6, 'ks', 1.651, 'alpha', 0.1439, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'nominal_c2'
        gains = struct('KRdiag', [3.073 0.01241 5.623], 'Kxidiag', [100 28.7 100], 'LambdaDiag', [14.17 5.649 12.24 0.1466 0.1087 0.1126], 'kd', 0.5001, 'ks', 23.46, 'alpha', 0.8573, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'euclidean_c1'
        gains = struct('KRdiag', [4 5 6], 'Kxidiag', [3 3 4], 'LambdaDiag', [2 2 2 2 2 2], 'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 0.001);
    case 'euclidean_c2'
        gains = struct('KRdiag', [0.02081 0.121 2.522], 'Kxidiag', [21.81 12 28.13], 'LambdaDiag', [33.02 99.51 100 1.226 2.753 0.905], 'kd', 85.84, 'ks', 6.393, 'alpha', 0.1081, 'gammaE', [0.008287;3.186e-05;8.089e-05;1e-05;1e-05;1e-05;1e-05;1e-05;0.002694;0.0002115], 'gammaB', 0.001);
    case 'bregman_c1'
        gains = struct('KRdiag', [0.8752 0.1478 3.518], 'Kxidiag', [100 43.03 20.77], 'LambdaDiag', [1.405 4.572 3.305 0.1037 0.04545 0.4213], 'kd', 0.5001, 'ks', 10.52, 'alpha', 0.9, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 1e-05);
    case 'bregman_c2'
        gains = struct('KRdiag', [5.287 5.753 0.003384], 'Kxidiag', [77.78 107.1 86.15], 'LambdaDiag', [11.06 10.27 18.06 0.2897 0.2356 0.2348], 'kd', 25.28, 'ks', 4.512, 'alpha', 0.3227, 'gammaE', [0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001;0.001], 'gammaB', 1e-05);
    otherwise
        error('agc:config:optimized_gains:UnknownScenario', 'Unknown scenario key: %s.', key);
end
end
