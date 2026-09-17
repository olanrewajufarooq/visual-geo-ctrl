function gains = manual_gains(mode, coriolis)
%MANUAL_GAINS Hand-selected gains for one controller mode and factorization.
%
% The entries are explicit per scenario so experiments can intentionally
% diverge later while retaining one stable, readable registry interface.

key = scenarioKey(mode, coriolis);
switch key
    case {'nominal_c1', 'nominal_c2', 'euclidean_c1', 'euclidean_c2', 'bregman_c1', 'bregman_c2'}
        gains = defaultEntry();
    otherwise
        error('agc:config:manual_gains:UnknownScenario', 'Unknown scenario key: %s.', key);
end
end

function gains = defaultEntry()
gains = struct('KRdiag', [4, 5, 6], 'Kxidiag', [3, 3, 4], ...
    'LambdaDiag', 2 * ones(1,6), 'kd', 1, 'ks', 0.5, 'alpha', 0.5, ...
    'gammaE', 1e-3 * ones(10, 1), 'gammaB', 1e-3);
end

function key = scenarioKey(mode, coriolis)
key = sprintf('%s_%s', lower(char(string(mode))), lower(char(string(coriolis))));
end
