function indices = gainBlockIndices(mode, block)
%GAINBLOCKINDICES Return optimizer-coordinate indices for one gain block.
%
% The first fifteen coordinates are always the non-adaptive controller
% gains. Adaptive coordinates follow and depend on the estimator mode.

mode = lower(char(string(mode)));
block = lower(char(string(block)));
switch mode
    case 'nominal'
        total = 15;
    case 'euclidean'
        total = 25;
    case 'bregman'
        total = 16;
    otherwise
        error('agc:opt:gainBlockIndices:Mode', 'Unknown controller mode: %s.', mode);
end

switch block
    case 'all'
        indices = 1:total;
    case 'nonadaptive'
        indices = 1:15;
    case 'adaptive'
        indices = 16:total;
    otherwise
        error('agc:opt:gainBlockIndices:Block', 'Unknown gain block: %s.', block);
end
end
