function variants = expandScenarioSelection(modes, coriolis)
%EXPANDSCENARIOSELECTION Expand mode/form selectors into scenario pairs.
%
% A blank selector ('', "", or {}) selects every supported value. Otherwise,
% pass one char/string or a cell/string array, for example
% {'bregman','euclidean'} and {'c1','c2'}. The result is an N-by-2 cell array
% in [mode, coriolis] order.

availableModes = {'nominal', 'euclidean', 'bregman'};
availableForms = {'c1', 'c2'};
selectedModes = normalizeSelector(modes, availableModes, 'mode');
selectedForms = normalizeSelector(coriolis, availableForms, 'coriolis');

variants = cell(numel(selectedModes) * numel(selectedForms), 2);
index = 0;
for modeIndex = 1:numel(selectedModes)
    for formIndex = 1:numel(selectedForms)
        index = index + 1;
        variants(index,:) = {selectedModes{modeIndex}, selectedForms{formIndex}};
    end
end
end

function selected = normalizeSelector(value, available, label)
if nargin < 1 || isempty(value) || (isstring(value) && all(strlength(value) == 0))
    selected = available;
    return;
end
if ischar(value)
    requested = {lower(value)};
elseif isstring(value)
    requested = cellstr(lower(value(:))).';
elseif iscellstr(value)
    requested = cellfun(@lower, value(:).', 'UniformOutput', false);
else
    error('agc:opt:expandScenarioSelection:SelectorType', ...
        '%s selector must be char, string, cellstr, or blank.', label);
end

selected = cell(1, numel(requested));
for k = 1:numel(requested)
    index = find(strcmp(available, requested{k}), 1);
    if isempty(index)
        error('agc:opt:expandScenarioSelection:UnknownSelector', ...
            'Unsupported %s selector: %s.', label, requested{k});
    end
    selected{k} = available{index};
end
end
