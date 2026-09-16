function scenarios = applyTransverseGains(x, scenarios)
%APPLYTRANSVERSEGAINS Decode [kd ks alpha] into immutable scenario copies.
x = x(:).';
validateattributes(x, {'numeric'}, {'real', 'finite', 'numel', 3});
for k = 1:numel(scenarios)
    scenarios{k}.controller.kd = x(1);
    scenarios{k}.controller.ks = x(2);
    scenarios{k}.controller.alpha = x(3);
end
end
