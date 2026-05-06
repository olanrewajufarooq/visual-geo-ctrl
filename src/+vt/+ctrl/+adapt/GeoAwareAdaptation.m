classdef GeoAwareAdaptation < vt.ctrl.adapt.EuclideanAdaptation
    %GEOAWAREADAPTATION Placeholder for geometry-aware adaptation.
    %   Currently falls back to Euclidean adaptation.
    %
    %   This class is kept for future extension.
    methods
        function obj = GeoAwareAdaptation(cfg)
            %GEOAWAREADAPTATION Initialize (falls back to Euclidean).
            %   Input:
            %     cfg - configuration struct.
            %   Output:
            %     obj - GeoAwareAdaptation instance.
            obj@vt.ctrl.adapt.EuclideanAdaptation(cfg);
            warning('Geo-aware adaptation not implemented; using euclidean.');
        end
    end
end
