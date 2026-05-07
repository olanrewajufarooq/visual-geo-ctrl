classdef GeoAwareAdaptation < fth.ctrl.adapt.EuclideanAdaptation
    %GEOAWAREADAPTATION Stub for geometry-aware adaptation (not yet implemented).
    %   Errors on construction until the geo-aware law is ready.
    %   Remove the error and add implementation here when implementing.
    methods
        function obj = GeoAwareAdaptation(cfg)
            %GEOAWAREADAPTATION Stub constructor — errors until implemented.
            %   Input:
            %     cfg - configuration struct.
            obj@fth.ctrl.adapt.EuclideanAdaptation(cfg);
            error('fth:GeoAwareAdaptation:NotImplemented', ...
                'Geo-aware adaptation is not yet implemented. Use ''euclidean'' or ''none''.');
        end
    end
end
