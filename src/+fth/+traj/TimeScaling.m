classdef TimeScaling
    %TIMESCALING Fifth-order polynomial time scaling for smooth trajectories.
    %   Maps t ∈ [0,T] to a normalized path parameter s ∈ [0,1] with
    %   zero velocity and acceleration at both endpoints (C² continuity).
    %
    %   The scaling separates the geometric path definition from time
    %   parameterization, following the path/time-scaling decomposition in
    %   Modern Robotics §9.1.
    %
    %   Usage:
    %     ts = fth.traj.TimeScaling.fifthOrder(T)
    %     [s, sd, sdd] = ts.evaluate(t)
    properties (SetAccess = private)
        % Polynomial coefficients in descending order for polyval.
        % Solution of the 5th-order boundary-condition system:
        %   s(0)=0, ds/dt(0)=0, d²s/dt²(0)=0
        %   s(T)=1, ds/dt(T)=0, d²s/dt²(T)=0
        % Normalized (τ=t/T): a = [0;0;0;10;-15;6], stored descending.
        coeffs  % 1×6 row [6 -15 10 0 0 0]
        T       % total time horizon [s]
    end

    methods (Static)
        function ts = fifthOrder(T)
            %FIFTHORDER Create a fifth-order time scaling for horizon T.
            %   Enforces zero velocity and acceleration at both endpoints.
            %   Input:
            %     T - total time horizon [s].
            %   Output:
            %     ts - TimeScaling instance.
            if T <= 0
                error('fth:TimeScaling:InvalidHorizon', ...
                    'Time horizon T must be positive.');
            end
            ts = fth.traj.TimeScaling();
            ts.T = T;
            % Closed-form solution of M*a = b for the normalized polynomial:
            %   s(τ)  = 10τ³ - 15τ⁴ + 6τ⁵
            %   ds/dτ = 30τ² - 60τ³ + 30τ⁴
            ts.coeffs = [6, -15, 10, 0, 0, 0];
        end
    end

    methods
        function [s, sd, sdd] = evaluate(obj, t)
            %EVALUATE Compute s(t), ds/dt(t), d²s/dt²(t).
            %   τ is clamped to [0,1] so the scaling saturates outside [0,T].
            %   Derivatives follow the chain rule: ds/dt = (ds/dτ)/T.
            %   Inputs:
            %     t - current time [s].
            %   Outputs:
            %     s   - path parameter ∈ [0,1].
            %     sd  - ds/dt [1/s].
            %     sdd - d²s/dt² [1/s²].
            tau = min(max(t / obj.T, 0), 1);
            c1 = polyder(obj.coeffs);
            c2 = polyder(c1);
            s   = polyval(obj.coeffs, tau);
            sd  = polyval(c1, tau) / obj.T;
            sdd = polyval(c2, tau) / (obj.T^2);
        end
    end
end
