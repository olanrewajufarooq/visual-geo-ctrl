classdef RBDynamics
    %RBDYNAMICS Spatial rigid-body dynamics utilities.
    %   Static methods for spatial algebra operators and the linear
    %   rigid-body dynamics regressor.
    %
    %   Parameter vector convention (10×1):
    %     pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    %   where h = m * CoG is the first moment of mass.
    methods (Static)
        function A = paramangmomentum(w)
            %PARAMANGMOMENTUM Angular momentum action matrix.
            %   Returns the 3×6 matrix A such that A * pi(5:10) = J * w,
            %   where J is the inertia tensor.
            %   Input:
            %     w - 3×1 angular velocity.
            %   Output:
            %     A - 3×6 action matrix.
            A = [w(1),    0,    0, w(2), w(3),    0;
                    0, w(2),    0, w(1),    0, w(3);
                    0,    0, w(3),    0, w(1), w(2)];
        end

        function X = paramgenmomentum(V)
            %PARAMGENMOMENTUM Generalized momentum action matrix.
            %   Returns the 6×10 matrix X such that X(V) * pi = I6 * V,
            %   where I6 is the 6×6 generalized inertia and pi is the
            %   10×1 parameter vector [m; h; Jparams].
            %   Input:
            %     V - 6×1 spatial velocity [angular; linear].
            %   Output:
            %     X - 6×10 action matrix.
            X = zeros(6, 10, 'like', V);
            w = V(1:3);
            v = V(4:6);
            % Column 1 (m): translational momentum = m*v
            X(4:6, 1) = v;
            % Columns 2-4 (h = m*CoG): rotational and translational coupling
            X(1:3, 2:4) = -fth.se3.hat3(v);
            X(4:6, 2:4) =  fth.se3.hat3(w);
            % Columns 5-10 (J params): rotational inertia
            X(1:3, 5:10) = fth.utils.RBDynamics.paramangmomentum(w);
        end

        function Y = regressor_rb_dynamics(V, Vd)
            %REGRESSOR_RB_DYNAMICS Rigid-body dynamics regressor (6×10).
            %   Satisfies Y(V, Vd) * pi = I6 * Vd + ad_V^T * I6 * V.
            %   Inputs:
            %     V  - 6×1 spatial velocity.
            %     Vd - 6×1 spatial acceleration (or reference quantity).
            %   Output:
            %     Y - 6×10 regressor matrix.
            Y = fth.utils.RBDynamics.paramgenmomentum(Vd) ...
                - fth.utils.RBDynamics.adjoint(V)' * fth.utils.RBDynamics.paramgenmomentum(V);
        end

        function ad = adjoint(V)
            %ADJOINT 6×6 spatial adjoint (small ad) of a twist.
            %   ad(V) satisfies  ad(V) * W = [V, W]  (Lie bracket).
            %   Input:
            %     V - 6×1 spatial velocity [angular; linear].
            %   Output:
            %     ad - 6×6 matrix.
            w = V(1:3);
            v = V(4:6);
            ad = [fth.se3.hat3(w), zeros(3, 3);
                  fth.se3.hat3(v), fth.se3.hat3(w)];
        end
    end
end
