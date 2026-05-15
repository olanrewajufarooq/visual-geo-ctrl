classdef AdaptationUtils
    %ADAPTATIONUTILS Rigid-body inertia parameterization utilities.
    %   Static methods for converting between the 10×1 parameter vector pi,
    %   the 4×4 pseudo-inertia matrix J (symmetric positive definite), and
    %   the 6×6 generalized inertia matrix I6.
    %
    %   Parameter vector convention:
    %     pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    %   where h = m * CoG is the first moment of mass and the inertia
    %   parameters follow the rotInertiaMat2Vec ordering.
    %
    %   Pseudo-inertia matrix structure:
    %     J = [S,   h  ]   where S = rotInertia2CoRotInertia(I_tensor)
    %         [h^T, m  ]
    methods (Static)
        function psi = spd2params(P)
            %SPD2PARAMS Convert 4×4 pseudo-inertia matrix to parameter vector.
            %   pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
            %   Input:
            %     P - 4×4 symmetric positive definite pseudo-inertia matrix.
            %   Output:
            %     psi - 10×1 parameter vector.
            m = P(4, 4);
            h = P(1:3, 4);
            J = fth.ctrl.adapt.AdaptationUtils.coRotInertia2RotInertia(P(1:3, 1:3));
            psi = [m; h; fth.se3.rotInertiaMat2Vec(J)];
        end

        function P = params2spd(psi)
            %PARAMS2SPD Convert parameter vector to 4×4 pseudo-inertia matrix.
            %   Input:
            %     psi - 10×1 parameter vector [m; h; Jparams].
            %   Output:
            %     P - 4×4 symmetric positive definite pseudo-inertia matrix.
            m = psi(1);
            h = psi(2:4);
            J = fth.se3.rotInertiaVec2Mat(psi(5:10));
            P = zeros(4, 4, 'like', psi);
            P(1:3, 1:3) = fth.ctrl.adapt.AdaptationUtils.rotInertia2CoRotInertia(J);
            P(1:3, 4) = h;
            P(4, 1:3) = h';
            P(4, 4) = m;
        end

        function I = params2genInertia(psi)
            %PARAMS2GENINERTIA Convert parameter vector to 6×6 generalized inertia.
            %   I6 = [J,        hat(h) ]
            %        [-hat(h),  m*I_3  ]
            %   Input:
            %     psi - 10×1 parameter vector [m; h; Jparams].
            %   Output:
            %     I - 6×6 generalized inertia matrix.
            m = psi(1);
            h = psi(2:4);
            J = fth.se3.rotInertiaVec2Mat(psi(5:10));
            I = zeros(6, 6, 'like', psi);
            I(1:3, 1:3) = J;
            I(1:3, 4:6) =  fth.se3.vec2tilde(h);
            I(4:6, 1:3) = -fth.se3.vec2tilde(h);
            I(4:6, 4:6) = m * eye(3, 'like', psi);
        end

        function psi = genInertia2params(I6)
            %GENINERTIA2PARAMS Convert 6×6 generalized inertia to parameter vector.
            %   Input:
            %     I6 - 6×6 generalized inertia matrix.
            %   Output:
            %     psi - 10×1 parameter vector [m; h; Jparams].
            m = I6(4, 4);
            h_hat = I6(1:3, 4:6);
            h = [h_hat(3,2); h_hat(1,3); h_hat(2,1)];
            J = I6(1:3, 1:3);
            psi = [m; h; fth.se3.rotInertiaMat2Vec(J)];
        end

        function N = spd2params_jacobian()
            %SPD2PARAMS_JACOBIAN Jacobian of spd2params w.r.t. vech(P).
            %   Maps the 10 independent elements of P to the parameter vector pi.
            %   The independent elements are ordered as pseudoInertiaMat2Vec(P):
            %     [P11, P22, P33, P44, P12, P13, P23, P14, P24, P34]
            %   Output:
            %     N - 10×10 constant matrix (persistent, computed once).
            persistent N_cache
            if isempty(N_cache)
                N_cache = [ ...
                    0  0  0  1  0  0  0  0  0  0;   % m    = P44
                    0  0  0  0  0  0  0  1  0  0;   % hx   = P14
                    0  0  0  0  0  0  0  0  1  0;   % hy   = P24
                    0  0  0  0  0  0  0  0  0  1;   % hz   = P34
                    0  1  1  0  0  0  0  0  0  0;   % Ixx  = P22 + P33
                    1  0  1  0  0  0  0  0  0  0;   % Iyy  = P11 + P33
                    1  1  0  0  0  0  0  0  0  0;   % Izz  = P11 + P22
                    0  0  0  0 -1  0  0  0  0  0;   % Ixy  = -P12
                    0  0  0  0  0 -1  0  0  0  0;   % Ixz  = -P13
                    0  0  0  0  0  0 -1  0  0  0];  % Iyz  = -P23
            end
            N = N_cache;
        end

        function E = eliminationspd4()
            %ELIMINATIONSPD4 Elimination matrix for 4×4 symmetric matrices.
            %   Converts vec(P) [16×1, column-major] to vech(P) [10×1]:
            %     vech(P) = [P11, P22, P33, P44, P12, P13, P23, P14, P24, P34]
            %   Output:
            %     E - 10×16 constant matrix (persistent, computed once).
            persistent E_cache
            if isempty(E_cache)
                E_cache = [ ...
                    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;   % P11  = vec( 1)
                    0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;   % P22  = vec( 6)
                    0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;   % P33  = vec(11)
                    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;   % P44  = vec(16)
                    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;   % P12  = vec( 2)
                    0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;   % P13  = vec( 3)
                    0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;   % P23  = vec( 7)
                    0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;   % P14  = vec(13)
                    0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;   % P24  = vec(14)
                    0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0];  % P34  = vec(15)
            end
            E = E_cache;
        end

        function P_vec = pseudoInertiaMat2Vec(P)
            %PSEUDOINERTIAMAT2VEC Vectorize upper triangle of 4×4 symmetric matrix to 10×1.
            %   Output ordering: [P11, P22, P33, P44, P12, P13, P23, P14, P24, P34]
            P_vec = [P(1,1); P(2,2); P(3,3); P(4,4); ...
                     P(1,2); P(1,3); P(2,3); P(1,4); P(2,4); P(3,4)];
        end

        function P = pseudoInertiaVec2Mat(P_vec)
            %PSEUDOINERTIAVEC2MAT Reconstruct 4×4 symmetric matrix from 10×1 vector.
            %   Input ordering: [P11, P22, P33, P44, P12, P13, P23, P14, P24, P34]
            P = [P_vec(1), P_vec(5), P_vec(6), P_vec(8);
                 P_vec(5), P_vec(2), P_vec(7), P_vec(9);
                 P_vec(6), P_vec(7), P_vec(3), P_vec(10);
                 P_vec(8), P_vec(9), P_vec(10), P_vec(4)];
        end

        function J = coRotInertia2RotInertia(S)
            %COROTINERTIA2ROTINERTIA Convert co-rotational inertia matrix to rotational inertia tensor.
            %   J = trace(S)*I - S
            J = trace(S) * eye(3) - S;
        end

        function S = rotInertia2CoRotInertia(J)
            %ROTINERTIA2COROTINERTIA Convert rotational inertia tensor to co-rotational inertia matrix.
            %   S = 0.5*trace(J)*I - J
            S = 0.5 * trace(J) * eye(3) - J;
        end

        function isSPD = is_spd(P)
            %IS_SPD Check if a matrix is symmetric positive definite.
            tol = 1e-12;
            if norm(P - P', 'fro') >= tol * norm(P, 'fro')
                isSPD = false;
                return;
            end
            [~, flag] = chol(P);
            isSPD = (flag == 0);
        end

        function isValid = is_valid_params(psi)
            %IS_VALID_PARAMS Check if parameter vector corresponds to a valid (SPD) pseudo-inertia.
            P = fth.ctrl.adapt.AdaptationUtils.params2spd(psi);
            isValid = fth.ctrl.adapt.AdaptationUtils.is_spd(P);
        end
    end
end
