classdef RigidBodyRegressor < handle
    %RIGIDBODYREGRESSOR Self-contained regressor for the rigid-body model.
    %
    %   Y = rbr.getRegressor(H, V, VR, VRDot)
    %
    %   Parameter vector:
    %     pi = [m; h_x; h_y; h_z; J_xx; J_yy; J_zz; J_xy; J_xz; J_yz]
    %
    %   Convention: V = [omega; v]  (angular then translational velocity).
    %   Wrench rows: 1:3 = torques, 4:6 = forces.
    %
    %   Satisfies: Y*pi = I6*VRDot + C(V,I6)*VR + Wg
    %
    %   coriolisForm:
    %     'basic'      -> C(V, I) = -ad(V)' * I
    %     'consistent' -> C(V, I) = 1/2*(I*ad(V) - ad(I*V) - ad(V)'*I)

    properties (SetAccess = private)
        GravityWorld (3,1) double
        CoriolisForm (1,:) char
    end

    methods
        function obj = RigidBodyRegressor(g, coriolisForm)
            if nargin < 1 || isempty(g);            g = 9.81;    end
            if nargin < 2 || isempty(coriolisForm); coriolisForm = 'basic'; end
            obj.GravityWorld = obj.normalizeGravity(g);
            obj.CoriolisForm = obj.validateCoriolisForm(coriolisForm);
        end

        function Y = getRegressor(obj, H, V, VR, VRDot)
            %GETREGRESSOR Return the 6x10 regressor matrix.
            obj.validateInputs(H, V, VR, VRDot);
            Y = obj.inertiaRegressor(VRDot) + ...
                obj.coriolisRegressor(V, VR) + ...
                obj.gravityRegressor(H);
        end
    end

    methods (Access = private)
        function form = validateCoriolisForm(~, coriolisForm)
            form = lower(strtrim(char(coriolisForm)));
            if ~strcmp(form, 'basic') && ~strcmp(form, 'consistent')
                error('RigidBodyRegressor:InvalidCoriolisForm', ...
                    "coriolisForm must be 'basic' or 'consistent'.");
            end
        end

        function g = normalizeGravity(~, gIn)
            gIn = gIn(:);
            if isscalar(gIn)
                g = [0; 0; gIn];
            elseif numel(gIn) == 3
                g = gIn;
            else
                error('RigidBodyRegressor:InvalidGravity', ...
                    'g must be a scalar or a 3-vector.');
            end
        end

        function validateInputs(obj, H, V, VR, VRDot)
            if ~isnumeric(H) || ~ismatrix(H) || ~all(size(H) == [4 4])
                error('RigidBodyRegressor:InvalidH', ...
                    'H must be a 4x4 homogeneous transform.');
            end
            obj.validateSixVector(V,     'V');
            obj.validateSixVector(VR,    'VR');
            obj.validateSixVector(VRDot, 'VRDot');
        end

        function validateSixVector(~, x, name)
            if ~isnumeric(x(:)) || numel(x) ~= 6
                error('RigidBodyRegressor:InvalidTwist', ...
                    '%s must be a 6-vector.', name);
            end
        end

        function YI = inertiaRegressor(~, k)
            %INERTIAREGRESSOR Regressor for I6*k.
            % V = [omega; v]:  omega=k(1:3) (angular), v=k(4:6) (translational).
            % Row 1:3 = torque equations, row 4:6 = force equations.
            k     = k(:);
            omega = k(1:3);
            v     = k(4:6);

            YI = zeros(6, 10);
            YI(1:3, 2:4)  = -fth.se3.hat3(v);
            YI(1:3, 5:10) = fth.ctrl.RigidBodyRegressor.rotInertiaReg_(omega);
            YI(4:6, 1)    = v;
            YI(4:6, 2:4)  = fth.se3.hat3(omega);
        end

        function YC = coriolisRegressor(obj, V, VR)
            %CORIOLISREGRESSOR Return Yc such that C(V,I)*VR = Yc*pi.
            switch obj.CoriolisForm
                case 'basic'
                    YC = -fth.se3.adV(V).' * obj.inertiaRegressor(VR);

                case 'consistent'
                    YC = 0.5 * ( ...
                        obj.inertiaRegressor(fth.se3.adV(V) * VR) + ...
                        obj.zetaRegressor(V, VR) - ...
                        fth.se3.adV(V).' * obj.inertiaRegressor(VR) );
            end
        end

        function Z = zetaRegressor(~, V, VR)
            %ZETAREGRESSOR Closed-form Z(V,VR) for the consistent Coriolis.
            % V = [omega; v] convention; rows 1:3 = torque, rows 4:6 = force.
            omega   = V(1:3);    v  = V(4:6);
            omega_r = VR(1:3);   vr = VR(4:6);

            rIR = fth.ctrl.RigidBodyRegressor.rotInertiaReg_(omega);

            Z = zeros(6, 10);
            % Force rows (4:6):
            Z(4:6, 1)    = -fth.se3.hat3(vr) * v;
            Z(4:6, 2:4)  =  fth.se3.hat3(omega_r) * fth.se3.hat3(v) ...
                           - fth.se3.hat3(vr)      * fth.se3.hat3(omega);
            Z(4:6, 5:10) = -fth.se3.hat3(omega_r) * rIR;
            % Torque rows (1:3):
            Z(1:3, 1)    = -fth.se3.hat3(omega_r) * v;
            Z(1:3, 2:4)  = -fth.se3.hat3(omega_r) * fth.se3.hat3(omega);
        end

        function YG = gravityRegressor(obj, H)
            %GRAVITYREGRESSOR Return the gravity regressor Yg(H).
            % Rows 1:3 = torque, rows 4:6 = force (V=[omega;v] convention).
            R  = H(1:3, 1:3);
            gB = R.' * obj.GravityWorld;
            YG = zeros(6, 10);
            YG(4:6, 1)   = gB;
            YG(1:3, 2:4) = -fth.se3.hat3(gB);
        end
    end

    methods (Static, Access = private)
        function YJ = rotInertiaReg_(omega)
            %ROTINERTIAREG_ 3x6 block for [J_xx J_yy J_zz J_xy J_xz J_yz]'.
            omega = omega(:);
            YJ = [omega(1),  0,         0,        omega(2), omega(3),  0;        ...
                  0,         omega(2),  0,        omega(1),  0,        omega(3); ...
                  0,         0,         omega(3),  0,        omega(1), omega(2)];
        end
    end
end
