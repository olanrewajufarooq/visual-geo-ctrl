classdef TestSE3Utils < matlab.unittest.TestCase
    %TESTSE3UTILS Unit tests for the SE(3) Lie group utility functions.

    methods (Test)
        function testVec2TildeTilde2Vec3Roundtrip(testCase)
            w = [1; 2; 3];
            S = fth.se3.vec2tilde(w);
            testCase.verifyEqual(fth.se3.tilde2vec(S), w, 'AbsTol', 1e-14);
        end

        function testVec2Tilde3SkewSymmetry(testCase)
            w = [0.5; -1.3; 2.7];
            S = fth.se3.vec2tilde(w);
            testCase.verifyEqual(S, -S', 'AbsTol', 1e-14);
        end

        function testVec2Tilde3CrossProduct(testCase)
            a = [1; 2; 3];
            b = [4; 5; 6];
            testCase.verifyEqual(fth.se3.vec2tilde(a) * b, cross(a, b), 'AbsTol', 1e-14);
        end

        function testVec2TildeTilde2Vec6Roundtrip(testCase)
            V = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            se3mat = fth.se3.vec2tilde(V);
            testCase.verifyEqual(fth.se3.tilde2vec(se3mat), V, 'AbsTol', 1e-14);
        end

        function testVec2TildeRejectsInvalidDimension(testCase)
            testCase.verifyError(@() fth.se3.vec2tilde([1; 2]), ...
                                 'fth:vec2tilde:InvalidDimension');
        end

        function testTilde2VecRejectsInvalidDimension(testCase)
            testCase.verifyError(@() fth.se3.tilde2vec(eye(2)), ...
                                 'fth:tilde2vec:InvalidDimension');
        end

        function testExpSE3Identity(testCase)
            % Zero twist should give identity.
            se3mat = zeros(4);
            T = fth.se3.expSE3(se3mat);
            testCase.verifyEqual(T, eye(4), 'AbsTol', 1e-14);
        end

        function testExpSE3PureTranslation(testCase)
            % Pure translation (no rotation).
            v = [1; 2; 3];
            se3mat = [zeros(3), v; 0 0 0 0];
            T = fth.se3.expSE3(se3mat);
            testCase.verifyEqual(T(1:3,1:3), eye(3), 'AbsTol', 1e-14);
            testCase.verifyEqual(T(1:3,4), v, 'AbsTol', 1e-14);
        end

        function testExpSE3PureRotation(testCase)
            % 90-degree rotation about z-axis.
            theta = pi/2;
            se3mat = [0 -theta 0 0; theta 0 0 0; 0 0 0 0; 0 0 0 0];
            T = fth.se3.expSE3(se3mat);
            R_expected = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
            testCase.verifyEqual(T(1:3,1:3), R_expected, 'AbsTol', 1e-12);
        end

        function testLogSE3InverseOfExp(testCase)
            % log(exp(xi)) should return xi for small twists.
            xi = [0.1; -0.2; 0.3; 0.4; -0.5; 0.6];
            se3mat = fth.se3.vec2tilde(xi);
            T = fth.se3.expSE3(se3mat);
            xi_recovered = fth.se3.logSE3(T);
            testCase.verifyEqual(xi_recovered, xi, 'AbsTol', 1e-10);
        end

        function testLogSE3Identity(testCase)
            % log(I) should be zero twist.
            zeta = fth.se3.logSE3(eye(4));
            testCase.verifyEqual(zeta, zeros(6,1), 'AbsTol', 1e-14);
        end

        function testInvSE3(testCase)
            % H * H^{-1} = I.
            R = [0 -1 0; 1 0 0; 0 0 1];
            p = [1; 2; 3];
            H = [R, p; 0 0 0 1];
            Hinv = fth.se3.invSE3(H);
            testCase.verifyEqual(H * Hinv, eye(4), 'AbsTol', 1e-14);
        end

        function testAdAdjointProperty(testCase)
            % Ad(H) * xi = vee(H * hat(xi) * H^{-1}) for any twist xi.
            R = [0 -1 0; 1 0 0; 0 0 1];
            p = [1; 2; 3];
            H = [R, p; 0 0 0 1];
            xi = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            AdH = fth.se3.Ad(H);
            result_Ad = AdH * xi;
            result_conj = fth.se3.tilde2vec(H * fth.se3.vec2tilde(xi) * fth.se3.invSE3(H));
            testCase.verifyEqual(result_Ad, result_conj, 'AbsTol', 1e-12);
        end

        function testAdInvIsInverseOfAd(testCase)
            R = [0 -1 0; 1 0 0; 0 0 1];
            p = [1; 2; 3];
            H = [R, p; 0 0 0 1];
            AdH = fth.se3.Ad(H);
            AdInvH = fth.se3.Ad_inv(H);
            testCase.verifyEqual(AdH * AdInvH, eye(6), 'AbsTol', 1e-12);
        end

        function testAdVLieBracket(testCase)
            % ad_V * W = [V, W] Lie bracket.
            V = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            W = [0.6; 0.5; 0.4; 0.3; 0.2; 0.1];
            adV = fth.se3.adV(V);
            bracket = fth.se3.tilde2vec(fth.se3.vec2tilde(V) * fth.se3.vec2tilde(W) - fth.se3.vec2tilde(W) * fth.se3.vec2tilde(V));
            testCase.verifyEqual(adV * W, bracket, 'AbsTol', 1e-12);
        end

        function testRotInertiaVec2MatUsesCanonicalOffDiagOrder(testCase)
            Iparams = [1; 2; 3; 4; 5; 6];
            J = fth.se3.rotInertiaVec2Mat(Iparams);
            J_expected = [1 4 5; 4 2 6; 5 6 3];
            testCase.verifyEqual(J, J_expected, 'AbsTol', 1e-14);
        end

        function testRotInertiaMat2VecInverse(testCase)
            Iparams = [1; 2; 3; 4; 5; 6];
            J = fth.se3.rotInertiaVec2Mat(Iparams);
            testCase.verifyEqual(fth.se3.rotInertiaMat2Vec(J), Iparams, 'AbsTol', 1e-14);
        end
    end
end
