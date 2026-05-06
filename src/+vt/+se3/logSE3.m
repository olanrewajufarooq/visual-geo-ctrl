function zeta = logSE3(T)
%LOGSE3 Logarithmic map from SE(3) to a 6x1 twist coordinate.
%   zeta = vee(log(T)) = [omega; v], the inverse of expSE3.
%
%   Recovers the rotation angle via theta = acos((tr(R)-1)/2) and
%   computes the inverse left-Jacobian G^{-1} to extract the linear
%   component v = G^{-1} * p.
%
%   Input:  T - 4x4 SE(3) matrix.
%   Output: zeta - 6x1 twist [angular; linear].
    R = T(1:3,1:3);
    p = T(1:3,4);
    acosinput = (trace(R) - 1) / 2;

    if acosinput >= 1
        omgmat = zeros(3);
    elseif acosinput <= -1
        if ~isnan(1 + R(3,3))
            omg = (1 / sqrt(2*(1+R(3,3)))) * [R(1,3); R(2,3); 1+R(3,3)];
        elseif ~isnan(1 + R(2,2))
            omg = (1 / sqrt(2*(1+R(2,2)))) * [R(1,2); 1+R(2,2); R(3,2)];
        else
            omg = (1 / sqrt(2*(1+R(1,1)))) * [1+R(1,1); R(2,1); R(3,1)];
        end
        omgmat = vt.se3.hat3(pi * omg);
    else
        theta = acos(acosinput);
        omgmat = theta / (2 * sin(theta)) * (R - R');
    end

    if norm(omgmat) == 0
        se3mat = [zeros(3), p; 0 0 0 0];
    else
        theta = acos(acosinput);
        omgmat_sq = omgmat * omgmat;
        G_inv = eye(3) - omgmat/2 + (1/theta - cot(theta/2)/2) * (omgmat_sq / theta);
        v = G_inv * p;
        se3mat = [omgmat, v; 0 0 0 0];
    end

    zeta = vt.se3.vee6(se3mat);
end
