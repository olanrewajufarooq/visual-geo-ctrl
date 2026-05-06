function T = expSE3(se3mat)
%EXPSE3 Exponential map from se(3) to SE(3).
%   T = exp(hat(xi)) where xi = [omega; v] is a twist.
%
%   Uses the Rodrigues formula for rotation and the closed-form
%   translation map:
%     R = I + sin(theta)*[w] + (1-cos(theta))*[w]^2
%     p = (I*theta + (1-cos)*[w] + (theta-sin)*[w]^2) * v / theta
%   where [w] = omgmat/theta, theta = ||omega||.
%
%   Input:  se3mat - 4x4 element of se(3) (hat of a twist).
%   Output: T - 4x4 SE(3) matrix.
    omgmat = se3mat(1:3,1:3);
    v = se3mat(1:3,4);

    theta = sqrt(omgmat(1,2)^2 + omgmat(1,3)^2 + omgmat(2,3)^2);

    if theta == 0
        R = eye(3);
        p = v;
    else
        omgmat_norm = omgmat / theta;
        R = eye(3) + sin(theta) * omgmat_norm + (1 - cos(theta)) * (omgmat_norm * omgmat_norm);
        p = (eye(3)*theta + (1 - cos(theta))*omgmat_norm + (theta - sin(theta))*(omgmat_norm*omgmat_norm)) * v / theta;
    end

    T = [R, p; 0 0 0 1];
end
