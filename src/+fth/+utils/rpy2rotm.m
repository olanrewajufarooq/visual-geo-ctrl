function R = rpy2rotm(rpy)
%RPY2ROTM Convert roll-pitch-yaw angles to rotation matrix.
%   R = rpy2rotm(rpy) converts [roll; pitch; yaw] to a rotation matrix
%   using ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).

    rpy = rpy(:);
    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);

    cr = cos(roll);
    sr = sin(roll);
    cp = cos(pitch);
    sp = sin(pitch);
    cy = cos(yaw);
    sy = sin(yaw);

    R = [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr; ...
         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr; ...
         -sp,     cp * sr,            cp * cr];
end
