function rpy = rotm2rpy(R)
%ROTM2RPY Convert rotation matrix to ZYX roll-pitch-yaw.
%   Assumes R = Rz(yaw)*Ry(pitch)*Rx(roll).
%   Warns if near gimbal lock (pitch ≈ ±90°).
%   Input:
%     R - 3x3 rotation matrix.
%   Output:
%     rpy - 3x1 [roll; pitch; yaw] in radians.
    assert(isequal(size(R), [3 3]), 'fth:rotm2rpy: R must be a 3x3 matrix');
    cy = sqrt(R(1,1)^2 + R(2,1)^2);
    if cy < 1e-6
        warning('fth:gimbalLock', ...
            'rotm2rpy: near gimbal lock (pitch ≈ ±90°). Roll/yaw are not uniquely defined.');
        roll  = atan2(-R(2,3), R(2,2));
        pitch = atan2(-R(3,1), 0);  % cy ≈ 0 at lock; explicit 0 is clearer
        yaw   = 0;
    else
        pitch = atan2(-R(3,1), cy);
        roll  = atan2(R(3,2), R(3,3));
        yaw   = atan2(R(2,1), R(1,1));
    end
    rpy = [roll; pitch; yaw];
end
