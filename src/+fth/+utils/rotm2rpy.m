function rpy = rotm2rpy(R)
%ROTM2RPY Convert rotation matrix to ZYX roll-pitch-yaw
    % Assumes R = Rz(yaw)*Ry(pitch)*Rx(roll)
    pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
    roll = atan2(R(3,2), R(3,3));
    yaw = atan2(R(2,1), R(1,1));
    rpy = [roll; pitch; yaw];
end
