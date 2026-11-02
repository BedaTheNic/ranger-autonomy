"""Ideal swerve with wheel centres directly below steering pivots, SI units."""
import math


def inverse_kinematics(vx, vy, wz, wheelbase, track, radius, max_speed,
                       previous_angles=None):
    """Return steering radians and signed wheel rad/s in FL, FR, RL, RR order."""
    if not all(math.isfinite(v) for v in (vx, vy, wz, wheelbase, track, radius, max_speed)):
        raise ValueError('Kinematics inputs must be finite')
    if min(wheelbase, track, radius, max_speed) <= 0:
        raise ValueError('Geometry and speed limits must be positive')
    previous_angles = [0.0] * 4 if previous_angles is None else previous_angles
    angles, speeds = [], []
    for i, (x, y) in enumerate(((wheelbase/2, track/2), (wheelbase/2, -track/2),
                                (-wheelbase/2, track/2), (-wheelbase/2, -track/2))):
        ux, uy = vx - wz*y, vy + wz*x
        speed = math.hypot(ux, uy) / radius
        if speed < 1e-9:
            angle = previous_angles[i]
        else:
            angle = math.atan2(uy, ux)
            if angle > math.pi/2:
                angle -= math.pi
                speed = -speed
            elif angle < -math.pi/2:
                angle += math.pi
                speed = -speed
            # At the lateral boundary choose the nearest equivalent orientation.
            if abs(abs(angle) - math.pi/2) < 1e-9:
                other = -angle
                if abs(other - previous_angles[i]) < abs(angle - previous_angles[i]):
                    angle, speed = other, -speed
        angles.append(angle)
        speeds.append(speed)
    scale = max(1.0, max(abs(s) for s in speeds) / max_speed)
    return angles, [s / scale for s in speeds]
