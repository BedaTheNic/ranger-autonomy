"""Planar encoder odometry for ideal swerve wheels below their steering pivots."""
import math
import numpy as np


def angle_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def body_increment(vx, vy, wz, dt):
    angle = wz*dt
    if abs(angle) < 1e-8:
        return vx*dt, vy*dt, angle
    s, c = math.sin(angle)/wz, (1-math.cos(angle))/wz
    return s*vx-c*vy, c*vx+s*vy, angle


class EncoderOdometry:
    def __init__(self, wheelbase, track, radius, max_dt=0.1, velocity_std=0.02):
        if not all(math.isfinite(v) and v > 0
                   for v in (wheelbase, track, radius, max_dt, velocity_std)):
            raise ValueError('Geometry, sample gap and noise must be finite and positive')
        self.radius, self.max_dt, self.velocity_std = radius, max_dt, velocity_std
        rows = []
        for x, y in ((wheelbase/2, track/2), (wheelbase/2, -track/2),
                     (-wheelbase/2, track/2), (-wheelbase/2, -track/2)):
            rows.extend(((1, 0, -y), (0, 1, x)))
        self.matrix = np.array(rows, dtype=float)
        self.inverse = np.linalg.pinv(self.matrix)
        self.reset()

    def reset(self):
        self.previous = None
        self.pose = np.zeros(3)
        self.covariance = np.eye(3)*1e-6
        self.distance = 0.0
        self.last_event = 'initializing'

    def update(self, stamp, rotations, steering):
        """Integrate encoder deltas; no commands, IMU or ground truth enter here.

        Encoder deltas are unwrapped assuming < pi rad between valid samples.
        At the model limit (10 rad/s), max_dt=0.1 satisfies this constraint.
        """
        if (len(rotations) != 4 or len(steering) != 4 or
                not all(math.isfinite(v) for v in [stamp, *rotations, *steering])):
            self.last_event = 'invalid'
            return None
        sample = (stamp, list(rotations), list(steering))
        if self.previous is None:
            self.previous = sample
            return None
        previous_stamp, previous_rotations, previous_steering = self.previous
        dt = stamp-previous_stamp
        if dt == 0:
            self.last_event = 'duplicate'
            return None
        if dt < 0:
            self.reset()
            self.previous = sample
            self.last_event = 'time_reset'
            return None
        self.previous = sample
        if dt > self.max_dt:
            self.covariance += np.diag([0.01, 0.01, 0.04])
            self.last_event = 'gap'
            return None  # unknown motion over missing data must not be invented
        wheel_vectors = []
        for now, before, steer, old_steer in zip(
                rotations, previous_rotations, steering, previous_steering):
            rolling_speed = self.radius*angle_difference(now, before)/dt
            midpoint = old_steer + 0.5*angle_difference(steer, old_steer)
            wheel_vectors.extend((rolling_speed*math.cos(midpoint),
                                  rolling_speed*math.sin(midpoint)))
        measured = np.array(wheel_vectors)
        twist = self.inverse @ measured
        residual = float(np.sqrt(np.mean((self.matrix @ twist-measured)**2)))
        # Nonzero heuristic noise floor, inflated by disagreement between wheels.
        twist_cov = (self.velocity_std**2+residual**2)*(self.inverse @ self.inverse.T)
        dx, dy, dtheta = body_increment(*twist, dt)
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        world_dx, world_dy = c*dx-s*dy, s*dx+c*dy
        f = np.array([[1, 0, -world_dy], [0, 1, world_dx], [0, 0, 1]])
        b = dt*np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        self.covariance = (f @ self.covariance @ f.T + b @ twist_cov @ b.T +
                           np.diag([1e-5, 1e-5, 1e-4])*dt)
        self.pose += (world_dx, world_dy, dtheta)
        self.distance += math.hypot(twist[0], twist[1])*dt
        self.last_event = 'integrated'
        return twist, twist_cov, residual
