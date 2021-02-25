from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

KP = 0.3
KI = 0.1
KD = 0.

mn = 0.  # min throlltle
mx = 0.2 # max throttle

tau = 0.5
ts = 0.02

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
        wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(KP, KI, KD, mn, mx)
        self.vel_lpf = LowPassFilter(tau, ts)

        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband

        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, twist_linear, twist_angular, dbw_enabled):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(twist_linear, twist_angular, current_vel)

        vel_error = twist_linear - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        brake = 0

        if twist_linear == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700

        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

#         rospy.logwarn("Control result {0} {1} {2}".format(throttle, brake, steering))

        return throttle, brake, steering
