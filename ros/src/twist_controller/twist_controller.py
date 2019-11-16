from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel,
                 max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        self.min_speed = min_speed
        self.vel_controller = PID(0.5, 0.01, 0.05, 0.0, 1.0)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, linear_velocity, angular_velocity, current_velocity, sample_time):
        if linear_velocity == 0 and current_velocity < self.min_speed:
            throttle = 0
            brake = 700
        else:
            vel_delta = linear_velocity - current_velocity
            if vel_delta > 0:
                throttle = self.vel_controller.step(vel_delta, sample_time)
                brake = 0
            else:
                throttle = 0
                brake = abs(max(vel_delta, self.decel_limit)) * self.vehicle_mass * self.wheel_radius

        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # Return throttle, brake, steer
        return throttle, brake, steer

    def reset(self):
        self.vel_controller.reset()
