
import rospy
import math

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
    	               decel_limit, accel_limit, wheel_radius,
        	           wheel_base, steer_ratio, max_lat_accel,
        	           max_steer_angle):
        # TODO: Implement
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        min_throttle = 0.0
        max_throttle = 0.2

        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)

        tau = 0.5 # 1/(2*pi*tau) = cutoff frequency
        ts = 0.02 #Sampling time

        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband 
        self.decel_limit = decel_limit 
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius 
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio 
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.last_time = rospy.get_time()


    def control(self, desired_lin_vel, desired_ang_vel, current_lin_vel, current_ang_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0.0, 0.0, 0.0

        current_lin_vel = self.vel_lpf.filt(current_lin_vel)

        #rospy.logwarn("Desired angular vel: {0}".format(desired_ang_vel))
        #rospy.logwarn("DBW enabled: {0}".format(dbw_enabled))
        #rospy.logwarn("Desired linear vel: {0}".format(desired_lin_vel))
        #rospy.logwarn("Current linear vel: {0}".format(current_lin_vel))

        steering = self.yaw_controller.get_steering(desired_lin_vel, desired_ang_vel, current_lin_vel)

        vel_error = desired_lin_vel - current_lin_vel
        self.last_vel = current_lin_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if desired_lin_vel == 0.0 and current_lin_vel < 0.1:
        	throttle = 0
        	brake = 700 #hold the car in place
        elif throttle < 0.1 and vel_error < 0:
        	throttle = 0
        	decel = max(vel_error, self.decel_limit)
        	brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        
        #rospy.logwarn("Throttle: {0}".format(throttle))
        #rospy.logwarn("Steering: {0}".format(steering))

        return throttle, brake, steering
