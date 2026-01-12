#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from armybot_msgs.msg import RobotCommand
from armybot_msgs.msg import VehicleControl
from armybot_msgs.msg import CameraAngle
import math


class AckermannTwistController(Node):
    def __init__(self):
        super().__init__('ackermann_twist_controller')
        
        # UGV parameters 
        self.wheelbase = 0.9  # Distance between front and rear axles
        self.track_width = 0.67  # Distance between left and right wheels
        self.wheel_radius = 0.175  # Wheel radius in meters
        self.max_steering_angle = 0.2616  # 15 degrees in radians (from URDF)
        
        # Parameters for bicycle kinematic model
        self.current_steering_angle = 0.0  # Current actual steering angle
        self.target_steering_angle = 0.0   # Target steering angle from kinematics
        self.steering_rate = 2.0  # Rate of steering change (rad/s)
        
        # Timer for continuous steering updates
        self.timer_period = 0.02  # 50 Hz update rate for smoother robot motion
        self.timer = self.create_timer(self.timer_period, self.update_steering)
        
        # Current state of robot
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        # Timeout parameters
        self.cmd_timeout = 2.0  # 2 seconds timeout to reset steering to center
        self.steering_reset_rate = 0.5  # Rate of return (rad/s)
        self.is_resetting_steering = False  # Flag to track if robot steering is resetting
        

        # self.wheel_vel_pub = self.create_publisher(
        #     Float64MultiArray, 
        #     '/forward_velocity_controller/commands', 
        #     10
        # )
        self.wheel_data_pub = self.create_publisher(RobotCommand,'/robot/cmd',10)
        # self.steering_pub = self.create_publisher(
        #     Float64MultiArray,
        #     '/forward_position_controller/commands',
        #     10
        # )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Ackermann Twist Controller started with enhanced steering control')
        
    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x  # Forward velocity (m/s)
        angular_vel = msg.angular.z  # Angular velocity (rad/s)
        
        # Appling Bicycle Kinematic Model for Twist to Ackermann Conversion
        steering_angle = self.twist_to_ackermann_kinematics(linear_vel, angular_vel)
        
        # Storing commands
        self.current_linear_vel = linear_vel
        self.current_angular_vel = angular_vel
        self.target_steering_angle = steering_angle  # Target from kinematic model
        self.last_cmd_time = self.get_clock().now()

        # Resetting auto-reset flag when new commands are received
        if abs(linear_vel) > 0.001 or abs(angular_vel) > 0.001:
            self.is_resetting_steering = False
        
        # Calculating Turning Radius for Debug Logging
        if abs(angular_vel) > 0.001:
            turning_radius = linear_vel / angular_vel
        else:   
            turning_radius = math.inf #Infinite for straight line motion
        self.get_logger().debug(
            f'v={linear_vel:.2f}, ω={angular_vel:.2f} -> R={turning_radius:.2f}, δ={math.degrees(steering_angle):.1f} degrees'
        )
    
    def twist_to_ackermann_kinematics(self, linear_vel, angular_vel):
        """
        This code transforms Twist command to steering angle using bicycle kinematic model.
        
        Inverse Kinematics:
        We have: v (linear velocity), ω (angular velocity)
        We need: δ (steering angle)
        
        Theory:
        1. ω = v/R  (fundamental kinematic relationship)
        2. R = v/ω  (turning radius)
        3. R = L/tan(δ)  (bicycle model geometry)
        4. Therefore: v/ω = L/tan(δ)
        5. tan(δ) = L*ω/v
        6. δ = atan(L*ω/v)
        """

        # Handling special cases
        if abs(linear_vel) < 0.001: 
            # If linear velocity is near zero, then ignore it and maintain current steering angle
            return self.current_steering_angle
        
        if abs(angular_vel) < 0.001:
            # If angular velocity is near zero, then ignore it and keep steering straight
            return 0.0
        
        # Applying bicycle kinematic model: δ = atan(L*ω/v)
        steering_angle = math.atan(self.wheelbase * angular_vel / linear_vel)
        
        # Keeping in the steering limits
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        return steering_angle
    
    def update_steering(self):
        # Continuously publishing steering and wheel commands based on current steering and velocity
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        # Checking for command timeout
        if time_diff > self.cmd_timeout:
            # Timeout exceeded so stopping all the motions and bringing the robot to default state
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.target_steering_angle = 0.0
            self.is_resetting_steering = True
            
            # Returning steering to center position
            if abs(self.current_steering_angle) > 0.001:
                # Calculating direction 
                if self.current_steering_angle > 0:
                    reset_direction = -1.0
                else:
                    reset_direction = 1.0
                reset_change = reset_direction * self.steering_reset_rate * self.timer_period
                
                # Overshoot Prevention
                if abs(self.current_steering_angle) < abs(reset_change):
                    self.current_steering_angle = 0.0  # Directly change to center position
                else:
                    self.current_steering_angle += reset_change
                    
                self.get_logger().debug(f'Command Timeout Exceeded, resetting steering angle: {math.degrees(self.current_steering_angle):.1f} to 0.0')
            else:
                self.is_resetting_steering = False
        else:
            # No timeout 
            self.is_resetting_steering = False
            
            # Smoothly move current steering toward kinematic target
            steering_error = self.target_steering_angle - self.current_steering_angle
            
            if abs(steering_error) > 0.001:
                # Calculating how much wheels can change once a callback
                max_change = self.steering_rate * self.timer_period
                steering_change = max(-max_change, min(max_change, steering_error)) # Overshoot Prevention and Sign Consideration
                self.current_steering_angle += steering_change
                
                # Keeping in the steering limits
                self.current_steering_angle = max(-self.max_steering_angle, 
                                                min(self.max_steering_angle, self.current_steering_angle))
        
        # Calculating wheel velocities based on current linear velocity and steering
        # left_wheel_vel, right_wheel_vel = self.calculate_wheel_velocities(
        #     self.current_linear_vel, self.current_steering_angle
        # )
        
        # Calculating individual steering angles
        # left_steering, right_steering = self.calculate_ackermann_angles(self.current_steering_angle)

        # Publishing motion commands
        # self.publish_steering_commands(left_steering, right_steering)
        # self.publish_wheel_velocities(left_wheel_vel, right_wheel_vel)

        self.publish_wheel_commands()
        
        # Debug logging
        # self.get_logger().debug(
        #     f'{"TIMEOUT" if self.is_resetting_steering else "KINEMATICS"}: '
        #     f'Target: {math.degrees(self.target_steering_angle):.1f}, '
        #     f'Current: {math.degrees(self.current_steering_angle):.1f}, '
        #     f'Wheels: L={left_wheel_vel:.2f}, R={right_wheel_vel:.2f}'
        # )
    
    def publish_wheel_commands(self):
        cmd=RobotCommand()
        wheel=VehicleControl()
        wheel.linear_velocity=self.current_linear_vel*60
        wheel.steering_angle=self.current_steering_angle*540/math.pi
        camera=CameraAngle()
        camera.azimuth=self.current_steering_angle*540/math.pi
        camera.elevation=0.0
        cmd.vehicle_control=wheel
        cmd.camera_angle=camera
        self.wheel_data_pub.publish(cmd)



    def calculate_wheel_velocities(self, linear_vel, steering_angle):
        """
        Calculating wheel velocities using bicycle kinematic model.
        
        Forward Kinematics for wheel speeds:
        We have: v (desired linear velocity), δ (steering angle)
        We need: v_left, v_right (rear wheel velocities)
        
        Theory:
        1. R = L/tan(δ)  (turning radius from steering angle)
        2. ω = v/R = v*tan(δ)/L  (angular velocity)
        3. v_left = ω * (R - track_width/2)  (inside wheel slower)
        4. v_right = ω * (R + track_width/2)  (outside wheel faster)
        """
        
        if abs(linear_vel) < 0.001:
            return 0.0, 0.0
        
        if abs(steering_angle) < 0.001:
            # Moving in straight line with same speed for both wheels
            wheel_vel = linear_vel / self.wheel_radius
            return wheel_vel, wheel_vel
        
        # Calculating turning radius from steering angle
        turning_radius = self.wheelbase / math.tan(abs(steering_angle))
        
        # Calculating angular velocity from linear velocity and turning radius
        angular_vel = linear_vel / turning_radius
        
        # Calculating individual wheel speeds for rear wheels
        if steering_angle > 0:  # Left turn
            # Left wheel which is on inside moves slower than the outer wheel
            left_radius = turning_radius - self.track_width / 2.0
            right_radius = turning_radius + self.track_width / 2.0
        else:  # Right turn
            # Right wheel which is on the inside moves slower than the outer wheel
            left_radius = turning_radius + self.track_width / 2.0
            right_radius = turning_radius - self.track_width / 2.0
        
        # Converting to individual wheel angular velocities 
        left_wheel_vel = (angular_vel * left_radius) / self.wheel_radius
        right_wheel_vel = (angular_vel * right_radius) / self.wheel_radius
        
        return left_wheel_vel, right_wheel_vel
    
    def calculate_ackermann_angles(self, center_steering_angle):
        if abs(center_steering_angle) < 0.001:
            return 0.0, 0.0

        # Calculating turning radius
        turning_radius = self.wheelbase / math.tan(abs(center_steering_angle))
        
        if center_steering_angle > 0:  # Left turn
            # Left wheel which is on the inside has larger angle than the outer wheel
            left_steering = math.atan(self.wheelbase / (turning_radius - self.track_width / 2.0))
            right_steering = math.atan(self.wheelbase / (turning_radius + self.track_width / 2.0))
        else:  # Right turn
            # Right wheel which is on the inside has larger angle than the outer wheel
            left_steering = -math.atan(self.wheelbase / (turning_radius + self.track_width / 2.0))
            right_steering = -math.atan(self.wheelbase / (turning_radius - self.track_width / 2.0))
        
        return left_steering, right_steering
    
    def ackermann_to_twist_kinematics(self, linear_vel, steering_angle):
        """
        Forward Kinematics: To Convert Ackermann command back to Twist (for future).
        We Have: v (linear velocity), δ (steering angle)
        We need: ω (angular velocity)
        
        Theory:
        1. R = L/tan(δ)  (turning radius)
        2. ω = v/R = v*tan(δ)/L  (angular velocity)
        """
        
        if abs(linear_vel) < 0.001 or abs(steering_angle) < 0.001:
            return 0.0  # No motion
        
        # Forward kinematics: ω = v*tan(δ)/L
        angular_vel = linear_vel * math.tan(steering_angle) / self.wheelbase
        
        return angular_vel
    
    def publish_steering_commands(self, left_angle, right_angle):
        steering_msg = Float64MultiArray()
        steering_msg.data = [left_angle, right_angle]
        self.steering_pub.publish(steering_msg)
    
    def publish_wheel_velocities(self, left_vel, right_vel):
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [left_vel, right_vel]
        self.wheel_vel_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannTwistController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()