#!/usr/bin/env python3

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ROS2 module imports
import rclpy # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Node class for Python nodes
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from sensor_msgs.msg import LaserScan # LaserScan (LIDAR range measurements) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from rclpy.duration import Duration # Time duration class
from nav_msgs.msg import Odometry

# Python mudule imports
import math
import queue # FIFO queue
import time # Tracking time

# PID controller class
class PIDController:
    '''
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    '''
    def __init__(self, kP, kI, kD, kS):
        self.kP       = kP # Proportional gain
        self.kI       = kI # Integral gain
        self.kD       = kD # Derivative gain
        self.kS       = kS # Saturation constant (error history buffer size)
        self.err_int  = 0 # Error integral
        self.err_dif  = 0 # Error difference
        self.err_prev = 0 # Previous error
        self.err_hist = queue.Queue(self.kS) # Limited buffer of error history
        self.t_prev   = 0 # Previous time

    def control(self, err, t):
        '''
        Generate PID controller output.
        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp
        :return u: PID controller output
        '''
        dt = t - self.t_prev # Timestep
        if dt > 0.0:
            self.err_hist.put(err) # Update error history
            self.err_int += err # Integrate error
            if self.err_hist.full(): # Jacketing logic to prevent integral windup
                self.err_int -= self.err_hist.get() # Rolling FIFO buffer
            self.err_dif = (err - self.err_prev) # Error difference
            u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt) # PID control law
            self.err_prev = err # Update previos error term
            self.t_prev = t # Update timestamp
            return u # Control signal

# Node class
class RobotController(Node):

    #######################
    '''Class constructor'''
    #######################

    def __init__(self):
        # Information and debugging
        info = '\nMake the robot avoid obstacles by maintaining a safe distance from them.\n'
        print(info)
        # ROS2 infrastructure
        super().__init__('robot_controller') # Create a node with name 'robot_controller'
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=10 # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        
        self.robot_scan_sub = self.create_subscription(LaserScan, '/scan', self.robot_laserscan_callback, qos_profile) # Subscriber which will subscribe to LaserScan message on the topic '/scan' adhering to 'qos_profile' QoS profile
        self.robot_scan_sub # Prevent unused variable warning
        
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile) # Publisher which will publish Twist message to the topic '/cmd_vel' adhering to 'qos_profile' QoS profile
        
        timer_period = 0.1 # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
        
        self.laserscan = [] # Initialize variable to capture the laserscan
        self.ctrl_msg = Twist() # Robot control commands (twist)
        self.start_time = self.get_clock().now() # Record current time in seconds
        print(self.start_time)
        
        #0.2, 0.01, 0.2, 10
        self.pid_lat = PIDController(0.2, 0.01, 0.2, 10) # Lateral PID controller object initialized with kP, kI, kD, kS
        self.pid_lon = PIDController(0.1, 0.001, 0.05, 10) # Longitudinal PID controller object initialized with kP, kI, kD, kS

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.odom_received = False

        self.target_x = float(input("Enter the goal x position: ")) # Posición X deseada
        self.target_y = float(input("Enter the goal y position: ")) # Posición Y deseada
    ########################
    '''Callback functions'''
    ########################

    def robot_laserscan_callback(self, msg):
        self.laserscan = msg.ranges # Capture most recent laserscan
        #self.get_logger().info("Scan data received")
        

    def robot_controller_callback(self):
        
        DELAY = 4.0 # Time delay (s)
        self.get_logger().info("Robot")
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
        
            if not self.odom_received:
                LIN_VEL = 0.0
                self.get_logger().info('Waiting for Odom data...')
                return
            if not self.laserscan:
                LIN_VEL = 0.0
                self.get_logger().info('Waiting for scan data...')
                return
            
            self.leng = len(self.laserscan)
            
            left_scan_min = min(self.laserscan[0:self.leng//12]) # Minimum of laserscan from left sector
            right_scan_min = min(self.laserscan[(self.leng*11)//12:self.leng]) # Minimum of laserscan from right sector
            lat_left_scan = min(self.laserscan[(self.leng//12)+1:self.leng//6]) # Minimum of laserscan from left sector
            lat_right_scan = min(self.laserscan[(self.leng*5)//6:((self.leng*11)//12)-1]) # Minimum of laserscan from right sector
            tstamp = time.time() # Current timestamp (s)
            
            if right_scan_min < 0.5 or left_scan_min < 0.5 or lat_right_scan < 0.5 or lat_left_scan < 0.5:
                
                if right_scan_min - left_scan_min > 0.5 and left_scan_min < 0.5:
                    
                    ANG_VEL = -self.pid_lat.control(left_scan_min, tstamp) # Angular velocity (rad/s)
                    print('GIRO A LA DERECHA')
                    if left_scan_min >= 0.5:
                        LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp) # Linear velocity (m/s) from PID controller
                        time.sleep(0.5)
                    else:
                        LIN_VEL = 0.0 # Linear velocity (m/s)
                elif left_scan_min - right_scan_min > 0.5 and right_scan_min < 0.5:
                    
                    ANG_VEL = self.pid_lat.control(right_scan_min, tstamp) # Angular velocity (rad/s)
                    print('GIRO A LA IZQUIERDA')
                    if right_scan_min >= 0.5:
                        LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp) # Linear velocity (m/s) from PID controller
                        time.sleep(0.5)
                    else:
                        LIN_VEL = 0.0 # Linear velocity (m/s)
                else:
                    if left_scan_min <= 0.5 or right_scan_min <= 0.5:
                    
                        LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp) # Linear velocity (m/s) from PID controller
                        print(LIN_VEL) # Linear velocity (m/s)
                        if right_scan_min <= left_scan_min:
                            ANG_VEL = self.pid_lat.control(right_scan_min, tstamp) # Angular velocity (rad/s)
                            print(ANG_VEL)
                            self.get_logger().info("Robot22")
                        else:
                            ANG_VEL = -self.pid_lat.control(left_scan_min, tstamp) # Angular velocity (rad/s)
                            

                    else:
                        LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp) # Linear velocity (m/s) from PID controller
                        ANG_VEL = 0.0 # Angular velocity (rad/s)    
                        print('AVANZO')   
            else:
                self.get_logger().info("Robot33")
                distance_to_goal = math.sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)
                #self.get_logger().info('La distancia a la meta es {}'.format(distance_to_goal))
                angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
                angle_error = angle_to_goal - self.theta
                
                if distance_to_goal < 0.1:
                    LIN_VEL = 0.0
                    ANG_VEL = 0.0
                    self.ctrl_msg.linear.x = LIN_VEL # Set linear velocity
                    self.ctrl_msg.angular.z = ANG_VEL # Set angular velocity
                    self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                    print('OBJETIVO ALCANZADO')
                    elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                    error_x = self.target_x - self.x
                    error_y = self.target_y - self.y
                    self.get_logger().info(f'Tiempo de simulación: {elapsed_time:.2f}s, Error en X: {error_x:.2f}, Error en Y: {error_y:.2f}')
                elif abs(angle_error) > 0.1:
                    print('CORRIGIENDO TRAYECTORIA')
                    ANG_VEL = 0.1 * angle_error / abs(angle_error)
                    LIN_VEL = 0.0
                else:
                    LIN_VEL = self.pid_lon.control(min(3.5, self.laserscan[0]), tstamp) # Linear velocity (m/s) from PID controller
                    ANG_VEL = 0.0 # Angular velocity (rad/s)
                    print('AVANZO')
            self.ctrl_msg.linear.x = LIN_VEL # Set linear velocity
            self.ctrl_msg.angular.z = ANG_VEL # Set angular velocity
            self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
            #print('Distance to closest obstacle is {} m'.format(round(min(left_scan_min, right_scan_min), 4)))
            #print('Robot moving with {} m/s and {} rad/s'.format(LIN_VEL, ANG_VEL))
        else:
            self.get_logger().info('Initializing...')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        _, _, self.theta = self.euler_from_quaternion(orientation)
        #self.get_logger().info('Odom data received: x: {:.2f}, y: {:.2f}, theta: {:.2f}'.format(self.x, self.y, self.theta))
        self.odom_received = True

    def euler_from_quaternion(self, q):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args) # Start ROS2 communications
    node = RobotController() # Create node
    rclpy.spin(node) # Execute node
    node.destroy_node() # Destroy node explicitly (optional - otherwise it will be done automatically when garbage collector destroys the node object)
    rclpy.shutdown() # Shutdown ROS2 communications

if __name__ == "__main__":
    main()
    