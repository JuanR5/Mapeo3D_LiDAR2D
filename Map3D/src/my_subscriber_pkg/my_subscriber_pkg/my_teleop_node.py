#!/usr/bin/env python

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import sys
import tty
import termios

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   U    I    O
   H    J    K
   B    N    M

anything else : stop

q/a : increase/decrease max speeds by 10%
w/s : increase/decrease only linear speed by 10%
e/d : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'I':(1,0,0,0),
        'O':(0,0,0,-1),
        'H':(0,1,0,0),
        'K':(0,-1,0,0),
        'U':(0,0,0,1),
        'N':(-1,0,0,0),
        'M':(-1,0,0,-1),
        'B':(-1,0,0,1),
        'i':(1,0,0,0),
        'o':(0,0,0,-1),
        'h':(0,1,0,0),
        'k':(0,-1,0,0),
        'u':(0,0,0,1),
        'n':(-1,0,0,0),
        'm':(-1,0,0,-1),
        'b':(-1,0,0,1),
    }

speedBindings={
        'q':(1.1,1.1),
        'a':(.9,.9),
        'w':(1.1,1),
        's':(.9,1),
        'e':(1,1.1),
        'd':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, node, rate, stamped, twist_frame):
        super(PublishThread, self).__init__()
        self.publisher = node.create_publisher(Twist if not stamped else TwistStamped, 'cmd_vel', 10)
        self.node = node
        self.stamped = stamped
        self.twist_frame = twist_frame
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not self.done and self.publisher.get_subscription_count() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.topic_name))
            rclpy.sleep(0.5)
            i += 1
            i = i % 5

    def update(self, x, y, z, th, speed, turn):
        with self.condition:
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.th = float(th)
            self.speed = float(speed)
            self.turn = float(turn)
            # Notify publish thread that we have a new message.
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistStamped() if self.stamped else Twist()

        while not self.done:
            with self.condition:
                self.condition.wait(self.timeout)

                if self.stamped:
                    twist = twist_msg.twist
                    twist_msg.header.stamp = self.node.get_clock().now().to_msg()
                    twist_msg.header.frame_id = self.twist_frame
                else:
                    twist = twist_msg

                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn

                self.publisher.publish(twist_msg)

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = rclpy.spin_once([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def main():
    settings = saveTerminalSettings()

    rclpy.init(args=None)
    node = rclpy.create_node('teleop_twist_keyboard')

    speed = node.declare_parameter("speed", 0.5).value
    turn = node.declare_parameter("turn", 0.5).value
    speed_limit = node.declare_parameter("speed_limit", 1.0).value
    turn_limit = node.declare_parameter("turn_limit", 1.0).value
    repeat = node.declare_parameter("repeat_rate", 0.0).value
    key_timeout = node.declare_parameter("key_timeout", 0.5).value
    stamped = node.declare_parameter("stamped", False).value
    twist_frame = node.declare_parameter("frame_id", '').value

    pub_thread = PublishThread(node, repeat, stamped, twist_frame)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if key == '\x03':
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
