#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (0, 0, 0, -1),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (0, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, -1),
    'm': (-1, 0, 0, 1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


class PublishThread(threading.Thread, Node):
    def __init__(self, rate):
        super().__init__()
        self.node = rclpy.create_node('my2_teleop_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = Twist()

        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist_msg.linear.x = self.x * self.speed
            twist_msg.linear.y = self.y * self.speed
            twist_msg.linear.z = self.z * self.speed
            twist_msg.angular.x = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
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


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_twist_keyboard')

    speed = node.get_parameter('speed', 0.5).value
    turn = node.get_parameter('turn', 0.5).value
    speed_limit = node.get_parameter('speed_limit', 1.0).value
    turn_limit = node.get_parameter('turn_limit', 1.0).value
    repeat = node.get_parameter('repeat_rate', 0.0).value
    key_timeout = node.get_parameter('key_timeout', 0.52).value
    stamped = node.get_parameter('stamped', False).value
    twist_frame = node.get_parameter('frame_id', '').value

    pub_thread = PublishThread(node, repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while rclpy.ok():
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
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    settings = saveTerminalSettings()
    main()
