import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import customtkinter as ctk
from PIL import Image
from threading import Thread

class RobotGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Initialize ROS2 node
        rclpy.init(args=None)
        self.node = Node('robot_gui')

        # Set up GUI window
        self.title("MAP3D")
        self.geometry("900x600")
        self.configure(fg_color="light blue")  # Updated to use fg_color for background

        # Set up logo
        logo_image = Image.open("/home/juan/Robotics/Proyecto/Mapeo3D_LiDAR2D/src/my_subscriber_pkg/my_subscriber_pkg/RAS.png")
        logo_image = logo_image.resize((100, 100), Image.LANCZOS)
        self.logo_photo = ctk.CTkImage(light_image=logo_image, size=(100, 100))  # Use CTkImage
        self.logo_label = ctk.CTkLabel(self, image=self.logo_photo)
        self.logo_label.place(x=700, y=10)

        # Title
        self.title_label = ctk.CTkLabel(self, text="MAP3D", font=("Times New Roman", 32, "bold"), text_color="dark blue")
        self.title_label.place(x=400, y=0)

        # Message box
        self.message_box = ctk.CTkTextbox(self, width=200, height=100)
        self.message_box.place(x=600, y=270)
        self.message_box.insert("0.0", "Messages Robot Status: ")

        # Odom data display
        self.odom_label = ctk.CTkLabel(self, text="ODOMETRY:\nPos x:\nPos y:\nAng. Z:", font=("Arial", 20, "bold"), text_color="black")
        self.odom_label.place(x=50, y=400)

        # Speed data display
        self.speed_label = ctk.CTkLabel(self, text="SPEED:\nVel. x:\nVel. y:\nAng. Z:", font=("Arial", 20, "bold"), text_color="black")
        self.speed_label.place(x=300, y=400)

        # Goal input
        self.goal_label = ctk.CTkLabel(self, text="Goal: Input [x;y]", font=("Arial", 20, "bold"), text_color="orange")
        self.goal_label.place(x=50, y=100)
        self.goal_entry = ctk.CTkEntry(self, width=200, fg_color="gray")
        self.goal_entry.place(x=50, y=130)

        # Buttons
        self.scan_button = ctk.CTkButton(self, text="Scan 3D", font=("Arial", 20, "bold"), command=self.start_scan, fg_color="red", text_color="black")
        self.scan_button.place(x=50, y=50)
        self.goal_button = ctk.CTkButton(self, text="Goal", font=("Arial", 20, "bold"), command=self.set_goal, fg_color="green", text_color="black")
        self.goal_button.place(x=50, y=170)
        self.teleop_button = ctk.CTkButton(self, text="Teleop", font=("Arial", 20, "bold"), command=self.start_teleop, fg_color="red", text_color="black")
        self.teleop_button.place(x=50, y=220)
        self.exit_button = ctk.CTkButton(self, text="Exit", font=("Arial", 18, "bold"), command=self.exit_app, fg_color="red", text_color="black")
        self.exit_button.place(x=700, y=500)
        
        # Loading bar
        self.goal_label = ctk.CTkLabel(self, text="Progress...", font=("Arial", 20, "italic"), text_color="purple")
        self.goal_label.place(x=50, y=290)
        self.loading_bar = ctk.CTkProgressBar(self, width=200, height=25)
        self.loading_bar.place(x=50, y=320)
        self.loading_bar.set(0.69)  # Set loading bar to 69%

        # Subscribers
        self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.node.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.node.create_subscription(LaserScan, '/scan2', self.scan2_callback, 10)

        # Spin ROS2 in a separate thread
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin)
        self.executor_thread.start()

    def start_scan(self):
        self.message_box.insert("end", "\nStarting 3D scan...")
        # Implement the logic to start the 3D scan

    def set_goal(self):
        goal_coords = self.goal_entry.get()
        self.message_box.insert("end", f"\nSetting goal to: {goal_coords}")
        # Implement the logic to set the goal coordinates

    def start_teleop(self):
        self.message_box.insert("end", "\nStarting teleop...")
        # Implement the logic to start teleoperation

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        self.odom_label.configure(text=f"Odom:\nPos x: {x}\nPos y: {y}\nAng. Z: {z}")

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        self.speed_label.configure(text=f"Speed:\nVel. x: {vx}\nVel. y: {vy}\nAng. Z: {vz}")

    def scan_callback(self, msg):
        # Implement the logic to handle scan data
        pass

    def scan2_callback(self, msg):
        # Implement the logic to handle scan2 data
        pass

    def exit_app(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.destroy()

if __name__ == "__main__":
    app = RobotGUI()
    app.mainloop()
