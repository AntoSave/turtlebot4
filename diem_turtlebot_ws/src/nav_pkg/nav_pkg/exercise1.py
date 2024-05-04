import rclpy
import threading
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Duration

class MyNode(Node):
    def __init__(self):
        super().__init__("nav_node") #Init node
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Navigation node initialized")
        self.create_timer(0.1, self.move_callback) #Creating a timer
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publisher
        self.callback_group = ReentrantCallbackGroup()
        self.rviz_visual_tools_subscriber = self.create_subscription(Joy, "/rviz_visual_tools_gui", self.handle_button, 10, callback_group=self.callback_group)
        self.ok = False
        self._next = False

    def handle_button(self, msg: Joy):
        print("Button pressed", msg._buttons.index(1))
        if msg._buttons.index(1) == 1:
            self._next = True
            print("Button pressed 2")

    def move_callback(self):
        if self.ok:
            self.cmd_pub.publish(self.msg)

    def rotate(self, clockwise=True):
        factor = -1 if clockwise else 1
        msg = Twist()
        msg.angular.z = 1.57 * factor #rad/s
        self.msg = msg
        self.ok = True
        self.sleep(1)
        self.ok = False

    def go(self):
        msg = Twist()
        msg.linear.x = 0.25 #m/s
        self.msg = msg
        self.ok = True
        self.sleep(4)
        self.ok = False

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0 #m/s
        self.msg = msg
        self.ok = True
        self.sleep(1)
        self.ok = False

    def wait_for_next(self):
        self._next = False
        self.get_logger().info("Press Next to continue")
        while not self._next:
            pass

    def sleep(self, time_seconds):
        self.get_clock().sleep_for(Duration(seconds=time_seconds)) #sleep for <time_seconds> seconds

    def follow_trajectory(self):
        self.get_logger().info("Node starting...")
        self.wait_for_next()
        self.get_logger().info("Moving forward...")
        self.go() #Moving forward for 4 seconds
        self.wait_for_next()
        self.get_logger().info("Rotating...")
        self.rotate() #Rotating for 1 seconds
        self.wait_for_next()
        self.get_logger().info("Moving forward...")
        self.go() #Move forward for 4 seconds
        self.wait_for_next()
        self.get_logger().info("Stopping...")
        self.stop()
        self.get_logger().info("End!")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = MyNode()
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True) #Creating 'use_sim_time' node parameter
    node.set_parameters([param]) #Setting 'use_sim_time' node parameter
    executor.add_node(node) #Adding node to executor
    executor.create_task(node.follow_trajectory) #Creating a task with callable given as input
    try:
        executor.spin() #Running loop - bocking call
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == "__main__":
    main()