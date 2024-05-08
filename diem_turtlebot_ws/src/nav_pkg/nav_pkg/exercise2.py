import rclpy
import threading
from turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
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
        self.navigator = TurtleBot4Navigator()

    def handle_button(self, msg: Joy):
        if msg._buttons.index(1) == 1:
            self._next = True

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
        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 Ready")
        waypoints = [self.navigator.getPoseStamped([-25.2043, 1.1586], TurtleBot4Directions.NORTH),
                     self.navigator.getPoseStamped([-25.7649, -10.0092], TurtleBot4Directions.NORTH),
                     self.navigator.getPoseStamped([-23.7542, -9.6984], TurtleBot4Directions.NORTH),
                     self.navigator.getPoseStamped([-23.05457, 0.8257], TurtleBot4Directions.NORTH)]

        for goal_pose in waypoints:
            self.wait_for_next()
            self.navigator.startToPose(goal_pose)

        

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