import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from math import pi
from random import uniform
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Explore(Node):

    def __init__(self):
        super().__init__("dora")

        self.pose_known = False
        self.tf_known = False

        self.targetPose = PoseStamped()
        self.targetPose.header.frame_id = "map"

        self.currentPose = PoseStamped()
        self.currentPose.header.frame_id = "map"

        # The buffer stores received tf frames.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Create subscribers
        self.mapsub = self.create_subscription(OccupancyGrid, "map", self.map_callback, qos_profile=10)
        self.posesub = self.create_subscription(PoseWithCovarianceStamped, "pose", self.pose_callback, qos_profile=10)

        # Create a timer.
        self.frequency = 0.1 # Node frequency
        self.dt = 1/self.frequency # Node timestep
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # Create Publisher for /goal_pose topic.
        self.goalposepub = self.create_publisher(PoseStamped, "goal_pose", 10)

    def map_callback(self, msg: OccupancyGrid):

        self.map = np.zeros([msg.info.height, msg.info.width])
        # self.get_logger().info(f"{np.shape(self.map), [msg.info.height, msg.info.width]}")

        self.map_res = msg.info.resolution
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_origin = msg.info.origin

        self.map[:] = np.reshape(msg.data, (msg.info.height, msg.info.width))

        # self.get_logger().info(f"{self.map}")

    def pose_callback(self, msg: PoseWithCovarianceStamped):

        self.currentPose.header.stamp = self.get_clock().now().to_msg()
        self.currentPose.pose = msg.pose.pose
       
        self.get_logger().info(f"LOOOOOOOOOK  HEEEEEEEEREEEEEEE ")
        self.get_logger().info(f"Current Pose: {self.currentPose.pose.position.x, self.currentPose.pose.position.y}")

        self.pose_known = True

    def timer_callback(self):

        self.targetPose.header.stamp = self.get_clock().now().to_msg()

        self.targetPose.pose = self.currentPose.pose

        # self.targetPose.pose.orientation.x = 0
        # self.targetPose.pose.orientation.y = 0
        # self.targetPose.pose.orientation.w = uniform(0,1)
        # self.targetPose.pose.orientation.z = (1 - self.targetPose.pose.orientation.w**2)**0.5

        # self.targetPose.pose.position.x = self.targetPose.pose.position.x

        # self.targetPose.pose.position.x = 13.0
        # self.targetPose.pose.position.y = -19.5
        # self.targetPose.pose.position.z = 0.0

        try:
            # get the latest transform from world to brick.
            tf = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.empty = 0
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")

        self.get_logger().info(f"{tf.transform.translation.x, tf.transform.translation.y }")

        if self.pose_known:

            # REPRESENT CRAZZZY TURTLE
            r = 5
            theta = uniform(-pi,pi)

            self.targetPose.pose.position.x = self.currentPose.pose.position.x + r * np.cos(theta)
            self.targetPose.pose.position.y = self.currentPose.pose.position.y + r * np.sin(theta)

            self.goalposepub.publish(self.targetPose)
            self.get_logger().info(f"Published goalpose")
            self.get_logger().info(f"CRAZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZYYYYYYYYY TURTLEEEEEEEEEEEEEEEEEEEEEEEEE")

            self.pose_known = False
        
        else:

            self.get_logger().info(f"MISSED OPPORTUNITY ")


        pass

def main(args=None):
    rclpy.init(args=args)

    dora = Explore()

    rclpy.spin(dora)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dora.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()