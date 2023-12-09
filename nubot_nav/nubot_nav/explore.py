"""The Dora node makes the nubot explore its environment."""
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from math import pi
from random import uniform
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Explore(Node):
    """
    The Explore node governs the frontier exploration of the robot.

    PUBLISHERS:

        goalposepub (geometry_msgs/msg/PoseStamped) : Publishes the 2-D goalpose of the nubot.

    SUBSCRIBERS:

        mapsub (nav_msgs/msg/OccupancyGrid) : Loads map into the node.
        posesub (geometry_msgs/msg/PoseWithCovarianceStamped) : Gets current pose of the nubot.

    """

    def __init__(self):
        super().__init__("dora")

        # Define flags.
        self.pose_known = False
        self.tf_known = False
        self.allgood = True

        # Define transform of nubot.
        self.prevtf = TransformStamped()

        # Define goal pose.
        self.targetPose = PoseStamped()
        self.targetPose.header.frame_id = "map"

        # Define current pose of nubot
        self.currentPose = PoseStamped()
        self.currentPose.header.frame_id = "map"

        # The buffer stores received tf frames.
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # SUBSCRIBERS

        # Create /map subscriber
        self.mapsub = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, qos_profile=10
        )

        # Create /pose subscriber
        self.posesub = self.create_subscription(
            PoseWithCovarianceStamped, "pose", self.pose_callback, qos_profile=10
        )

        # Create a timer.
        self.frequency = 0.1  # Node frequency
        self.dt = 1 / self.frequency  # Node timestep
        self.tmr = self.create_timer(self.dt, self.timer_callback)

        # Create Publisher for /goal_pose topic.
        self.goalposepub = self.create_publisher(PoseStamped, "goal_pose", 10)

    def map_callback(self, msg: OccupancyGrid):
        """
        Load map into node.

        Args:
        ----
            msg (nav_msgs/msg/OccupancyGrid) : The current map.

        """
        self.map = np.zeros([msg.info.height, msg.info.width])

        self.map_res = msg.info.resolution
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_origin = msg.info.origin

        self.map[:] = np.reshape(msg.data, (msg.info.height, msg.info.width))

        # self.get_logger().info(f"{self.map}")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Get the pose of the nubot.

        Args:
        ----
            msg (geometry_msgs/msg/PoseWithCovarianceStamped) : The current pose of the robot.

        """
        self.currentPose.header.stamp = self.get_clock().now().to_msg()
        self.currentPose.pose = msg.pose.pose

        self.pose_known = True

    def timer_callback(self):
        """Decide the behaviour of the node at each timestep."""
        self.targetPose.header.stamp = self.get_clock().now().to_msg()

        self.targetPose.pose = self.currentPose.pose

        try:
            # get the latest transform from world to brick.
            tf = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Frames don't exist yet: {e}")
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")

        self.get_logger().info(
            f"{tf.transform.translation.x, tf.transform.translation.y }"
        )

        # Push the nubot out of where it is if it is not moving.
        if round(self.prevtf.transform.translation.x, 1) == round(
            tf.transform.translation.x, 1
        ) and round(self.prevtf.transform.translation.y, 1) == round(
            tf.transform.translation.y, 1
        ):
            # REPRESENT CRAZZZY TURTLE
            r = 8
            theta = uniform(-pi, pi)

            self.targetPose.pose.position.x = (
                self.currentPose.pose.position.x + r * np.cos(theta)
            )
            self.targetPose.pose.position.y = (
                self.currentPose.pose.position.y + r * np.sin(theta)
            )

            self.goalposepub.publish(self.targetPose)

            self.pose_known = False

        self.prevtf = tf

        # Make the robot move every timestep if it is not supposed to stop.
        if self.pose_known:
            # REPRESENT CRAZZZY TURTLE
            r = 8
            theta = uniform(-pi, pi)

            self.targetPose.pose.position.x = (
                self.currentPose.pose.position.x + r * np.cos(theta)
            )
            self.targetPose.pose.position.y = (
                self.currentPose.pose.position.y + r * np.sin(theta)
            )

            # self.goalposepub.publish(self.targetPose)

            self.pose_known = False

        # Flag when the /pose topic is not published.
        else:
            self.get_logger().info("MISSED OPPORTUNITY ")


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
