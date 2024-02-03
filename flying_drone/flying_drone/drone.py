from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from djitellopy import Tello
from std_msgs.msg import Empty
from flying_interfaces.msg import LiveFeed



class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    STOPPED = (auto(),)
    GOAL = (auto(),)
    CENTER = (auto(),)
    POSITIONED = (auto(),)
    TILTING = (auto(),)


class Drone(Node):

    def __init__(self):
        super().__init__("drone")

        self.drone = Tello()

        self.connect_drone()

        self.pub_live_feed = self.create_publisher(LiveFeed, "/live_feed", 10)

        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """Call timer at 100 hz."""

        image = self.frame_reader.frame
        self.get_logger().info(f"Image type: {type(image)}")
        self.get_logger().info(f"Imape shape: {image.shape}")
        msg_image = LiveFeed()
        msg_image.img_flat = image.flatten()
        msg_image.img_shape = image.shape
        self.pub_live_feed.publish(msg_image)



    def connect_drone(self):

        self.drone.connect()

        print(f"Battery Life Percentage: {self.drone.get_battery()}")

        # Start the video Stream
        self.drone.streamon()

        # Get the frame reader
        self.frame_reader = self.drone.get_frame_read()
        
   


def main(args=None):
    """Call node function."""
    rclpy.init(args=args)
    robo = Drone()
    rclpy.spin(robo)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
