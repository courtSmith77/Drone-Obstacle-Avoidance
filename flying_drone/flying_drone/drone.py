from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from djitellopy import Tello
from std_srvs.srv import Empty
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

        # create publisher
        self.pub_live_feed = self.create_publisher(LiveFeed, "/live_feed", 10)

        # create services
        self.srv_takeoff = self.create_service(Empty, "takeoff", self.callback_takeoff)
        self.srv_land = self.create_service(Empty, "land", self.callback_land)
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """Call timer at 100 hz."""

        # publish to live footage
        image = self.frame_reader.frame
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

    def callback_takeoff(self, request, response):

        print("Taking off ...")
        self.drone.takeoff()
        print("Take off complete ...")

        return response
    
    def callback_land(self, request, response):

        print("Landing ...")
        self.drone.streamoff()
        self.drone.land()
        print("Landing complete ...")

        return response


def main(args=None):
    """Call node function."""
    rclpy.init(args=args)
    robo = Drone()
    rclpy.spin(robo)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
