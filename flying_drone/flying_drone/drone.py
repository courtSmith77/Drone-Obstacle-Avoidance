from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from djitellopy import Tello
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from flying_interfaces.msg import LiveFeed, Command
from cv_bridge import CvBridge


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    AVOID = (auto(),)
    FORWARD = (auto(),)
    HOVER = (auto(),)


class Drone(Node):

    def __init__(self):
        super().__init__("drone")

        self.drone = Tello()

        self.connect_drone()

        # create publisher
        self.pub_live_feed = self.create_publisher(LiveFeed, "/live_feed", 10)
        self.pub_img = self.create_publisher(Image, "/image", 10)

        # create subscriber
        self.sub_command = self.create_subscription(Command, "/flight_command", self.callback_fly, 10)

        # create services
        self.srv_takeoff = self.create_service(Empty, "takeoff", self.callback_takeoff)
        self.srv_land = self.create_service(Empty, "land", self.callback_land)
        self.srv_restart_cam = self.create_service(Empty, "restart_cam", self.callback_restart_cam)

        # add cv_bridge
        self.bridge = CvBridge()
        
        self.timer = self.create_timer(0.005, self.timer_callback)

        self.state = State.HOVER

    def timer_callback(self):
        """Call timer at 200 hz."""

        # publish to live footage
        image = self.frame_reader.frame
        msg_image = LiveFeed()
        msg_image.img_flat = image.flatten()
        msg_image.img_shape = image.shape
        self.pub_live_feed.publish(msg_image)

        # try:
        #     msg_img = self.bridge.cv2_to_imgmsg(image, "bgr8")
        #     self.pub_img.publish(msg_img)
        # except Exception:
        #     self.get_logger().info("Image Publishing error")

        if self.state == State.AVOID:

            if self.fly_dist != 0:
                # 0,1,2,3 (down,left,right,up)
                if self.direction == 0:
                    self.get_logger().info("Flying Down")
                    self.drone.move_down(self.fly_dist)
                elif self.direction == 1:
                    self.get_logger().info("Flying Left")
                    self.drone.move_left(self.fly_dist)
                elif self.direction == 2:
                    self.get_logger().info("Flying Right")
                    self.drone.move_right(self.fly_dist)
                elif self.direction == 3:
                    self.get_logger().info("Flying Up")
                    self.drone.move_up(self.fly_dist)

        #     self.get_logger().info("Changing state")
        #     self.state == State.FORWARD
        
        # if self.state == State.FORWARD:
            
            if self.forward_dist != 0:
                self.get_logger().info("Flying Forward")
                self.drone.move_forward(self.forward_dist)

            self.get_logger().info("Changing state to Hover")
            self.state = State.HOVER

        if self.state == State.HOVER:
            self.get_logger().info("Hovering")
            self.direction = 4
            self.fly_dist = 0
            self.forward_dist = 0


    def connect_drone(self):

        self.drone.connect()

        self.get_logger().info(f"Battery Life Percentage: {self.drone.get_battery()}")

        # Start the video Stream
        self.drone.streamon()

        # Get the frame reader
        self.frame_reader = self.drone.get_frame_read()

    def callback_takeoff(self, request, response):

        self.get_logger().info("Taking off ...")
        self.drone.takeoff()
        self.get_logger().info("Take off complete ...")

        return response
    
    def callback_land(self, request, response):

        self.get_logger().info("Landing ...")
        self.drone.streamoff()
        self.drone.land()
        self.get_logger().info("Landing complete ...")

        return response
    
    def callback_fly(self, msg):

        self.direction = msg.direction
        self.fly_dist = msg.fly_dist
        self.forward_dist = msg.fly_forward

        self.get_logger().info("Command Received, avoiding now")

        self.state = State.AVOID

    def callback_restart_cam(self, request, response):

        self.get_logger().info("Restarting Camera ...")
        self.drone.streamoff()

        self.drone.streamon()

        self.frame_reader = self.drone.get_frame_read()
        self.get_logger().info("Camera Restarted ...")

        return response


def main(args=None):
    """Call node function."""
    rclpy.init(args=args)
    robo = Drone()
    rclpy.spin(robo)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
