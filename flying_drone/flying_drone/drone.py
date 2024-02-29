from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from djitellopy import Tello
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from flying_interfaces.msg import LiveFeed, Command
from cv_bridge import CvBridge
import cv2


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    WAITING = (auto(),)
    AVOID = (auto(),)
    FORWARD = (auto(),)
    HOVER = (auto(),)


class Drone(Node):

    def __init__(self):
        super().__init__("drone")

        self.drone = Tello()

        self.connect_drone()

        # create publisher
        self.pub_img = self.create_publisher(Image, "/image", 10)

        # create subscriber
        self.sub_command = self.create_subscription(Command, "/flight_command", self.callback_fly, 10)

        # create services
        self.srv_takeoff = self.create_service(Empty, "takeoff", self.callback_takeoff)
        self.srv_land = self.create_service(Empty, "land", self.callback_land)
        self.srv_stop = self.create_service(Empty, "stop", self.callback_stop)
        self.srv_restart_cam = self.create_service(Empty, "restart_cam", self.callback_restart_cam)
        self.srv_celebrate = self.create_service(Empty, "celebrate", self.callback_celebrate)

        # add cv_bridge
        self.bridge = CvBridge()
        
        self.timer = self.create_timer(0.01, self.timer_callback)

        # wait until person triggers to begin
        self.first = True

        self.state = State.WAITING

    def timer_callback(self):
        """Call timer at 100 hz."""

        # publish to live footage
        # Get the frame reader
        self.frame_reader = self.drone.get_frame_read()
        image = self.frame_reader.frame

        try:
            resized = cv2.resize(image, (0,0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
            msg_img = self.bridge.cv2_to_imgmsg(resized, "rgb8")
            msg_img.header.stamp = self.get_clock().now().to_msg()
            msg_img.header.frame_id = "world"
            self.pub_img.publish(msg_img)
        except Exception:
            self.get_logger().info("Image Publishing error")

        # # check if drone has stopped 
        # if not self.first:
        #     x_speed = self.drone.get_speed_x()
        #     y_speed = self.drone.get_speed_y()
        #     z_speed = self.drone.get_speed_z()

        #     if x_speed == 0 and y_speed == 0 and z_speed == 0:
        #         self.state = State.AVOID

        if self.state == State.AVOID:
            
            self.get_logger().info("Inside Avoid")
            self.get_logger().info(f"Direction command = {self.direction}")
            self.get_logger().info(f"Command = direction: {self.direction}, fly: {self.fly_dist}, forward: {self.forward_dist}")
            # 0,1,2,3 (down,left,right,up)
            if self.forward_dist == 0 and self.fly_dist == 0:
                self.get_logger().info("Do nothing")
            else :
                if self.direction == 0:
                    self.get_logger().info("Flying Down")
                    self.drone.go_xyz_speed(self.forward_dist, 0, -self.fly_dist, 10)
                elif self.direction == 1:
                    self.get_logger().info("Flying Left")
                    self.drone.go_xyz_speed(self.forward_dist, self.fly_dist, 0, 10)
                elif self.direction == 2:
                    self.get_logger().info("Flying Right")
                    self.drone.go_xyz_speed(self.forward_dist, -self.fly_dist, 0, 10)
                elif self.direction == 3:
                    self.get_logger().info("Flying Up")
                    self.drone.go_xyz_speed(self.forward_dist, 0, self.fly_dist, 10)

            # self.get_logger().info("Changing state")
            self.state = State.HOVER

        # if self.state == State.HOVER:
            # self.get_logger().info("Hovering")
            # self.direction = 0
            # self.fly_dist = 0
            # self.forward_dist = 0


    def connect_drone(self):

        self.drone.connect()

        self.get_logger().info(f"Battery Life Percentage: {self.drone.get_battery()}")

        # Start the video Stream
        self.drone.streamon()

    def callback_takeoff(self, request, response):

        self.get_logger().info("Taking off ...")
        self.drone.takeoff()
        self.get_logger().info("Take off complete ...")

        self.state = State.HOVER

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
        self.get_logger().info(f"Command = direction: {self.direction}, fly: {self.fly_dist}, forward: {self.forward_dist}")

        self.first = False
        self.state = State.AVOID

    def callback_restart_cam(self, request, response):

        self.get_logger().info("Restarting Camera ...")
        self.drone.streamoff()

        self.drone.streamon()

        self.frame_reader = self.drone.get_frame_read()
        self.get_logger().info("Camera Restarted ...")

        return response
    
    def callback_stop(self, request, response):

        self.get_logger().info("Stopping Flight")
        
        self.first = True
        self.state = State.HOVER

    def callback_celebrate(self, request, response):

        self.drone.flip("b")


def main(args=None):
    """Call node function."""
    rclpy.init(args=args)
    robo = Drone()
    rclpy.spin(robo)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
