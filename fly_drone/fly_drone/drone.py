from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from djitellopy import Tello
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from .modules.helper import detect_arrow, classify_direction, important_edge, control_cmds

class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    FLYING = (auto(),)
    HOVER = (auto(),)

class Drone(Node):

    def __init__(self):
        super().__init__("drone")

        self.declare_parameter("model_detect_path", "")
        self.model_detect_path = self.get_parameter("model_detect_path").get_parameter_value().string_value
        self.declare_parameter("model_classify_path", "")
        self.model_classify_path = self.get_parameter("model_classify_path").get_parameter_value().string_value
        
        # initialize model
        self.model_detect = YOLO(self.model_detect_path)
        self.model_classify = YOLO(self.model_classify_path)

        # create publisher
        self.pub_img = self.create_publisher(Image, "/image", 10)

        # create services
        self.srv_takeoff = self.create_service(Empty, "takeoff", self.callback_takeoff)
        self.srv_land = self.create_service(Empty, "land", self.callback_land)
        self.srv_stop = self.create_service(Empty, "stop", self.callback_stop)
        self.srv_restart_cam = self.create_service(Empty, "restart_cam", self.callback_restart_cam)
        self.srv_celebrate = self.create_service(Empty, "celebrate", self.callback_celebrate)
        self.srv_rotate = self.create_service(Empty, "rotate", self.callback_rotate)
        self.srv_forward = self.create_service(Empty, "forward", self.callback_forward)

        # add cv_bridge
        self.bridge = CvBridge()
        
        # timer frequency same as camera frame rate
        self.timer = self.create_timer(0.033333, self.timer_callback)

        # set up drone
        self.drone = Tello()
        self.connect_drone()

        # number of no detections in a row
        self.no_detections = 0
    
        self.state = State.HOVER

    def timer_callback(self):
        """Call timer at 30 hz."""

        # publish to live footage
        # Get the frame reader
        self.frame_reader = self.drone.get_frame_read()
        self.image = self.frame_reader.frame

        try:
            resized = cv2.resize(self.image, (0,0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
            msg_img = self.bridge.cv2_to_imgmsg(resized, "rgb8")
            msg_img.header.stamp = self.get_clock().now().to_msg()
            msg_img.header.frame_id = "world"
            self.pub_img.publish(msg_img)
        except Exception:
            self.get_logger().info("Image Publishing error")

        if self.state == State.FLYING:

            box = detect_arrow(self.model_detect, self.image)

            if box is not None:

                # publish image with box on it
                img_box = cv2.rectangle(self.image, (int(box[0]),int(box[3])), (int(box[2]), int(box[1])), color=(0,0,0), thickness=2)
                resized = cv2.resize(img_box, (0,0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
                msg_img = self.bridge.cv2_to_imgmsg(resized, "rgb8")
                msg_img.header.stamp = self.get_clock().now().to_msg()
                msg_img.header.frame_id = "world"
                self.pub_img.publish(msg_img)

                area = abs((box[2]-box[0]) * (box[3] - box[1]))
                class_id = classify_direction(self.model_classify, self.image, box)

                if class_id is not None:

                    edge = important_edge(class_id, box)

                    forward, dist = control_cmds(self, area, edge, class_id)
                    print(f"Command: dir = {class_id}, dist = {dist}, forward = {forward}")
                
                    # 0,1,2,3 (down,left,right,up)
                    if forward == 0 and dist == 0:
                        self.get_logger().info("Do nothing")
                    else :
                        if class_id == 0:
                            self.get_logger().info("Flying Down")
                            self.drone.go_xyz_speed(forward, 0, -dist, 15)
                        elif class_id == 1:
                            self.get_logger().info("Flying Left")
                            self.drone.go_xyz_speed(forward, dist, 0, 15)
                        elif class_id == 2:
                            self.get_logger().info("Flying Right")
                            self.drone.go_xyz_speed(forward, -dist, 0, 15)
                        elif class_id == 3:
                            self.get_logger().info("Flying Up")
                            self.drone.go_xyz_speed(forward, 0, dist, 15)
                
                self.no_detections = 0
            
            else:
                
                self.no_detections += 1
                if self.no_detections > 20 :
                    self.get_logger().info("No detections 20 frames in a row")
                    self.get_logger().info("Terminating ...")
                    self.state = State.HOVER



    def connect_drone(self):
        """Connects to drone and begin camera feed.
        
        Request: None
        Response: None
        """
        self.drone.connect()

        self.get_logger().info(f"Battery Life Percentage: {self.drone.get_battery()}")

        # Start the video Stream
        self.drone.streamon()

    def callback_takeoff(self, request, response):
        """Communicates Drone Takeoff and begin flying state.
        
        Request: None
        Response: None
        """
        self.get_logger().info("Taking off ...")
        self.drone.takeoff()
        self.get_logger().info("Take off complete ...")

        # self.drone.move_down(20)

        self.state = State.FLYING

        return response
    
    def callback_land(self, request, response):
        """Lands drone.
        
        Request: None
        Response: None
        """
        self.get_logger().info("Landing ...")
        self.drone.streamoff()
        self.drone.land()
        self.get_logger().info("Landing complete ...")

        self.state = State.HOVER

        return response

    def callback_restart_cam(self, request, response):
        """Restarts the camera feed.
        
        Request: None
        Response: None
        """
        self.get_logger().info("Restarting Camera ...")
        self.drone.streamoff()

        self.drone.streamon()

        self.frame_reader = self.drone.get_frame_read()
        self.get_logger().info("Camera Restarted ...")

        return response
    
    def callback_stop(self, request, response):
        """Stops drone flight but continues to hover.
        
        Request: None
        Response: None
        """
        self.get_logger().info("Stopping Flight")

        self.state = State.HOVER

        return response

    def callback_celebrate(self, request, response):
        """Flips drone backwards
        
        Request: None
        Response: None
        """
        self.drone.flip("b")

        return response

    def callback_rotate(self, request, response):
        """Rotates drone 180 degrees and begins flying again.
        
        Request: None
        Response: None
        """
        self.drone.rotate_clockwise(180)

        self.state = State.FLYING

        return response

    def callback_forward(self, request, response):

        self.drone.move_forward(40)

        return response

def main(args=None):
    """Call node function."""
    rclpy.init(args=args)
    robo = Drone()
    rclpy.spin(robo)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
