


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
from .modules.helper import detect_arrow, classify_direction, important_edge, control_cmds, symbol_center, special_cmds

class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    FLYING = (auto(),)
    HOVER = (auto(),)
    SPECIAL_FUNCTION = (auto(),)
    AVOID = (auto(),)

class Drone(Node):
    """
    Fly the drone through an obstacle course.

    PUBLISHERS:
    Topic name: "image", Type: sensor_msgs/msg/Image -
    the live feed image from the drone

    SERVICES:
    Topic name: "takeoff", Type: std_srvs/srv/Empty -
    command the drone to takeoff

    Topic name: "land", Type: std_srvs/srv/Empty -
    command the drone to land

    Topic name: "stop", Type: std_srvs/srv/Empty -
    command the drone to stop flying and hover

    Topic name: "celebrate", Type: std_srvs/srv/Empty -
    command the drone to do a back flip

    Topic name: "rotate", Type: std_srvs/srv/Empty -
    command the drone to rotate 180 degrees

    Topic name: "forward", Type: std_srvs/srv/Empty -
    command the drone to fly forward 40 cm

    Parameters
    ----------
    Name: "model_detect_path", Type: String - the location of the detection model weights

    Name: "model_classify_arrow_path", Type: String - the location of the arrow classification model weights

    Name: "model_classify_symbol_path", Type: String - the location of the special symbol classification model weights

    """
    def __init__(self):
        super().__init__("drone")

        self.declare_parameter("model_detect_path", "/home/csmith/Desktop/School/Winter 2024/winter Project/ws/Winter-Project/fly_drone/config/detect_best.pt")
        self.model_detect_path = self.get_parameter("model_detect_path").get_parameter_value().string_value
        self.declare_parameter("model_classify_arrow_path", "/home/csmith/Desktop/School/Winter 2024/winter Project/ws/Winter-Project/fly_drone/config/classify_best.pt")
        self.model_classify_arrow_path = self.get_parameter("model_classify_arrow_path").get_parameter_value().string_value
        self.declare_parameter("model_classify_symbol_path", "/home/csmith/Desktop/School/Winter 2024/winter Project/ws/Winter-Project/fly_drone/config/symbol_best.pt")
        self.model_classify_symbol_path = self.get_parameter("model_classify_symbol_path").get_parameter_value().string_value

        # initialize models
        self.model_detect = YOLO(self.model_detect_path)
        self.model_classify_arrow = YOLO(self.model_classify_arrow_path)
        self.model_classify_symbol = YOLO(self.model_classify_symbol_path)

        # create publisher
        self.pub_img = self.create_publisher(Image, "/image", 10)

        # create services
        self.srv_takeoff = self.create_service(Empty, "takeoff", self.callback_takeoff)
        self.srv_land = self.create_service(Empty, "land", self.callback_land)
        self.srv_stop = self.create_service(Empty, "stop", self.callback_stop)
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
        # Get the footage from drone
        self.frame_reader = self.drone.get_frame_read()
        self.image = self.frame_reader.frame

        try:
            # publish image to RVIZ topic
            resized = cv2.resize(self.image, (0,0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
            msg_img = self.bridge.cv2_to_imgmsg(resized, "rgb8")
            msg_img.header.stamp = self.get_clock().now().to_msg()
            msg_img.header.frame_id = "world"
            self.pub_img.publish(msg_img)
        except Exception:
            self.get_logger().info("Image Publishing error")

        if self.state == State.FLYING:
            """Detect arrow or special symbol and changes state accordingly."""
            self.box = detect_arrow(self.model_detect, self.image)

            if self.box is not None:

                # publish image with detection box shown
                img_box = cv2.rectangle(self.image, (int(self.box[0]),int(self.box[3])), (int(self.box[2]), int(self.box[1])), color=(0,0,0), thickness=2)
                resized = cv2.resize(img_box, (0,0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
                msg_img = self.bridge.cv2_to_imgmsg(resized, "rgb8")
                msg_img.header.stamp = self.get_clock().now().to_msg()
                msg_img.header.frame_id = "world"
                self.pub_img.publish(msg_img)

                img_ratio = (self.box[3]-self.box[1])/(self.box[2]-self.box[0])

                if (img_ratio < 1.1) & (img_ratio > 0.82):
                    self.state = State.SPECIAL_FUNCTION
                else :
                    self.state = State.AVOID

                self.no_detections = 0
            
            else:

                # stop flying if nothing is detected for 20 frames   
                self.no_detections += 1
                if self.no_detections > 20 :
                    self.get_logger().info("No detections 20 frames in a row")
                    self.get_logger().info("Terminating ...")
                    self.state = State.HOVER

        if self.state == State.AVOID :
            """Classifies arrow direction and moves drone accordingly."""
            class_id = classify_direction(self.model_classify_arrow, self.image, self.box)

            if class_id is not None:
                
                area = abs((self.box[2]-self.box[0]) * (self.box[3]-self.box[1]))
                edge = important_edge(class_id, self.box)
                x_c, y_c = symbol_center(self.box)

                forward, dist, ortho = control_cmds(self, area, edge, x_c, y_c, class_id)
                print(f"Command: dir = {class_id}, dist = {dist}, forward = {forward}, ortho = {ortho}")
            
                # 0,1,2,3 (down,left,right,up)
                if forward == 0 and dist == 0 and ortho == 0:
                    self.get_logger().info("Do nothing")
                else :
                    if class_id == 0:
                        self.get_logger().info("Flying Down")
                        self.drone.go_xyz_speed(forward, ortho, -dist, 15)
                    elif class_id == 1:
                        self.get_logger().info("Flying Left")
                        self.drone.go_xyz_speed(forward, dist, ortho, 15)
                    elif class_id == 2:
                        self.get_logger().info("Flying Right")
                        self.drone.go_xyz_speed(forward, -dist, ortho, 15)
                    elif class_id == 3:
                        self.get_logger().info("Flying Up")
                        self.drone.go_xyz_speed(forward, ortho, dist, 15)
                
            self.state = State.FLYING
        
        if self.state == State.SPECIAL_FUNCTION :
            """Classifies special symbol and move drone accordingly."""
            class_id = classify_direction(self.model_classify_symbol, self.image, self.box)

            if class_id is not None:
                
                area = abs((self.box[2]-self.box[0]) * (self.box[3]-self.box[1]))
                self.get_logger().info(f"Area of Box = {area}")
                x_c, y_c = symbol_center(self.box)

                forward, x, y = special_cmds(self, area, x_c, y_c)
                print(f"Command: dir = {class_id}, forward = {forward}, x = {x}, y = {y}")

                # 0,1 (star, uturn)
                if class_id == 0:
                    
                    if forward == 0 and x == 0 and y == 0:

                        enter = input("Flip now? y/n")
                        if enter == "y":
                            self.get_logger().info("You Completed the Course!")
                            self.get_logger().info("Flipping!")
                            self.drone.flip("b")
                            self.state = State.HOVER
                        else:
                            self.get_logger().info("Not flipping Moving forward")
                            self.drone.go_xyz_speed(20, 0, 0, 15)
                            self.state = State.FLYING

                    else:
                        self.get_logger().info("Moving closer to Star")
                        self.drone.go_xyz_speed(forward, x, y, 15)
                        self.state = State.FLYING
                
                elif class_id == 1:

                    if forward == 0 and x == 0 and y == 0:
                        self.get_logger().info("Turning Around")
                        self.drone.rotate_clockwise(180)
                    else:
                        self.get_logger().info("Moving closer to UTurn")
                        self.drone.go_xyz_speed(forward, x, y, 15)
                    self.state = State.FLYING
                
                else:
                    self.state = State.FLYING
            else:
                self.state = State.FLYING

    def connect_drone(self):
        """Connects to drone and begins camera feed.
        
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
        """Moves drone forward 40cm.
        
        Request: None
        Response: None
        """
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
