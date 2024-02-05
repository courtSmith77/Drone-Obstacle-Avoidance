from enum import Enum, auto
import rclpy
from rclpy.node import Node
from flying_interfaces.msg import LiveFeed, Detection, Classify
from ultralytics import YOLO
import cv2
import numpy as np


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    CONNECTING_FEED = (auto(),)
    DETECTING = (auto(),)
    CLASSIFYING = (auto(),)

def order_corner_points(corners):
    top_r, top_l, bottom_l, bottom_r = corners[0], corners[1], corners[2], corners[3]
    return (top_l, top_r, bottom_r, bottom_l)

def perspective_transform(image, corners):

    ordered_corners = order_corner_points(corners)
    top_l, top_r, bottom_r, bottom_l = ordered_corners

    width_A = np.sqrt(((bottom_r[0] - bottom_l[0]) ** 2) + ((bottom_r[1] - bottom_l[1]) ** 2))
    width_B = np.sqrt(((top_r[0] - top_l[0]) ** 2) + ((top_r[1] - top_l[1]) ** 2))
    width = max(int(width_A), int(width_B))

    height_A = np.sqrt(((top_r[0] - bottom_r[0]) ** 2) + ((top_r[1] - bottom_r[1]) ** 2))
    height_B = np.sqrt(((top_l[0] - bottom_l[0]) ** 2) + ((top_l[1] - bottom_l[1]) ** 2))
    height = max(int(height_A), int(height_B))

    dimensions = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], 
                    [0, height - 1]], dtype = "float32")
    ordered_corners = np.array(ordered_corners, dtype="float32")
    matrix = cv2.getPerspectiveTransform(ordered_corners, dimensions)
    transformed_image = cv2.warpPerspective(image, matrix, (width, height))

    return transformed_image

def detect_arrow(model, img):

    results = model.predict(img)

    detected = None
    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, score, detected = result

    if detected is not None:
        return detected
    else :
        return [x1, y1, x2, y2]
    
def classify_direction(model, img, box):

    yolo_corners = [[box[0],box[3]],[box[2],box[3]],[box[2],box[1]],[box[0],box[1]]] # top_l, top_r, bot_r, bot_l

    transformed = perspective_transform(img, yolo_corners)
    flipped = cv2.flip(transformed, -1)

    pred = model.predict(flipped)

    class_id = None
    if pred is not None:
        class_id = pred[0].probs.top1

    return class_id


class Vision(Node):

    def __init__(self):
        super().__init__("vision")

        # initialize parameters
        self.declare_parameter("model_detect_path")
        self.model_detect_path = self.get_parameter("model_detect_path").get_parameter_value().string_value
        self.declare_parameter("model_classify_path")
        self.model_classify_path = self.get_parameter("model_classify_path").get_parameter_value().string_value
        
        # initialize model
        # self.model_detect = YOLO("./model_weights/detect_best.pt")
        # self.model_classify = YOLO("./model_weights/classify_best.pt")
        self.model_detect = YOLO(self.model_detect_path)
        self.model_classify = YOLO(self.model_classify_path)

        # create subscriber
        self.sub_live_feed = self.create_subscription(LiveFeed, "/live_feed", self.callback_livefeed, 10)
        
        # create publishers
        self.pub_detect = self.create_publisher(Detection, "/detection", 10)
        self.pub_class = self.create_client(Classify, "/classify", 10)

        self.state = State.CONNECTING_FEED

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """Call timer at 100 hz."""

        if self.state == State.CONNECTING_FEED:
            self.get_logger().info(f"Waiting for camera feed")
        
        if self.state == State.DETECTING:

            self.detected_img = self.live_img
            box = detect_arrow(self.model_detect, self.live_img)

            if box is not None:

                self.pub_detect.publish(box)
                self.box = box
                
                self.state = State.CLASSIFYING
        
        if self.state == State.CLASSIFYING:

            class_id = classify_direction(self.model_classify, self.detected_img, self.box)

            if class_id is not None:

                self.pub_class.publish(class_id)

                self.state = State.DETECTING


    def callback_livefeed(self, msg):

        img_flat = msg.img_flat
        img_shape = msg.img_shape

        self.live_img = img_flat.reshape(img_shape)

        self.state = State.DETECTING


def main(args=None):
    """Call node function."""
    rclpy.init(args=args)
    cam = Vision()
    rclpy.spin(cam)
    rclpy.shutdown()


if __name__ == "__main__":
    main()