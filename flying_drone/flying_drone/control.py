from enum import Enum, auto
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from flying_interfaces.msg import Detection, Classify, Command

class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    WAIT = (auto(),)
    FLY = (auto(),)
    HOVER = (auto(),)

def control_cmds(area, edge, classify):

    if area > 45000:
        fly_forward = 0
    else:
        fly_forward = 20
    
    if area < 20000:
        fly_dist = 0
    else :        
        if classify == 0:
            dist_from_center = 360 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center < 200:
                fly_dist = 20
            else:
                fly_dist = 0

        elif classify == 1:
            dist_from_center = 640 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center < 200:
                fly_dist = 20
            else:
                fly_dist = 0

        elif classify == 2:
            dist_from_center = 640 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center > -200:
                fly_dist = 20
            else:
                fly_dist = 0

        elif classify == 3:
            dist_from_center = 360 - edge
            print(f"Distance from Center = {dist_from_center}")
            if dist_from_center > -100:
                fly_dist = 20
            else:
                fly_dist = 0
    
    return fly_forward, fly_dist

class Control(Node):

    def __init__(self):
        super().__init__("control")

        
        # create subscribers
        self.sub_detect = self.create_subscription(Detection, "/detection", self.callback_detect, 10)
        self.sub_classify = self.create_subscription(Classify, "/classify", self.callback_class, 10)

        # create publisher
        self.pub_command = self.create_publisher(Command, "/flight_command", 10)

        # create service
        self.srv_avoid = self.create_service(Empty, "avoid", self.callback_avoid)

        self.box = None
        self.curr_box = None

        self.classify = None
        self.curr_class = None

        self.area = None
        self.curr_area = None

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.state = State.WAIT

    def timer_callback(self):
        """Call timer at 4 hz."""

        if self.state == State.FLY:
            self.get_logger().info("Ready to fly")
            
            if self.box is not None:
                self.curr_box = self.box
                self.curr_area = self.area
                self.curr_class = self.classify
                self.important_edge()

                msg_cmd = Command()
                msg_cmd.direction = self.classify

                forward, direction = control_cmds(self.area, self.edge, self.classify)
                msg_cmd.fly_dist = direction
                msg_cmd.fly_forward = forward
                
                self.get_logger().info("Publishing command")
                self.get_logger().info(f"Command: dir = {msg_cmd.direction}, dist = {msg_cmd.fly_dist}, forward = {msg_cmd.fly_forward}")
                self.pub_command.publish(msg_cmd)
                
                self.state = State.HOVER
        
        if self.state == State.HOVER:

            msg_cmd = Command()
            msg_cmd.direction = 0 # not a valid direction
            msg_cmd.fly_dist = 0
            msg_cmd.fly_forward = 0
            self.pub_command.publish(msg_cmd)

            self.state = State.WAIT

    def callback_detect(self, msg):
        self.box = msg.box
        self.area = abs(msg.box[2]-msg.box[0]) * abs(msg.box[3] - msg.box[1])

    def callback_class(self, msg):
        self.classify = msg.direction

    def callback_avoid(self, request, response):

        self.state = State.FLY
        
        return response
        
    def important_edge(self):

        # 0,1,2,3 (down,left,right,up)
        if self.classify == 0:
            self.edge = self.box[1]
        elif self.classify == 1:
            self.edge = self.box[0]
        elif self.classify == 2:
            self.edge = self.box[2]
        elif self.classify == 3:
            self.edge = self.box[3]


def main(args=None):
    """Call node function."""
    rclpy.init(args=args)
    controller = Control()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()