# fly_drone

A ROS node that utilizes the Tello drone and associated SDK to navigate the drone through a series of obstacles. Several Ultralytics YOLOv8 models were trained to detect and classify symbols for drone control.

## How to run
1. run the drone node and launch rviz by running `ros2 launch fly_drone visual.launch.xml`
3. To start the drone run `ros2 service call /takeoff std_srvs/srv/Empty`
4. Once the drone reaches the end of the course and is close to the star symbol, the command line in the additional window will prompt you if it is safe to flip. Enter 'y' if yes and 'n' if no.
5. To land the drone run `ros2 service call /land std_srvs/srv/Empty`

## Node
`drone` - communicates the commands to the drone and feeds the live image to the correct detection and classification models

### Publisher
`/image` sensor_msgs/msg/Image - publishes the live feed image to be viewed in RVIZ via the image topic

### Services
- `takeoff` std_srvs/srv/Empty - commands the drone to takeoff and begin
- `land` std_srvs/srv/Empty - commands the drone to land
- `stop` std_srvs/srv/Empty - commands the drone to stop flying and hover
- `celebrate` std_srvs/srv/Empty - commands the drone to do a back flip
- `rotate` std_srvs/srv/Empty - commands the drone to rotate 180 degrees
- `forward` std_srvs/srv/Empty - commands the drone to move forward 40 cm

## Modules
`helper.py` - contains helper function for image cropping, model deployment, and control commands


