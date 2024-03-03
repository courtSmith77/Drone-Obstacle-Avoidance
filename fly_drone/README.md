# fly_drone

This project utilizes the Tello drone and associated SDK to navigate the drone through a series of obstacles. Several Ultralytics YOLOv8 models were trained to detect and classify symbols for drone control.

## How to run
1. run the drone node by running `ros2 run fly_drone drone --ros-args -p model_detect_path:='file_path' model_classify_arrow_path:='file_path' model_classify_symbol_path:='file_path'`
2. launch rviz with `ros2 run rviz2 rviz2 -d 'config_file'`to view the live drone footage
3. To start the drone run `ros2 service call /takeoff std_srvs/srv/Empty`
4. Once the drone reaches the end of the course and is close to the star symbol, the command line will prompt you if it is safe to flip. Submit enter 'y' if yes and 'n' if no.
5. To land the drone run `ros2 service call /land std_srvs/srv/Empty`

## Node
`drone` - communicates the commands to the drone and feeds the live image to the correct detection and classification models

### Publishers
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


