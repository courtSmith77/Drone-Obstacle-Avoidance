# Obstacle Course Traversing Drone

This project contains a ROS2 package for drone navigation in addition to  creating custom YOLOv8 detection and classification models.

## fly_drone
`fly_drone` is the ROS2 package that handles all communication with the drone as well as deploying the YOLOv8 models for intelligent navigation. Check out its README.md for more information.

## model_dev
`model_dev` details the necessary data annotation and structure required for training a model on a custom dataset. It also includes all files to train and utilize the developed models. Check out its README.md for more information.

## testing
`testing` includes scripts for testing your models both offline and online. Check out its README.md for more information.