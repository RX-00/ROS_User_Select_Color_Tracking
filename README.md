# ROS_User_Select_Color_Tracking
This repository is the package for a program for ROS that takes the camera feed from a remote camera (I'm using a raspberry pi camera on a turtlebot3) and sends it to the user pc in order for the user to select a ROI to color track and display the result on rviz.

DEPENDENCIES:

-I used OpenCV 3.2.0 and ROS Kinetic for these programs

-image_publisher.cpp was used to publish the camera feed while image_subscriber_cv.cpp was then used to receive the image msg from image_publisher and then the user would be able to select a ROI and a colored tracked image would then be published to the topic: "sensor_msg/Images"

to run, use the command:
rosrun ros_user_select_color_tracking [program name]
