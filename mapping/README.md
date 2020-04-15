The Kalman Filter is implemented for robot localization. The package used is robot_localization. To get the filtered odometry message follow the given steps:-

1. Run the uwsim simulation using $rosrun uwsim uwsim
2. Run the odom-message-generator.py node by the command $rosrun my_odom odom-message-generator.py
3. Run the robot localization ekf node to start the ekf node. $ roslaunch robot_localization/launch/ekf_template.launch

The filtered odometry message would be published on the topic /filtered/odometry
