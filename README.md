# duckiebot_lane_trajectory

This project runs on ROS workspace.

![Duckiebot ROS Pipeline](https://github.com/techy-dpu/duckiebot_lane_trajectory/tree/main/resized_images/duckiebot_pipeline.jpg)


Running Instructions
----------------------
# Create ROS package
----------------------

1. Move to catkin work space in your machine

    $ cd ~/catkin_ws/src

2. catkin_create_pkg is the script used to create a new package. I want to create a package named 'lane_move' which depends on std_msgs, and rospy.

    $ catkin_create_pkg lane_move std_msgs rospy

3. Check the files created under the package

    $ ls lane_move

----------------------
# Add publisher code
----------------------

1. Change directory to the package source

    $ cd src/lane_move/src


2. copy the code move.py, lane_detection.py to the location 'src/lane_move/src'

    $ cp move.py lane_detection.py ./


3. Make the script executable

    $ chmod +x move.py

    $ chmod +x lane_detection.py


4. Move back to the catkin space

    $ cd ~/catkin_ws/src


5. Build the package

    $ catkin_make


6. Source the bash.

        $ source devel/setup.bash

----------------------
# Make the robot move
----------------------

Run the publisher code

    $ rosrun lane_move move.py


------------------------
# Rereferences
------------------------

https://docs.duckietown.org/daffy/opmanual_duckiebot/draft/lane_following_with_vehicles.html

http://wiki.ros.org/ROS/Tutorials

