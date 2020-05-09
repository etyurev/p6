
Install gazebo_ros_package  http://wiki.ros.org/gazebo_ros_pkgs 
Copy to the folder ~/.gazebo/models/ folders ce_jacket and realsense_camera
roslaunch gazebo_ros empty_world.launch
Disable gravity in the Gazebo GUI
Insert models realsense_camera and ce_jacket 
Manually place ce_jacket at [0,0,0,1.56,0,0] and realsense_camera to one of the poses:
[0.6;2.5;3.2;0;0.51;-1.56]
[-2.5;-0.5;3.2;0;0.46;0]
[1;-3.8;3.2;0;0.50;1.56]
[4.8;-0.5;3.2;0;0.5;3.15]
In file ROS_Gazebo_RealSense_CE_Jacket_040520.py specify the path to folder for saving the point clouds
Run python2 /path_to_the_program/ROS_Gazebo_RealSense_CE_Jacket_040520.py
Pres Space wnen you need to capture the t-joint 
