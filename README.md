# TIAGo Robot Navigation with Pocketsphinx

The objective of this package is to integrate voice command with the laser-based localization and autonomous navigation with avoiding obstacles [3]. It is based on the simple ROS wrapper for using Pocketsphinx (via gstreamer) with ROS [2]. The commands implemented are the following:

Go to the kitchen.

Go to the living room.

Where is the bathroom?

Where is the bedroom?

Where is the porch?

About the code, more details can be found in [1]. For simulation in Gazebo, copy small_house.world to ~/tiago_public_ws/src/pal_gazebo_worlds/worlds. After that, modify the tiago_mapping.launch and tiago_navigation.launch codes in ~/tiago_public_ws/src/tiago_simulation/tiago_2dnav_gazebo/launch:

name="world"  default="small_house"

name="gzpose" default="-x -4.0  -y   0.0 -z  0.0   -R 0.0 -P 0.0 -Y  0.0"

To map the small_house.world, create the map with gmapping, save in ~/.pal/tiago_maps/config and copy to ~/.pal/tiago_maps/configurations/small_house.

### References
[1] Goebel, R. P. (2015). ROS by example. Lulu. com.

[2] Pocketsphinx

http://wiki.ros.org/pocketsphinx

https://github.com/mikeferguson/pocketsphinx

[3] Mapping, localization and path planning

http://wiki.ros.org/Robots/TIAGo/Tutorials/Navigation/Mapping

http://wiki.ros.org/Robots/TIAGo/Tutorials/Navigation/Localization
