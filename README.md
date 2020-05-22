# Collaborative-Drone-System-Node

download px4 fw
change branch to version 1.8
build using  make posix_sitl_default gazebo

create catkin_ws
initialize workspace
clone this repo into src folder

clone https://github.com/OBSchofieldUK/RM-ICS20 into src folder
run installation of simulation assets
make copy of drone model for the second drone and change mavros udp port 

px4_launch_gazebo.sh needs to have correct Firmware path, which can be given as the first parameter when running it, or can be modified in the file

run px4_launch_gazebo.sh