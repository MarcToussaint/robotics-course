mkdir -p src
cd src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update
source /opt/ros/kinetic/setup.bash
cd ..
catkin_make
catkin_make install
