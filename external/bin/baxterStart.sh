echo "Enabling joints"
rosrun baxter_tools enable_robot.py -e

echo "Untucking arms"
rosrun baxter_tools tuck_arms.py -u

echo "Calibrating grippers"
rostopic pub -1 /robot/end_effector/left_gripper/command baxter_core_msgs/EndEffectorCommand '{ id :  65538,  command : calibrate, args : "{ }", sender : "foo", sequence: 1 }'
rostopic pub -1 /robot/end_effector/right_gripper/command baxter_core_msgs/EndEffectorCommand '{ id :  65538,  command : calibrate, args : "{ }", sender : "foo", sequence: 1 }'

echo "Turning off sonar"
rostopic pub -1 /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0

echo "Enabling left hand camera"
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800

echo "Done."
