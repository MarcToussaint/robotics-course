# Baxter Start-up and Shutdown

## Booting

On the back of the robot near the pedestal base, there is a power button. Push it and wait for the machine to finish booting.

## Communicating with Baxter

Connect to the lab mlr-robolab WLAN (password: mlr-robolab)

Baxter runs with ROS, and you'll need to set your environment
variables to enable ROS communication. The easiest way to do this is
to connect to run one of following scripts in Terminal from the mlr
folder:

'''
source bin/baxterwlansetup.sh
'''

## Start-up

Call the start-up script, which enables baxter, untucks the arms,
turns off the ultrasonics (they click very loudly in any videos), and
calibrates the grippers.

'''
bin/baxterStart.sh
'''

## Shutdown

Always tuck the arms before shutting down, to keep the spring wear to a minimum.
'''
bin/baxterTuck.sh
'''
or
'''
rosrun baxter_tools tuck_arms.py -t
'''

Then press the power button once to turn the robot off.

Alternatively, you can ssh in to the robot and run:
sudo shutdown -h now

