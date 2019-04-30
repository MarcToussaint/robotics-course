echo "Setting ENV..."

export ROS_MASTER_URI=http://thecount.local:11311
export DEV=`udevadm info -e | grep -oE -m 1 "/wl[[:alnum:]]*" | grep -oE "wl[[:alnum:]]*"`
export ROS_IP=`ifconfig $DEV | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`

echo ROS_HOSTNAME $ROS_HOSTNAME
echo ROS_MASTER_URI $ROS_MASTER_URI
echo WLAN device $DEV
echo ROS_IP $ROS_IP
echo "Done."
