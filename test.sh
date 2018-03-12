if echo $ROS_PACKAGE_PATH | grep -q `pwd`/3rd-party-libs ; then
	echo "Its There!"
else
	echo "Not!"
fi
