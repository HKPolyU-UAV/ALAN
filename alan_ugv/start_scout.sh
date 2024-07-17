source ~/scout_ws/devel/setup.sh
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
roslaunch alan_ugv scout_vicon.launch &
sleep 2
roslaunch alan_ugv px4_car.launch
