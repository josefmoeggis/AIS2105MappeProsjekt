# AIS2105 Mappe Prosjekt
In this repository all code files and a videofile that shows the system running are uploaded.
When navigating the projects code, there are some important things to keep in mind:

## Steps to setup project
1. Go to the workspace/ and clone the repository
2. Confirm your usb connections. The camera usb port is configured in the launch file, while the usb->arduino serial connection is configured in "our_arduino_driver".
3. Make sure the usb connected to the arduino has permissions to read and write with this command `sudo chmod a+rw /dev/"yourUSBdevice"`
4. Run the launch file with: `ros2 launch py_launch launch.py`

The videofile is uploaded in the root of the repository named ais2105ProjectGroup5Video.mp4
