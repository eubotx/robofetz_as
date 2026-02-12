# List all video devices
ls -l /dev/video*

# Check camera info
sudo apt install v4l-utils
v4l2-ctl --list-devices

# See detailed info about each camera
v4l2-ctl --device=/dev/video0 --info
v4l2-ctl --device=/dev/video1 --info



v4l2-ctl --device=/dev/video6 --list-ctrls