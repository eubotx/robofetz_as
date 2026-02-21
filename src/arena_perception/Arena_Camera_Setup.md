# Arena Camera Setup Manual

This guide helps you permanently identify your Arena camera, configure device access, calibrate the camera for ROS2, and adjust V4L2 parameters.

---

## Identify Your Camera

**List Connected USB Devices**

```bash
lsusb
```

Look for your camera in the output. Example:

```
Bus 001 Device 003: ID 32e4:0144
```

* Vendor ID: `32e4`
* Product ID: `0144`

**List Video Devices**

```bash
v4l2-ctl --list-devices
```

**Check Supported Formats**

```bash
for dev in /dev/video*; do
  echo "=== $dev ==="
  v4l2-ctl --device=$dev --list-formats 2>/dev/null || echo "No formats"
done
```

---

## Permanent Device Naming via UDEV

To make your camera always appear at the same path, create a UDEV rule.

**Create UDEV Rule File**

```bash
sudo nano /etc/udev/rules.d/99-arena-camera.rules
```

**Add Rule (replace with your IDs)**

```udev
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="0144", SYMLINK+="arena_camera", MODE="0666"
```

**Optional: Multiple Cameras (by serial number)**

```udev
SUBSYSTEM=="video4linux", ATTRS{serial}=="ABC123", SYMLINK+="arena_camera_left"
SUBSYSTEM=="video4linux", ATTRS{serial}=="DEF456", SYMLINK+="arena_camera_right"
```

**Apply Rules**

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# Unplug and replug your camera
```

**Verify**

```bash
ls -la /dev/arena_camera
v4l2-ctl --device=/dev/arena_camera --list-formats
```

**Benefits:**

* Same device path regardless of USB port
* No code changes required
* Works across reboots

After UDEV setup, your YAML file can reference the camera permanently:

```yaml
video_device: "/dev/arena_camera"
```

---

## ROS2 Camera Calibration

**Calibration Command**

```bash
ros2 run camera_calibration cameracalibrator \
    --size=8x5 \
    --square=0.0254 \
    --no-service-check \
    --fisheye-k-coefficients=4 \
    --ros-args \
    -r image:=/arena_camera/image \
    -p camera:=/arena_camera
```

---

## Adjusting Camera Parameters with Camset

**Install Dependencies**

```bash
sudo apt-get install python3 python3-pip v4l-utils pkg-config
pip3 install camset --break-system-packages
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

**Use Camset**

```bash
camset  # Opens interactive interface
```

---

## Quick Reference Commands

* List video devices:

```bash
ls -l /dev/video*
```

* Check camera info:

```bash
v4l2-ctl --device=/dev/video0 --info
v4l2-ctl --device=/dev/video1 --info
```

* List controls for a device:

```bash
v4l2-ctl --device=/dev/video6 --list-ctrls
```