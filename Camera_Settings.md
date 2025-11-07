# Method 1: Default (video0) - just run
ros2 launch detection camera.launch.py

# Method 2: Environment variable (set once, works forever)
export ARENA_CAMERA_DEVICE=/dev/video2
ros2 launch detection camera.launch.py

# Method 3: Command line (overrides method 1 & 2)
ros2 launch detection camera.launch.py video_device:=/dev/video2



This now also works with ARENA_CAMERA_CONFIG to give the config name to use!!!



____________________


## The Problem with Environment Variables Here

Environment variables are **static** - they don't automatically update when you plug into a different USB port. You'd have to manually update them every time.

## Best Solution: **UDEV Rules + Symlinks**

This is the professional approach that avoids both rebuilding AND manual configuration.

### How It Works:

1. **Tell Linux**: "No matter which USB port I use, always call my camera `arena_camera`"
2. **System automatically creates**: `/dev/arena_camera` that always points to your actual camera
3. **Your code always uses** the same path: `/dev/arena_camera`
4. **Add environment variable** ARENA_CAMERA_DEVICE=/dev/arena_camera

### Step-by-Step Implementation:

#### 1. Identify Your Camera Permanently
```bash
# Plug in your camera
lsusb
# Find your camera in the list, note the vendor:product ID
# Example: Bus 001 Device 003: ID 046d:0825 Logitech, Inc. Webcam C270
```

#### 2. Create UDEV Rule
Create file: `/etc/udev/rules.d/99-arena-camera.rules`
```bash
sudo nano /etc/udev/rules.d/99-arena-camera.rules
```

Add this content (replace with your vendor:product ID):
```
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", SYMLINK+="arena_camera", MODE="0666"
```

#### 3. Apply the Changes
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# Unplug and replug your camera
```

#### 4. Verify It Works
```bash
ls -la /dev/arena_camera
# Should show: /dev/arena_camera -> video2 (or whatever)
```

#### 5. Update Your YAML File
```yaml
video_device: "/dev/arena_camera"  # Now it's permanent!
```

## What This Gives You:

- ✅ **Plug into any USB port** → always appears as `/dev/arena_camera`
- ✅ **No code changes ever** - same path always works
- ✅ **No rebuilding** - it's a system-level fix
- ✅ **No environment variables to manage**
- ✅ **Works immediately** for all users on the system

## Alternative: Camera Serial Number Approach

If you have multiple identical cameras:

```bash
# List cameras with serial numbers
udevadm info --query=property /dev/video0 | grep SERIAL
```

Then in your UDEV rule:
```
SUBSYSTEM=="video4linux", ATTRS{serial}=="ABC123", SYMLINK+="arena_camera_left"
SUBSYSTEM=="video4linux", ATTRS{serial}=="DEF456", SYMLINK+="arena_camera_right"
```

## Why This Beats Everything Else:

| Solution | USB Port Changes | Rebuild Needed | User Setup |
|----------|------------------|----------------|------------|
| Environment Variables | ❌ Manual update | ❌ | Medium |
| Command-line Args | ❌ Type every time | ❌ | Annoying |
| UDEV Rules | ✅ Automatic | ✅ None | **Set once, forever** |

## Real-World Workflow:

**Before:**
1. Plug camera into different USB port
2. Run `ls /dev/video*` to find new path  
3. Update environment variable or command-line arg
4. Hope you remember next time

**After:**
1. Plug camera into any USB port
2. Run your code → **it just works**
3. That's it.

## For Multiple Users:

If different users have different cameras, combine with environment variables:

```yaml
video_device: "/dev/arena_camera"  # UDEV rule for consistency
```

Or if users need different cameras:

```bash
# User A's UDEV rule: 
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1234", SYMLINK+="user_a_camera"

# User B's UDEV rule:
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="5678", SYMLINK+="user_b_camera"
```

## Bottom Line:

**UDEV rules are the professional solution** for hardware that moves between USB ports. They turn a dynamic, frustrating problem into a rock-solid, set-once-forget-forever solution.

This is how industrial systems, robotics labs, and production environments handle USB devices - because professionals can't be chasing device paths every time they replug a cable!