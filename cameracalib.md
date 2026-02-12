calibrate sim pinhole cam
checken projection matrix
dann sollten koordinaten stimmen



ros2 run camera_calibration cameracalibrator   --size=9x6   --square=0.024   --no-service-check   --fisheye-k-coefficients=4   --ros-args   -r image:=/arena_camera/image_raw   -p camera:=/arena_camera


**** Calibrating ****
mono pinhole calibration...
D = [-0.00042255744336957136, 0.00022077920508481891, 4.288532665789081e-06, 6.138631719452745e-05, 0.0]
K = [355.5533027380523, 0.0, 399.3649797479707, 0.0, 355.56152170100745, 299.54572370250645, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [355.1217956542969, 0.0, 398.96464385774016, 0.0, 0.0, 355.02838134765625, 299.0519589223368, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
800

height
600

[narrow_stereo]

camera matrix
355.553303 0.000000 399.364980
0.000000 355.561522 299.545724
0.000000 0.000000 1.000000

distortion
-0.000423 0.000221 0.000004 0.000061 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
355.121796 0.000000 398.964644 0.000000
0.000000 355.028381 299.051959 0.000000
0.000000 0.000000 1.000000 0.000000
