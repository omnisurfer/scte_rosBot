
#!/bin/sh
rosrun camera_calibration cameracalibrator.py \
--approximate 0.1 --size 8x6 --square 0.026 \
right:=/stereo/c270_right/image_raw right_camera:=/stereo/c270_right \
left:=/stereo/c270_left/image_raw left_camera:=/stereo/c270_left
