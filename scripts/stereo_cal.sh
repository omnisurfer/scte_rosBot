
#!/bin/sh
rosrun camera_calibration cameracalibrator.py \
<<<<<<< HEAD
--approximate 0.1 --size 8x6 --square 0.025 \
=======
--approximate 0.1 --size 8x6 --square 0.026 \
>>>>>>> 92acb0863bfb5eb1be56f5a16bbd49aafcdbd498
right:=/stereo/c270_right/image_raw right_camera:=/stereo/c270_right \
left:=/stereo/c270_left/image_raw left_camera:=/stereo/c270_left
