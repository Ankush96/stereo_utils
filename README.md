# stereo_utils
A package for calculating depth map 

1. git clone camera_umd
2. Update parameters in camera_umd/uvc_camera/launch/stereo_node.launch (Eg- Turn off Auto exposure, Auto focus and set left/device and right/device accordingly)
3.  roslaunch uvc_camera stereo_node.launch
4. Here 9*6 is the number of internal nodes. For a  10*7 checkerboard pattern, it is 9*6
And if length of side of square is 26 mm, enter param as 0.026

rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.026 right:=stereo/right/image_raw left:=stereo/left/image_raw left_camera:=/left right_camera:=/right

5. rosrun stereo_utils/Get_Depth
