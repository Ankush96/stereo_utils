# stereo_utils

##### A package for calculating depth map

1. Clone the [camera_umd package](https://github.com/ktossell/camera_umd) <br>
    ```
    $ git clone https://github.com/ktossell/camera_umd.git
    ```
2. Update parameters in camera_umd/uvc_camera/launch/stereo_node.launch (Eg- Turn off Auto exposure, Auto focus and set left/device and right/device accordingly)

3. Turn on the camera drivers
   ```
   $ roslaunch uvc_camera stereo_node.launch
   ```
   
4. Calibration step
   ```sh
   $ rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.026 right:=stereo/right/image_raw left:=stereo/left/image_raw left_camera:=/left right_camera:=/right
   ```
    Here 9 and 6 are the number of internal nodes. For a 10 * 7 checkerboard pattern, it is 9*6, and if length of side of square is 26 mm, enter param as 0.026
    
5. Calculate disparity and publish it
    ```sh
    $  ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
    ```

6. Run the code to get the depth of an object filtered by HSV color filter
   ```sh
   $ rosrun stereo_utils/Get_Depth
   ```
    

