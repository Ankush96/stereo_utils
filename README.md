# stereo_utils

##### A package for calculating depth map

- Clone the camera_umd package (https://github.com/ktossell/camera_umd)
    ```sh
    $ git clone https://github.com/ktossell/camera_umd.git
    ```
- Update parameters in camera_umd/uvc_camera/launch/stereo_node.launch (Eg- Turn off Auto exposure, Auto focus and set left/device and right/device accordingly)
- 
    ```sh
    $ roslaunch uvc_camera stereo_node.launch
    ```
-
    ```sh
    $ rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.026 right:=stereo/right/image_raw left:=stereo/left/image_raw left_camera:=/left right_camera:=/right
    ```
    Here 9 and 6 are the number of internal nodes. For a 10 * 7 checkerboard pattern, it is 9*6 And if length of side of square is 26 mm, enter param as 0.026
-   ```sh
    $  ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
    ```

-
     ```sh
    $ rosrun stereo_utils/Get_Depth
    ```
    

