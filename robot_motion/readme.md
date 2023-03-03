# Niryo Robot Python based Controlling

## Running Process
- Bringing Robot in Gazebo
    ```
    roslaunch niryo_robot_bringup desktop_gazebo_simulation.launch
    ```
- Set Group name to see it moving e.g group_name = arm or toll
    ```
    rosrun robot_motion a_move_group.py
    ```
- To visualize Camera feed
    ```
    rosrun robot_motion b_video_recording.py
    ```
- Sending Commands in pipline ( currently developing )
    ```
    rosrun robot_motion c_camera_processing_motion.py
    ```
## Installation Required
- sudo apt-get install ros-noetic-moveit*
- sudo apt-get install ros-noetic-rosbridge-server
- sudo apt-get install ros-noetic-tf2-web-republisher
- sudo apt-get install ros-noetic-joint-trajectory-controller
- sudo apt-get install ros-noetic-effort-controllers