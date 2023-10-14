# Square_Detection - Version 1.0
To use the square_detection package, add it to your catkin workspace and run
```bash

catkin build

```
Then, run the sky_sim simulation along with the correct billboard world.
```bash

rosrun square_detection blue_square_detection_node

```
# Conclusions 

We do not use this code in the competition. This is because it gave many false positives. We believe that with good calibration and application of a better threshold, the code could be more efficient. However, we decided to use YOLO V5 to detect the blue square from a synthetic dataset, just as was done in phase 4 with people detection. As phase 2 processing would be offboard, using YOLO V5, despite seeming like a very complicated tool for a simple task, proved to be an option with much greater accuracy than using OpenCV with color and shape filters. Anyway, it was a good way to learn how to deal with ROS packages and catkin workspaces.
