# Integrating-Open-cv-with-ROS-2
This package shows an integration of opencv (cv_bridge) with ROS 2 ,also face-dectection using ROS 2.
# OPENCV ROS2 (FACE DETECTION).

This package helps you integrate Opencv with ROS 2 and also face-detection.
Install all dependecies:
```
   sudo apt install ros-<ros2-distro>-cv-bridge
```  
using the pip command:
```
   pip install opencv-python
```
NOTE: python3 has to be installed before this nodes can work.

To run this Node.

Create a workspace and clone the package into your src folder.

Then:
```
   ros2 run ros2_cv face_detection 
```

Open rviz and visualize the image from your webcam.
```
   ros2 run rviz2 rviz2 
```  
set the image topic to "/webcam" 
You will see as Video displayed below:

  
[Screencast from 03-16-2023 12:54:55 AM.webm](https://user-images.githubusercontent.com/97457075/225474915-bcefc1c0-0e42-40a6-988c-16b32089fa94.webm)

You can integrate this package in simulation and also can be used in a pysical robot with Opencv.



[Screencast from 03-17-2023 02:15:13 AM.webm](https://user-images.githubusercontent.com/97457075/225787575-08a740c3-6f32-426f-b019-a367211019d9.webm)

[Screencast from 03-17-2023 08:20:00 AM.webm](https://user-images.githubusercontent.com/97457075/225844009-1fcbccce-c65d-4a09-836b-0634536294e8.webm)


