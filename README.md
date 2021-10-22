# Object detection on UAV

1. This program is developed for UAV and it depends on the ros platform. And it works with the sensors in Evarobot.
2. The sensors are depth-camera, monocamera, lidar and sonar.
3. This program can recognize objects around the robot. As result, it gives a list of objects, which 
4. contains all the Features of these objects.
5. Put this source code in the "\~/catkin\_ws/src" folder. And then change the path of the terminal to "~/catkin\_ws".
6. Enter the command "catkin_make" in the terminal.
7. Executable file is "~/catkin\_ws/devel/lib/evarobot\_adp/evarobot\_process\_node".
8. This file must execute with the gazebo-model of Evarobot and the Rviz.
9. For visualizition, there are boundary-boxes in the Rviz, that are detected. The topic of boundary-boxes is "/visualization\_marker"
10. All the sensors must be available. During the test, we find some computers have no data from lidar due to the GPU. If it happened, just turn off the process of lidar in the code. 
