This code is part of one of the projects in [Udacity sensor fusion nanodegree program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313). The goal of this project is to use sensor fusion technique to calculate the Time to Collision (TTC) with both camera and Lidar sensors. For this project we use the real-world data for both camera and Lidar.
<br>
<br>

![Project2_lidarCamera_final_gif](https://user-images.githubusercontent.com/54375769/125152635-7c3e0680-e113-11eb-8090-9497aeca3cff.gif)

<br>
<br>

### LiDAR
***

<br>
In order to compute the TTC for lidar, we need to find the distance to the closest Lidar point in the path of driving.
<br>
<br>

![image1](https://user-images.githubusercontent.com/54375769/125152576-edc98500-e112-11eb-836e-dcb0f33dd316.jpg)

<br>
<br>

### Camera
***
For camera data we track cluster of keypoints between two sussecive frames to estimate the TTC. In this project, we tried different combinations of detectors and descriptors to find the top 3 combinations in terms of speed and accuracy.
<br>

For Detectors, we tested Shitomasi, Harris, FAST, BRISK, ORB, Akaze, SIFT. For descriptors, we tested BRISK, BRIEF, ORB, FREAK, AKAZE, and SIFT.
<br>

The list of top 3 combinations of detectors/descriptors is shown in below table.
<br>
<br>


| <b>TOP</b>      | <b>Number of keypoints</b> |<b>Detector time (msec)</b>      | <b>Descriptor time (msec)</b> |<b>Number of keypont matches</b>      |<b>Matching time (msec)</b> |
| ----------- | ----------- | ----------- | ----------- | ----------- | ----------- |
| <b>1st</b>      | FAST       | FAST/BRIEF      | HARRIS/BRIEF       | FAST/SIFT      | HARRIS/BRIEF       |
| <b>2nd</b>   | BRISK        | FAST/ORB   | HARRIS/ORB        | FAST/BRIEF   | HARRIS/ORB        |
| <b>3rd</b>   | SIFT        | FAST/SHITOMASI   | HARRIS/BRISK        | FAST/ORB   | HARRIS/SIFT        |
