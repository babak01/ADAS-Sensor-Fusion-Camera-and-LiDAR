This code is part of one of the projects in [Udacity sensor fusion nanodegree program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313). The goal of this project is to use sensor fusion technique to calculate the Time to Collision (TTC) with both camera and Lidar sensors. For this project we use the real-world data for both camera and Lidar.
<br>
<br>
### LiDAR
***
In order to compute the TTC for lidar, we need to find the distance to the closest Lidar point in the path of driving.
![Image1](https://github.com/babak01/LiDAR-Obstacle-Detection/issues/1#issue-941168789)

### Camera
***
For camera data we track cluster of keypoints between two sussecive frames to estimate the TTC. In this project, we tried different combinations of detectors and descriptors to find the top 3 combinations in terms of speed and accuracy.
<br>
<br>

For Detectors, we tested Shitomasi, Harris, FAST, BRISK, ORB, Akaze, SIFT. For descriptors, we tested BRISK, BRIEF, ORB, FREAK, AKAZE, and SIFT.
<br>
<br>

The list of top 3 combinations of detectors/descriptors is shown in below table.
<br>
<br>
<br>

| <b>TOP</b>      | <b>Number of keypoints</b> |<b>Detector time (msec)</b>      | <b>Descriptor time (msec)</b> |<b>Number of keypont matches</b>      |<b>Matching time (msec)</b> |
| ----------- | ----------- | ----------- | ----------- | ----------- | ----------- |
| <b>1st</b>      | Title       | Header      | Title       | Header      | Title       |
| <b>2nd</b>   | Text        | Paragraph   | Text        | Paragraph   | Text        |
| <b>3rd</b>   | Text        | Paragraph   | Text        | Paragraph   | Text        |