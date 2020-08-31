# Visual Odometry

## Overview
Visual odometry is an algorithm which estimates camera motions from camera images. The goal of this project is to implement feature-based reconstruction on scene geometry, and analyze the path of a car from the driving record. In the beginning, we do image processing to recover image color and undistort the images. Next, we extract SURF features and find match points between current image and next image. After that, apply 8-point algorithm and RANSAC to the match points and estimate the camera Fundamental matrix. With the Fundamental matrix, we can reconstruct the camera translation and rotation, and then plot the trajectory on the 2D plane. Finally, we compare the result using self implemented method with the result using matlab library. (This project is implemented in matlab code and is only for self-practice purpose.)

## Dataset
This project uses OXFORD ROBOTCAR DATASET from Oxford's Robotics Institute.
<p align="center">
  <img src="https://github.com/ChienTeLee/visual_odometry/blob/master/doc/dataset.gif" width="50%" height="50%"> 
</p>

[Visual Odometry dataset (used in this project)](https://drive.google.com/drive/folders/1f2xHP_l8croofUL_G5RZKmJo2YE9spx9)

[OXFORD ROBOTCAR DATASET original website](https://robotcar-dataset.robots.ox.ac.uk/)

## How to run
1. Create a folder named "input"
2. Download [Visual Odometry dataset](https://drive.google.com/drive/folders/1f2xHP_l8croofUL_G5RZKmJo2YE9spx9) and unzip Oxford_dataset.zip.
3. put "model" and "stereo" folder in "input" folder
3. run VisualOdometry.m.
4. After running, an output figure containing the car route will appear in "output" folder.

## Implementation
1. Do image processing including recovering image color, undistorting the images for calibration, doing histogram equalization to ehnance contast, and applying gaussian filter for image smoothing.

2. Extract [SURF](https://en.wikipedia.org/wiki/Speeded_up_robust_features) features and find match points between current image and next image.
<p align="center">
  <img src="https://github.com/ChienTeLee/visual_odometry/blob/master/doc/fig0.png" width="100%" height="100%"> 
</p>

3. Apply [8-point algorithm](https://en.wikipedia.org/wiki/Eight-point_algorithm) and [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) to the match points for estimating the camera [Fundamental matrix](https://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision)).

4. From the Fundamental matrix, we can reconstrct 4 rotation and translation solutions. Check the image depth and feature point depth to find the only valid rotation and translation pair.

5. plot the camera motion path on 2D plane.

## Result
The red path is the visual odometry using self implemented methods. The blue path is the implementation using matlab library. We can tell that the car is driving a rectangle route, and the red and blue path matched pretty well.
<p align="center">
  <img src="https://github.com/ChienTeLee/visual_odometry/blob/master/output/output.jpg" width="50%" height="50%"> 
</p>

## Slides
[Slides](https://drive.google.com/file/d/1ppzgXlXXvlCBnEZOhR_cK2v2YY1CGe7P/view?usp=sharing)

## Reference
1. [Visual odometry (wikipeida)](https://en.wikipedia.org/wiki/Visual_odometry)

2. [detectSURFFeatures](https://www.mathworks.com/help/vision/ref/detectsurffeatures.html)

3. [KTH DD2429 Lecture 3 - Two view geometry & the Essential Matrix](https://kth.instructure.com/files/1241887/download?download_frd=1)

4. [NTU Digital Visual Effects Structure from motion](https://www.csie.ntu.edu.tw/~cyy/courses/vfx/16spring/lectures/handouts/lec10_sfm.pdf)

