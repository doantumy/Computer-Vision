# [Computer Vision] 3D Reconstruction using Kinect V2

## Purpose
The objective of the last practical session is to perform 3D reconstruction using a pair of Kinect V2 camera.

## Task
* You have been familiarized with Kinect V2 and the technology behind it.
* We acquired data from Microsoft Kinect V2 for Windows using MATLAB and you experienced how to interact with the RGB camera and 3D depth sensor.
* For this project, you will be given a series of color and depth image pairs (most of them were acquired for calibration purpose, the last pair corresponds to the scene to reconstruct using a point cloud).
* A point cloud (in the 3D world reference frame) is a way to represent the external surface of an object.

## Submission
Your work is now to complete the following tasks:
1. Calibrate the two RGB cameras using the checkerboard calibration technique.
* Detect the corner points of the checkerboard
* Calibrate each camera separately (to get intrinsic parameters)
* Calibrate one camera with respect of each other(to get extrinsic parameters)

2. Calibrate the two depth cameras with respect of RGB data using the knowledge that you have concerning the transformation that relies a RGB camera to a depth camera in a Kinect setup (see Kinect Parameters.m)

3.	Perform 3D reconstruction
* Back project the depth computed from Kinect disparity to real world coordinate using depth camera calibration parameter. 
* Associate the real world coordinate information (obtained from back projected depth) with the color camera using the transformation between color and depth camera. 
* Project the transformed information (from previous step) to the color camera.

4.	Write a report summarizing: (a) the methods you implemented, (b) the problems you faced and the solutions tested to solve these problems, (c) the results you obtained and how you checked the accuracy of your results, (d) discuss and comment your work. Submit also your code and resulting images.

## Material and References 
* Microsoft Kinect V2 & Windows adapter for PC. 
* Acquisition Using Kinect for Windows Hardware http://es.mathworks.com/help/imaq/acquisition-using-kinect-for-windows-hardware.html?requestedDomain=www.mathworks.com 
* MATLAB and Simulink http://es.mathworks.com/hardware-support/kinect-windows.html?requestedDomain=www.mathworks.com
* Simulink Support for Kinect http://www.mathworks.com/matlabcentral/fileexchange/32318-simulink-support-for-kinect 
* Kinect 2 Interface for Matlab (http://www.mathworks.com/matlabcentral/fileexchange/53439-kinect-2-interface-for-matlab) 





