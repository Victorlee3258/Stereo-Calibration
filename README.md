# calibration
# DualCameraCalibration.
Dual camera calibration for mobile phone.

# structure of the code.
1. use mono camera calibration to compute main/second camera intrinsic parameters;
2. use intrinsic parameters get from mono calibration for dual calibration to get better intrinsic/extrinsic parameters;
3. use camera parameters to undistort.

# usage of this code.
1. parameters need to init:
	
	all parameters write in InitParameters.yaml file, need to change parameters for differenet cameras.


2. data type for process:
	# image data get from the .xml file;

# change log:
1. (1) add and test mono calibration over, for now get better result than only use dual calibration. --- 2018.06.11

2. (1) downscale image size to speed up corners detection for large image size type;
   (2) add a init file which save all parameters for different camera type；						     --- 2018.06.13
  （3）add code to test undistort image pixel distance with different distort coeffices;				 --- 2018.06.21

3. remove resize main and second images size code.									                 --- 2018.07.10
# note:
