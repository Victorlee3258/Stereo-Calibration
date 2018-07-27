#pragma once

/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
Oct. 3, 2008
Right to use this code in any way you want without warranty, support or any guarantee of it working.

************************************************** */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include "Undistortion.h"
#include "BaseData.h"
#include "../mathlib/Matrix.h"
#include "MonoCalib.h"
#include "DLTAlgorithm.h"
#include "UnitTest.h"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <memory>
#include <numeric>
#include <functional>

using namespace cv;
using namespace std;

namespace DualCalib
{
	// Board parameter.
	class DualCalibration
	{
	public:
		DualCalibration(vector<string>& ImageList);

		~DualCalibration();

		Vec3f RotationMatrixToEulerAngles(Mat R);

		//void FeatureMatch(vector<Mat> &imgs);

		void DistortPixel(const Mat& K, const Mat& R, const Mat& P, const Mat& D, const cv::Point2f& PointsLocation, cv::Point2d& DistortPointsLocation);

		// run dual calibration.
		void RunDualCalibration();

		// convert opencv matrix to CMatrix.
		CMatrix ConvertMat2CMatrix(cv::Mat& cvMatrix);

		float CalcAB(vector<float>& PointsX, vector<float>& PointsY);

		void LinearFitting(vector<float> x, vector<float> y, double& slope);

		// process for image data, like divide image with four chessboard to four small images.
		void ImageProcessFourChessboard(const vector<string>& imagelist, vector<Mat>&images);

		// process for image data.
		void ImageProcessOneChessboard(const vector<string>& imagelist, vector<Mat>&images);

		// dual calibration.
		void StereoCalib(vector<Mat>& imgs, Size boardSize);

		// image undistort.
		void ImageUndistortion(Mat cameraMatrix[2], Mat distCoeffs[2], Mat& P1, Mat& P2, Mat& R, Mat& R1, Mat& R2, Size imageSize, Size imageSize2, Size BoardSize, int nimages, vector<Mat> goodImageList, Rect validRoi[2]);

	public:
		vector<Mat> mvImgs, mvOriginImages;

		vector<string> mvImageList;

		bool mUseCalibrate, mShowRectified;

		vector<vector<Point2f>> mvImageCorners[2];

		double mdCy1, mdCy2;

		// camera matrix for init, get from mono camera calibration.
		Mat mK1, mK2, mD1, mD2;

		cv::Size mBoardSize;

	public:
		// parameters init file.
		// read from init-parameters file.

		// when change chessboard and phone camera, parameters below need to change.
		int mnBoardWidth;
		int mnBoardHeight;

		// main image size.
		float mfMainImageWidth;
		float mfMainImageHeight;

		// second image size.
		float mfSecondImageWidth;
		float mfSecondImageHeight;

		// chessboard square size.
		float mfSquareSize;

		// display detected corners in the image.
		bool mbDisplayCorners;//true;

		// image scale to find image corners.
		int mnMaxScale;

		// option for fix K1 or not ( 0 or CV_CALIB_FIX_K1)
		int mnFlags = 0;

		// this parameter is false.
		bool mbUseUncalibrated;

		// show rectified image for debug.
		bool mbShowRectified;

		//------------- parameters for mono calibration--------------
		bool mbWriteExtrinsics, mbWritePoints, mbUndistortImage, mbShowUndistorted;

		std::string msMainCameraOutputFile, msSecondCameraOutputFile;

		vector<Mat> mvOnlineImages;

	private:

		// for crop from origin images.
		int mnFinalImageWidth;
		int mnFinalImageHeight;

		//Size mImageSize = cv::Size(4224, 3136);		
		Size mImageSize1, mImageSize2;

		float mfScale;

		double mdAFFocalLenDivideFF;

		int mnImageWidth, mnImageHeight;

		// four chessboard in one image or not.
		bool mbIsFourChessboard;

		// parameters for divide the image to four small part.
		const int mnCutRows = 2;
		const int mnCutCols = 2;

		double mdSlope;		

	};


}// NAMESPACE DUALCALIB
