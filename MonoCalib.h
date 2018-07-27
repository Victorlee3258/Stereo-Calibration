#pragma once

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "BaseData.h"

using namespace cv;
using namespace std;



namespace DualCalib
{
	class MonoCalib
	{
	public:
		MonoCalib(int boardWidth, int boardHeight, float squareSize);

		~MonoCalib();

		double ComputeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
			const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const Mat& cameraMatrix, const Mat& distCoeffs, vector<float>& perViewErrors);

		void CalcChessboardCorners(vector<Point3f>& corners);

		bool RunCalibration(vector<vector<Point2f> > imagePoints, Size imageSize,
			float aspectRatio, int flags, Mat& cameraMatrix, Mat& distCoeffs,
			vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr);

		void SaveCameraParams(const string& filename, Size imageSize,
			float aspectRatio, int flags, const Mat& cameraMatrix, const Mat& distCoeffs,
			const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs,
			const vector<vector<Point2f> >& imagePoints, double totalAvgErr);

		bool RunAndSave(const string& outputFilename,
			const vector<vector<Point2f> >& imagePoints, Size imageSize,
			float aspectRatio, int flags, Mat& cameraMatrix, Mat& distCoeffs, bool writeExtrinsics, bool writePoints);

		void MonoCalibration(std::vector<Mat>& Images, Mat& cameraMatrix, Mat& distCoeffs, Size imageSize, string OutputFilename);

	private:

		// parameters.
		enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

		double MaxReprojectionError;

		vector<Mat> mvImages;
		
		Mat cameraMatrix, distCoeffs;

		float aspectRatio = 1.f;
		int nframes = 10;

		const char* inputFilename = 0;

		// when change chessboard and phone camera, parameters below need to change.
		int mBoardWidth;
		int mBoardHeight;

		// chessboard square size.
		float mSquareSize;

		bool writeExtrinsics = true;
		bool writePoints = true;

		bool undistortImage = true;
		bool showUndistorted = false;

		int flags = 0;
		int delay = 1000;

		const int mMaxScale = 4;


	};

} // NAMESPACE DUALCALIB



