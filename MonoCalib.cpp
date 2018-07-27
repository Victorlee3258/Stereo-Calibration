#include "MonoCalib.h"

namespace DualCalib
{
	MonoCalib::MonoCalib(int boardWidth, int boardHeight, float squareSize):mBoardWidth(boardWidth), mBoardHeight(boardHeight), mSquareSize(squareSize)
	{
		// copy image data to member variables.

	}

	double MonoCalib::ComputeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
		const vector<vector<Point2f> >& imagePoints,const vector<Mat>& rvecs, const vector<Mat>& tvecs,
		const Mat& cameraMatrix, const Mat& distCoeffs,vector<float>& perViewErrors)
	{
		vector<Point2f> imagePoints2;
		int i, totalPoints = 0;
		double totalErr = 0, err;
		perViewErrors.resize(objectPoints.size());


		vector<double> vPerPixelPositionErrors;

		for (i = 0; i < (int)objectPoints.size(); i++)
		{
			projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
				cameraMatrix, distCoeffs, imagePoints2);

			// per pixel position error.
			for (size_t k = 0; k < imagePoints2.size(); k++)
			{
				double PerPixelPositionErr = sqrt((imagePoints2[k].x - imagePoints[i][k].x) * (imagePoints2[k].x - imagePoints[i][k].x) +
												  (imagePoints2[k].y - imagePoints[i][k].y) * (imagePoints2[k].y - imagePoints[i][k].y));
			
				vPerPixelPositionErrors.push_back(PerPixelPositionErr);
			}

			err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);		
			int n = (int)objectPoints[i].size();

			perViewErrors[i] = (float)std::sqrt(err*err / n);
			totalErr += err*err;
			totalPoints += n;
		}
		MaxReprojectionError = *max_element(vPerPixelPositionErrors.begin(), vPerPixelPositionErrors.end());
		std::cout << "单摄最大重投影误差:" << MaxReprojectionError << std::endl;

		return std::sqrt(totalErr / totalPoints);
	}

	void MonoCalib::CalcChessboardCorners(vector<Point3f>& corners)
	{
		corners.resize(0);

		for( int i = 0; i < mBoardHeight; i++ )
		    for( int j = 0; j < mBoardWidth; j++ )
		        corners.push_back(Point3f(float(j * mSquareSize),
		                                  float(i * mSquareSize), 0));
	}

	bool MonoCalib::RunCalibration(vector<vector<Point2f> > imagePoints, Size imageSize,
								float aspectRatio,int flags, Mat& cameraMatrix, Mat& distCoeffs,
								vector<Mat>& rvecs, vector<Mat>& tvecs,vector<float>& reprojErrs,double& totalAvgErr)
	{
		cameraMatrix = Mat::eye(3, 3, CV_64F);

		if (flags & CALIB_FIX_ASPECT_RATIO)
			cameraMatrix.at<double>(0, 0) = aspectRatio;

		distCoeffs = Mat::zeros(8, 1, CV_64F);

		vector<vector<Point3f> > objectPoints(1);
		CalcChessboardCorners(objectPoints[0]);

		objectPoints.resize(imagePoints.size(), objectPoints[0]);
		
		double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs, flags | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3
		|CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_ZERO_TANGENT_DIST);

		printf("单摄RMS值: %g\n", rms);

		bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

		totalAvgErr = ComputeReprojectionErrors(objectPoints, imagePoints,
			rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

		return ok;
	}


	void MonoCalib::SaveCameraParams(const string& filename, Size imageSize,
									float aspectRatio, int flags,const Mat& cameraMatrix, const Mat& distCoeffs,
									const vector<Mat>& rvecs, const vector<Mat>& tvecs,const vector<float>& reprojErrs,
									const vector<vector<Point2f> >& imagePoints,double totalAvgErr)
	{
		FileStorage fs(filename, FileStorage::WRITE);

		time_t tt;
		time(&tt);

		struct tm *t2 = localtime(&tt);
		char buf[1024];
		strftime(buf, sizeof(buf) - 1, "%c", t2);

		fs << "CalibrationTime" << buf;

		if (!rvecs.empty() || !reprojErrs.empty())
			fs << "NumFrames" << (int)std::max(rvecs.size(), reprojErrs.size());

		if (flags & CALIB_FIX_ASPECT_RATIO)
			fs << "AspectRatio" << aspectRatio;

		if (flags != 0)
		{
			sprintf(buf, "flags: %s%s%s%s",
				flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
				flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
				flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
				flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
			//cvWriteComment( *fs, buf, 0 );
		}		
		fs << "Flags" << flags;

		fs << "CameraMatrix" << cameraMatrix;
		fs << "DistortionCoefficients" << distCoeffs;

		fs << "AvgReprojectionError" << totalAvgErr;
		fs << "MaxReprojectionError" << MaxReprojectionError;

		if (!reprojErrs.empty())
			fs << "PerViewAverageReprojectionErrors" << Mat(reprojErrs);

		if (!rvecs.empty() && !tvecs.empty())
		{
			CV_Assert(rvecs[0].type() == tvecs[0].type());
			Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());

			for (int i = 0; i < (int)rvecs.size(); i++)
			{
				Mat r = bigmat(Range(i, i + 1), Range(0, 3));
				Mat t = bigmat(Range(i, i + 1), Range(3, 6));

				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				r = rvecs[i].t();
				t = tvecs[i].t();
			}
			fs << "ExtrinsicParameters" << bigmat;
		}

		if (!imagePoints.empty())
		{
			Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
			for (int i = 0; i < (int)imagePoints.size(); i++)
			{
				Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
				Mat imgpti(imagePoints[i]);
				imgpti.copyTo(r);
			}
			fs << "ImagePoints" << imagePtMat;
		}
	}


	bool MonoCalib::RunAndSave(const string& outputFilename,const vector<vector<Point2f> >& imagePoints,
								Size imageSize,float aspectRatio, int flags,
								Mat& cameraMatrix,Mat& distCoeffs, bool writeExtrinsics, bool writePoints)
	{
		vector<Mat> rvecs, tvecs;
		vector<float> reprojErrs;
		double totalAvgErr = 0;

		bool ok = RunCalibration(imagePoints, imageSize, 
			aspectRatio, flags, cameraMatrix, distCoeffs,
			rvecs, tvecs, reprojErrs, totalAvgErr);

		printf("%s \n单摄标定平均重投影误差 = %.2f\n \n",
			ok ? "单摄标定成功！" : "单摄标定失败！",
			totalAvgErr);

		if (ok)
			SaveCameraParams(outputFilename, imageSize,
				aspectRatio,
				flags, cameraMatrix, distCoeffs,
				writeExtrinsics ? rvecs : vector<Mat>(),
				writeExtrinsics ? tvecs : vector<Mat>(),
				writeExtrinsics ? reprojErrs : vector<float>(),
				writePoints ? imagePoints : vector<vector<Point2f> >(),
				totalAvgErr);
		return ok;
	}

	void MonoCalib::MonoCalibration(std::vector<Mat>& Images, Mat& cameraMatrix, Mat& distCoeffs, Size imageSize, string OutputFilename)
	{		
		size_t i;
		int mode = DETECTION;

		vector<vector<Point2f> > imagePoints;
		vector<string> imageList;
		vector<Point2f> cornerPoints;

		for (i = 0; i < Images.size(); i++)
		{
			Mat img, view, viewGray;			
			vector<Point2f>& pointbuf = cornerPoints;

			view = Images[i].clone();
			view.copyTo(viewGray);

			// 1 / mMaxScale downscale to speed up corner detection.
			bool found;
			for (int scale = mMaxScale; scale > 0; scale -= 2)
			{
				Mat timg;
				resize(view, timg, imageSize / scale, (0.0, 0.0), (0.0, 0.0), cv::INTER_LINEAR);
					
				found = findChessboardCorners(timg, Size(mBoardWidth, mBoardHeight), pointbuf,
					CALIB_CB_FILTER_QUADS | CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK  | CALIB_CB_NORMALIZE_IMAGE);

				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(pointbuf);
						cornersMat *= scale;
					}
					break;
				}
			}
		
			// add code to check points order.

			// improve the found corners' coordinate accuracy
			if (found)
			{
				// viewGray is origin image size.
				cornerSubPix(viewGray, pointbuf, Size(30, 30),
					Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

				imagePoints.push_back(pointbuf);
				drawChessboardCorners(view, Size(mBoardWidth, mBoardHeight), Mat(pointbuf), found);
			}
//#if true
#if false
			double sf = 640. / MAX(view.rows, view.cols);
			resize(view, view, Size(), sf, sf);
			imshow("corners", view);

			int key = 0xff & waitKey(500);

			if ((key & 255) == 27)
				break;

			if (key == 'u' && mode == CALIBRATED)
				undistortImage = !undistortImage;
#endif
		}
		// use found image points to calibration.    
		if (imagePoints.size() > 0)
			RunAndSave(OutputFilename, imagePoints, imageSize,
				aspectRatio,
				flags, cameraMatrix, distCoeffs,
				writeExtrinsics, writePoints);

		// distort test.
		if(true)
		{
			Mat view, rview, map1, map2;
			initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
				getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
				imageSize, CV_16SC2, map1, map2);

			Mat img1 = imread("FF.bmp");
			Mat img2;

			if (img1.empty())
				return;

			undistort(img1, img2, cameraMatrix, distCoeffs, cameraMatrix);
			remap(img1, img2, map1, map2, INTER_LINEAR);
			imwrite("monoUndist.bmp", img2);
		}

		if (showUndistorted)
		{
			Mat view, rview, map1, map2;
			initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
				getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
				imageSize, CV_16SC2, map1, map2);

			for (i = 0; i < Images.size(); i++)
			{
				view = Images[i];
				if (view.empty())
					continue;

				undistort(view, rview, cameraMatrix, distCoeffs, cameraMatrix);
				remap(view, rview, map1, map2, INTER_LINEAR);

				//imshow("Image View", rview);
				imwrite("monoUndist.bmp", rview);
				int c = 0xff & waitKey();

				if ((c & 255) == 27 || c == 'q' || c == 'Q')
					break;
			}		
		}
	}

	MonoCalib::~MonoCalib()
	{

	}
}// NAMESPACE DUALCALIB

