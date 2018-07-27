#include "StereoCalibration.h"

namespace DualCalib
{
// construction function.
DualCalibration::DualCalibration(vector<string>& ImageList):mvImageList(ImageList)
{	
	// read init parameters
	string FileName = "./Input/InitParameters.txt";
	FileStorage fsSettings(FileName, FileStorage::READ);

	if (!fsSettings.isOpened())
	{
		std::cerr << "ERROR: wrong path to settings" << std::endl;
		return;
	}

	fsSettings["BoardWidth"] >> mnBoardWidth;
	fsSettings["BoardHeight"] >> mnBoardHeight;

	fsSettings["SquareSize"] >> mfSquareSize;
	fsSettings["IsFourChessboard"] >> mbIsFourChessboard;

	fsSettings["MaxScale"] >> mnMaxScale;

	fsSettings["DisplayCorners"] >> mbDisplayCorners;
	fsSettings["Flags"] >> mnFlags;
	//fsSettings["UseCalibrated"] >> mbUseUncalibrated;
	fsSettings["ShowRectified"] >> mbShowRectified;
	
	// parameters for mono calibration
	fsSettings["WriteExtrinsics"] >> mbWriteExtrinsics;
	fsSettings["WritePoints"] >> mbWritePoints;
	fsSettings["UndistortImage"] >> mbUndistortImage;
	fsSettings["ShowIndistorted"] >> mbShowUndistorted;
	fsSettings["MainCameraOutputFile"] >> msMainCameraOutputFile;
	fsSettings["SecondCameraOutputFile"] >> msSecondCameraOutputFile;

	mBoardSize = cv::Size(mnBoardWidth, mnBoardHeight);
}

void DualCalibration::RunDualCalibration()
{
	// process image for different chessboard type.
	ImageProcessFourChessboard(mvImageList, mvImgs);

	// mono camera calibration.
	vector<Mat> Images1, Images2;
	vector<Mat> ImagesDual;

	for (size_t i = 0; i < mvImgs.size() - 1; i = i + 2)
	{
		// images for mono calibration.
		Mat img1, img2;
		img1 = mvImgs[i].clone();
		img2 = mvImgs[i + 1].clone();

		Images1.push_back(img1);
		Images2.push_back(img2);
	}

	mImageSize1 = mvImgs[0].size();
	mImageSize2 = mvImgs[1].size();

	//MonoCalib cameraIntrinsicCalibration(mnBoardWidth, mnBoardHeight, mfSquareSize);
	//cameraIntrinsicCalibration.MonoCalibration(Images1, mK1, mD1, mImageSize1, msMainCameraOutputFile);
	//cameraIntrinsicCalibration.MonoCalibration(Images2, mK2, mD2, mImageSize2, msSecondCameraOutputFile);

	// run dual camera calibration 
	StereoCalib(mvImgs, mBoardSize);
}

Vec3f DualCalibration::RotationMatrixToEulerAngles(Mat R)
{
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6;

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}

	return Vec3f(x, y, z);
}


float DualCalibration::CalcAB(vector<float>& PointsX, vector<float>& PointsY)
{
	int n = PointsX.size();
	float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

	for (int i = 0; i < n; i++)
	{
		sumX += PointsX[i];
		sumY += PointsY[i];

		sumXX += PointsX[i] * PointsX[i];
		sumXY += PointsX[i] * PointsY[i];
	}
	float A = (sumX * sumY) / (n * sumXX);

	return A;
}

void DualCalibration::LinearFitting(vector<float> x, vector<float> y, double& slope)
{
	int length = x.size();
	double xmean = 0.0;
	double ymean = 0.0;
	for (int i = 0; i < length; i++)
	{
		xmean += x[i];
		ymean += y[i];
	}
	xmean /= length;
	ymean /= length;

	double sumx2 = 0.0;
	double sumy2 = 0.0;
	double sumxy = 0.0;
	for (int i = 0; i < length; i++)
	{
		sumx2 += (x[i]) * (x[i]);
		sumy2 += (y[i]) * (y[i]);
		sumxy += (y[i]) * (x[i]);
	}
	slope = sumxy / sumx2;
}

void DualCalibration::DistortPixel(const Mat& K, const Mat& R, const Mat& P, const Mat& D, const cv::Point2f& PointsLocation, cv::Point2d& DistortPointsLocation)
{
	Mat PointsLocation3 = Mat::zeros(3, 1, CV_64FC1);
	PointsLocation3.at<double>(0, 0) = (double)PointsLocation.x;
	PointsLocation3.at<double>(1, 0) = (double)PointsLocation.y;
	PointsLocation3.at<double>(2, 0) = 0.0;

	// transform image coordinates to be size and focus independent.
	Mat RPMatrix = R * P.inv();

	// location in camera coordinate.
	const double& x = RPMatrix.at<double>(0, 0) * PointsLocation.x + RPMatrix.at<double>(0, 1) * PointsLocation.y + RPMatrix.at<double>(0, 2);
	const double& y = RPMatrix.at<double>(1, 0) * PointsLocation.x + RPMatrix.at<double>(1, 1) * PointsLocation.y + RPMatrix.at<double>(1, 2);
	const double& z = RPMatrix.at<double>(2, 0) * PointsLocation.x + RPMatrix.at<double>(2, 1) * PointsLocation.y + RPMatrix.at<double>(2, 2);

	Point3d NormDistortedPointsLocation(0.0, 0.0, z);
	double& xd = NormDistortedPointsLocation.x;
	double& yd = NormDistortedPointsLocation.y;

	const double& k1 = D.at<double>(0, 0);
	const double& k2 = D.at<double>(0, 1);
	const double& K3 = D.at<double>(0, 2);
	const double& p1 = D.at<double>(0, 3);
	const double& p2 = D.at<double>(0, 4);

	// undistort.
	const double r2 = x * x + y * y;
	const double r4 = r2 * r2;
	const double r6 = r4 * r2;
	const double kr = (1.0 + k1 * r2 + k2 * r4 + K3 * r6);

	xd = x * kr + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
	yd = y * kr + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
}

CMatrix DualCalibration::ConvertMat2CMatrix(cv::Mat& cvMatrix)
{
	int nRows = cvMatrix.rows;
	int nCols = cvMatrix.cols;

	int size = cvMatrix.total() * cvMatrix.elemSize();
	unsigned char* data = new unsigned char[size];

	std::memcpy(data, cvMatrix.data, size * sizeof(unsigned char));
	double* pData = (double*)data;

	CMatrix CMat(nRows, nCols, pData);

	if (!data)
	{
		delete[]data;
		data = nullptr;
	}	 
	return CMat;
}

void DualCalibration::ImageProcessFourChessboard(const vector<string>& imagelist, vector<Mat>&images)
{
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	int k, nimages = (int)imagelist.size() / 2;
	vector<Mat> vImages;

	for (k = 0; k < 2; k++)
	{
		const string& filename = imagelist[k];
		Mat img = imread(filename, 0);

		if (img.empty())
		{ 
			std::cout << "Read image ERROR !!!" << std::endl;
			break;
		}

		// save image for rectify.
		mvOriginImages.push_back(img);

		// use mask to divide image to four small parts.
		int height = img.rows;
		int width = img.cols;

		int CeilHeight = (int)(height / mnCutRows);
		int CeilWidth = (int)(width / mnCutCols);

		// mask 1.
		Mat tmpImg1;
		Mat mask1 = Mat::zeros(img.size(), CV_8UC1);

		Rect rect1(0, 0, CeilWidth, CeilHeight);
		mask1(rect1).setTo(255);
		img.copyTo(tmpImg1, mask1);
		vImages.push_back(tmpImg1);

		// mask 2.
		Mat tmpImg2;
		Mat mask2 = Mat::zeros(img.size(), CV_8UC1);

		Rect rect2(CeilWidth, 0, CeilWidth, CeilHeight);
		mask2(rect2).setTo(255);
		img.copyTo(tmpImg2, mask2);
		vImages.push_back(tmpImg2);
		
		// mask 3.
		Mat tmpImg3;
		Mat mask3 = Mat::zeros(img.size(), CV_8UC1);
		
		Rect rect3(0, CeilHeight, CeilWidth, CeilHeight);
		mask3(rect3).setTo(255);
		img.copyTo(tmpImg3, mask3);
		vImages.push_back(tmpImg3);
		
		// mask 4.
		Mat tmpImg4;
		Mat mask4 = Mat::zeros(img.size(), CV_8UC1);

		Rect rect4(CeilWidth, CeilHeight, CeilWidth, CeilHeight);
		mask4(rect4).setTo(255);
		img.copyTo(tmpImg4, mask4);
		vImages.push_back(tmpImg4);		
	}

	for (size_t i = 0; i < vImages.size() / 2; i++)
	{
		images.push_back(vImages[i]);
		images.push_back(vImages[i + 4]);		
	}
}

void DualCalibration::ImageProcessOneChessboard(const vector<string>& imagelist, vector<Mat>&images)
{
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	int k, nimages = (int)imagelist.size() / 2;
	vector<Mat> vImages;

	for (k = 0; k < nimages * 2; k++)
	{
		const string& filename = imagelist[k];

		Mat img0 = imread(filename, 0);

		if (img0.empty())
		{
			std::cout << "Read image ERROR !!!" << std::endl;
			break;
		}

		// Need to consider different image size, use scale to control.	
		// Use scale factor to make main camera's image_size as second camera's image_size.
		// Resize main camera image size to second camera image size.		
		// use new image size.		
		Mat img;
		if (k == 0)
		{
			Rect rect(0, 0, mfMainImageWidth, mfMainImageHeight);
			img0(rect).copyTo(img);
		}
		if (k == 1)
		{
			Rect rect(0, 0, mfSecondImageWidth, mfSecondImageHeight);
			img0(rect).copyTo(img);
		}
		images.push_back(img);
		}
}

void DualCalibration::StereoCalib(vector<Mat>& imgs, Size boardSize)
{	
	vector<vector<Point3f> > objectPoints;	
	int i, j, k, nimages = (int)imgs.size() / 2;

	vector<vector<Point2f> > imagePoints[2];
	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);

	vector<Mat> goodImageList;	
	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imgs[i * 2 + k];
			if (img.empty())
			{
				cerr << "read image data ERROR ...";
				break;
			}

			bool found = false;
			cv::Size ImageSize = img.size();

			vector<Point2f>& corners = imagePoints[k][j];

			for (int scale = mnMaxScale; scale > 0; scale -= 2)
			{
				Mat timg;
				resize(img, timg, ImageSize / scale, (0.0, 0.0), (0.0, 0.0), cv::INTER_LINEAR);

				found = findChessboardCorners(timg, boardSize, corners,
					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
			
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= scale;
					}
					break;
				}
			}

			if (mbDisplayCorners)
			{
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);				

				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)cv::waitKey();
				
				//Allow ESC to quit.
				if (c == 27 || c == 'q' || c == 'Q') 
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, Size(5, 5), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
					30, 0.01));					
		}
		if (k == 2)
		{
			goodImageList.push_back(imgs[i * 2]);
			goodImageList.push_back(imgs[i * 2 + 1]);
			j++;
		}
	}	

	std::cout << "\n" << j << " 对双摄图像检测到" <<  std::endl;
	nimages = j;

	if (nimages < 2)
	{
		printf("Error: too little pairs to run the calibration\n");
		return;
	}
	
	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);	

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
			{
				objectPoints[i].push_back(Point3f(k * mfSquareSize, j * mfSquareSize, 0));
			}
	}
	// DLT test.
	//DLTAlgorithm dltTest;
	//Mat HMatrix;
	//for (size_t k = 0; k < imagePoints[0].size(); k++)
	//{
	//	dltTest.dlt(imagePoints[0][k], imagePoints[1][k], HMatrix);
	//	Mat h = findHomography(imagePoints[0][k], imagePoints[1][k], RANSAC);

	//	//std::cout << "H matrix:" << HMatrix << std::endl;
	//	std::cout << "h matrix:" << h << std::endl;
	//}

	// run stereo calibration.
	Mat cameraMatrix[2], distCoeffs[2];
	//cameraMatrix[0] = mK1;
	//cameraMatrix[1] = mK2;

	//distCoeffs[0] = mD1;
	//distCoeffs[1] = mD2;

	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		mImageSize1, R, T, E, F,
		mnFlags +	
		//CALIB_USE_INTRINSIC_GUESS +	
		CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K4 + 
		CALIB_FIX_K5 +
		CALIB_FIX_K6,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 1000, 1e-5));

	std::cout << "双摄标定最终重投影误差:" << rms << std::endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t * F * m1 = 0

	double err = 0;
	int npoints = 0;

	vector<Vec3f> lines[2];	
	vector<double> Errors;

	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];

		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);			
		}

		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
				imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x * lines[0][j][0] +
					imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
			
			Errors.push_back(errij);
			err += errij;
		}
		npoints += npt;
	}

	double MaxReprojectionError = *max_element(Errors.begin(), Errors.end());
	double AverageReprojectionError = err / npoints;

	std::cout << "双摄平均重投影误差:" << AverageReprojectionError << std::endl;
	std::cout << "双摄最大重投影误差:" << MaxReprojectionError << std::endl;

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	Vec3f EulerAngular = RotationMatrixToEulerAngles(R);
	EulerAngular = EulerAngular * 57.3;
	//std::cout << "R:" << R << std::endl;
	//std::cout << "rvec:" << EulerAngular << std::endl;

	stereoRectify(cameraMatrix[0], distCoeffs[0],
				cameraMatrix[0], distCoeffs[1],
				mImageSize1, R, T, R1, R2, P1, P2, Q,
				CALIB_ZERO_DISPARITY, 1, cv::Size(4224, 3136), &validRoi[0], &validRoi[1]);

	// undistort image points.
	vector<float> Yvalue1, Yvalue2;
	Mat R0 = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);

	for (size_t m = 0; m < imagePoints[0].size(); m++)
	{
		vector<Point2f> undistortImagePoints1, undistortImagePoints2;

		undistortPoints(imagePoints[0][m], undistortImagePoints1, cameraMatrix[0], distCoeffs[0], R1, cameraMatrix[0]);
		undistortPoints(imagePoints[1][m], undistortImagePoints2, cameraMatrix[1], distCoeffs[1], R2, cameraMatrix[0]);

		for (size_t n = 0 ; n < undistortImagePoints1.size(); n++)
		{
			double diff1 = (undistortImagePoints1[n].y - cameraMatrix[0].at<double>(1, 1));
			double diff2 = (undistortImagePoints2[n].y - cameraMatrix[1].at<double>(1, 1));

			Yvalue1.push_back(diff1);
			Yvalue2.push_back(diff2);
		}
	}
	
	// use undistort poiint to compute scale factor.
	LinearFitting(Yvalue1, Yvalue2, mdSlope);
	//std::cout << "slope:" << mdSlope << std::endl;

	// save intrinsic parameters
	FileStorage fs("./Output/DualParameters.yaml", FileStorage::WRITE);

	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];

		fs << "Average re-projection error" << AverageReprojectionError << "Max re-projection error" << MaxReprojectionError;

		fs << "R" << R << "Rvec" << EulerAngular << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		std::cout << "Error: can not save the extrinsic parameters." << std::endl;

	// IF BY CALIBRATED (BOUGUET'S METHOD)
	// use intrinsic parameters of each camera, but
	// compute the rectification transformation directly
	// from the fundamental matrix

	if(mbUseUncalibrated)
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}

		Mat H1, H2;
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);		
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, mImageSize1, H1, H2, 3);

		R1 = cameraMatrix[0].inv() * H1 * cameraMatrix[0];
		R2 = cameraMatrix[1].inv() * H2 * cameraMatrix[1];

		P1 = cameraMatrix[0];		
		P2 = cameraMatrix[1];
	}
	ImageUndistortion(cameraMatrix, distCoeffs, P1, P2, R, R1, R2, mImageSize1,mImageSize2, boardSize, nimages, goodImageList, validRoi);
}

void DualCalibration::ImageUndistortion(Mat cameraMatrix[2], Mat distCoeffs[2], Mat& P1, Mat& P2, Mat& R, Mat& R1, Mat& R2, Size imageSize, Size imageSize2, Size BoardSize, int nimages, vector<Mat> goodImageList,
										Rect validRoi[2])
{
	//..................... opencv undistort ............................
	Mat R0 = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);	
	Mat rmap[2][2];

	// get cy1 and cy2 parameters.
	mdCy1 = cameraMatrix[0].at<double>(1, 1);
	mdCy2 = cameraMatrix[1].at<double>(1, 1);

	// use main camera intrinsic for second camera.
	double K[3][3] = { cameraMatrix[0].at<double>(0, 0) *(1 - abs(mdSlope)), 0, cameraMatrix[0].at<double>(0, 2) ,
		0, cameraMatrix[0].at<double>(1, 1) * (1 - abs(mdSlope)), cameraMatrix[0].at<double>(1, 2), 0, 0, 1 };

	Mat K2 = Mat(3, 3, CV_64FC1, K);	

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], Mat(), cameraMatrix[0], imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R.inv(), cameraMatrix[0], imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat r1, r2;
	Rodrigues(R1, r1);
	Rodrigues(R2, r2);

	Mat canvas;
	double sf;
	int w, h;

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	vector<Mat> Images(2);
	Images[0] = imread("AF5.bmp", 0);
	Images[1] = imread("FF5.bmp", 0);

	if (Images[0].empty() || Images[1].empty())
	{
		cerr << "read rectify images error !" << std::endl;
	}

	for (size_t k = 0; k < Images.size(); k++)
	{
		Mat img = Images[k], rimg, cimg;

		remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
		cvtColor(rimg, cimg, COLOR_GRAY2BGR);

		mvOnlineImages.push_back(rimg);

		if(k == 0)
			imwrite("img1.bmp", cimg);
		if (k == 1)
			imwrite("img2.bmp", cimg);
			
		Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
		resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	}

	vector<Mat> rmap1, rmap2;
	for (int i = 0; i < goodImageList.size() / 2; i++)
	{
		for (int k = 0; k < 2; k++)
		{
			Mat img = goodImageList[i * 2 + k], rimg, cimg;

			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);

			if (k == 0)
			{
				rmap1.push_back(cimg);
				imwrite("./output/ img1.bmp", cimg);
			}				
			if (k == 1)
			{
				rmap2.push_back(cimg);
				imwrite("./output/ img2.bmp", cimg);
			}				

			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		}

		if (mbShowRectified)
		{
			if (!isVerticalStereo)
				for (int m = 0; m < canvas.rows; m += 16)
					line(canvas, Point(0, m), Point(canvas.cols, m), Scalar(0, 255, 0), 1, 8);
			else
				for (int m = 0; m < canvas.cols; m += 16)
					line(canvas, Point(m, 0), Point(m, canvas.rows), Scalar(0, 255, 0), 1, 8);

			imshow("rectified", canvas);
			char c = (char)cv::waitKey();
			if (c == 27 || c == 'q' || c == 'Q')
				break;
		}
	}

#if false
	// find corners to check y value.
	int NumCorners;
	std::vector<double> DifferenceY, DifferenceX;
	vector<vector<Point2f>> imagePoints1, imagePoints2;

	imagePoints1.resize(rmap1.size());
	imagePoints2.resize(rmap2.size());

	ofstream pointsData("pointsData.txt", ios_base::out);
	if (!pointsData.is_open())
	{
		cout << "open file ERROR !" << std::endl;
	}

	vector<float> Y1, Y2;
	for (size_t i = 0; i < rmap1.size(); i++)
	{
		vector<Point2f>& corners1 = imagePoints1[i];
		vector<Point2f>& corners2 = imagePoints2[i];
		bool found1, found2;

		found1 = cv::findChessboardCorners(rmap1[i], BoardSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE |
											CALIB_CB_FAST_CHECK);
		found2 = cv::findChessboardCorners(rmap2[i], BoardSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE |
											CALIB_CB_FAST_CHECK);

		
		NumCorners = corners1.size();
		if (found1 && found2)
		{
			//cornerSubPix(rmap1[i], corners1, cv::Size(5, 5), cv::Size(-1, -1),
			//	TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

			//cornerSubPix(rmap2[i], corners2, cv::Size(5, 5), cv::Size(-1, -1),
			//	TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

			for (size_t j = 0; j < corners1.size(); j++)
			{
				double x = (corners1[j].x - corners2[j].x);
				double y = abs(corners1[j].y) - (corners2[j].y);				

				std::cout << "y:" << y << std::endl;
				//std::cout << "y2:" << corners2[j].y << std::endl;

				pointsData << y << "\t";

				Y1.push_back(corners1[j].y - mdCy1);
				Y2.push_back(corners2[j].y - mdCy1);
				
				DifferenceX.push_back(x);
				DifferenceY.push_back(y);	
			}
		}		
	}

	//double slope;
	//LinearFitting(Y1, Y2, slope);
	//std::cout << "slope:" << slope << std::endl;

	Mat DrawImage(cv::Size(mnFinalImageWidth, mnFinalImageWidth), CV_8UC3);

	for (size_t m = 0; m < imagePoints1.size(); m++)
	{
		
		if(imagePoints1[m].size() != imagePoints2[m].size())
			continue;

		for (size_t n = 0; n < imagePoints1[m].size(); n++)
		{
			circle(DrawImage, imagePoints1[m][n], 1, Scalar(0, 0, 255), -1);
			circle(DrawImage, Point2d(imagePoints2[m][n].x - 0.3294, imagePoints2[m][n].y - 34.0935), 1, Scalar(255, 0, 0), -1);
		}
	}
	imwrite("./Output/Ycheck.bmp", DrawImage);
	waitKey();

	// compute y difference mean and stdev.
	double ySum = 0.0, xSum = 0.0;
	for (size_t m = 0; m < DifferenceY.size(); m++)
	{
		ySum += DifferenceY[m];
		xSum += DifferenceX[m];
	}

	double yMean = ySum / DifferenceY.size();
	double xMean = xSum / DifferenceX.size();

	double Accum = 0.0;

	std::for_each(std::begin(DifferenceY), std::end(DifferenceY), [&](const double d)
	{
		Accum += (d - yMean) * (d - yMean);
	});

	double stdev = sqrt(Accum / (DifferenceY.size() - 1));

	std::cout << "max Y offset:" <<  *max_element(DifferenceY.begin(), DifferenceY.end()) << std::endl;
	std::cout << "(Y1 - Y2) Mean:" << yMean << std::endl;
	std::cout << "(X1 - X2) Mean:" << xMean << std::endl;
#endif

}

DualCalibration::~DualCalibration()
{
	 //deconstruction function.	
}

//{
//	// matrix type convert.
//	Mat OneMat = Mat::eye(cv::Size(3, 3), CV_32FC1);
//	Undistortion Undist;
//
//	// read image data and resize to undistort.
//	Mat img;
//	Mat cvImage1 = imread("AF1.bmp", -1);
//
//	if (cvImage1.empty())
//		std::cerr << "Read image data ERROR !!!" << std::endl;
//
//	Rect rect(0, 0, NewWidth, NewHeight);
//	cvImage1(rect).copyTo(img);
//
//	cv::resize(img, img, cv::Size(SecondImageWidth, SecondImageHeight), (0.0, 0.0), (0.0, 0.0), cv::INTER_LINEAR);
//	imwrite("AF1Resize.bmp", img);
//
//	char* pFileName1 = "AF1Resize.bmp";
//	char* pFileName2 = "FF1.bmp";
//
//	if (!Undist.mImage1.LoadFromBMPFile(pFileName1) || !Undist.mImage2.LoadFromBMPFile(pFileName2))
//	{
//		std::cerr << "Load image file ERROR !!! \n";
//	}
//
//	// parameters prepare.
//	Undist.mK1 = ConvertMat2CMatrix(cameraMatrix[0]);
//	Undist.mK2 = ConvertMat2CMatrix(cameraMatrix[1]);
//
//	Undist.mD1 = ConvertMat2CMatrix(distCoeffs[0]);
//	Undist.mD2 = ConvertMat2CMatrix(distCoeffs[1]);
//
//	Undist.mP1 = ConvertMat2CMatrix(P1);
//	Undist.mP2 = ConvertMat2CMatrix(P2);
//
//	Undist.mR1 = ConvertMat2CMatrix(R);
//	Undist.mR0 = ConvertMat2CMatrix(OneMat);
//
//	Undist.mRmap1.Create(imageSize.width, imageSize.height);
//	Undist.mRmap2.Create(imageSize.width, imageSize.height);
//
//	Undist.mRmap3.Create(imageSize.width, imageSize.height);
//	Undist.mRmap4.Create(imageSize.width, imageSize.height);
//
//	Undist.RunUndistortion();
//}

}// NAMESPACE DUALCALIB
