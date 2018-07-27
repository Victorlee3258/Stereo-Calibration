#include "UnitTestDistortImagePixelDitance.h"


int UnitTestDistortImagePixelDistance()
{
	int imageGroups = 5;
	Size imageSize(1600, 1200);

	vector<Mat> CameraMatrix1(imageGroups), CameraMatrix2(imageGroups), DistortMatrix1(imageGroups), DistortMatrix2(imageGroups);
	vector<Mat> rmap1(imageGroups), rmap2(imageGroups), rmap3(imageGroups), rmap4(imageGroups);
	vector<Mat> P1(imageGroups), P2(imageGroups);

	// read camera parameters
	vector<string> FileNames1(imageGroups), FileNames2(imageGroups);

	FileNames1[0] = "./parameters/DualParameters1.yaml";
	FileNames1[1] = "./parameters/DualParameters2.yaml";
	FileNames1[2] = "./parameters/DualParameters3.yaml";
	FileNames1[3] = "./parameters/DualParameters4.yaml";
	FileNames1[4] = "./parameters/DualParameters5.yaml";

	vector<vector<float>> x1(imageGroups), y1(imageGroups), x2(imageGroups), y2(imageGroups);

	for (size_t i = 0; i < FileNames1.size(); i++)
	{
		FileStorage fsSettings1(FileNames1[i], FileStorage::READ);

		if (!fsSettings1.isOpened())
		{
			std::cerr << "ERROR: wrong path to settings" << std::endl;
			return -1;
		}

		fsSettings1["M1"] >> CameraMatrix1[i];
		fsSettings1["D1"] >> DistortMatrix1[i];

		fsSettings1["M2"] >> CameraMatrix2[i];
		fsSettings1["D2"] >> DistortMatrix2[i];

		CMatrix K1, K2, D1, D2, R1, R2;
		CFloatImage Map1, Map2, Map3, Map4;
		Mat OneMat = Mat::eye(cv::Size(3, 3), CV_32FC1);

		K1 = ConvertMat2CMatrix(CameraMatrix1[i]);
		D1 = ConvertMat2CMatrix(DistortMatrix1[i]);
		R1 = ConvertMat2CMatrix(OneMat);

		K2 = ConvertMat2CMatrix(CameraMatrix2[i]);
		D2 = ConvertMat2CMatrix(DistortMatrix2[i]);
		R2 = ConvertMat2CMatrix(OneMat);

		InitUndistortionRectifyMap(K1, D1, R1, K1, 1200, 1600, Map1, Map2, x1[i], y1[i]);
		InitUndistortionRectifyMap(K2, D2, R2, K2, 1200, 1600, Map3, Map4, x2[i], y2[i]);

		std::cout << "DistortMatrix2:" << DistortMatrix2[i] << std::endl;
	}
	
	for (size_t i = 0; i < x1.size(); i++)
	{
		// set to zero to clear vector data.
		std::vector<double> maxValue1(0) , maxValue2(0);

		for (size_t j = 0; j < x1[i].size(); j++)
		{
			double SqrtValue1 = sqrt((x1[i][j] - x1[0][j]) * (x1[i][j] - x1[0][j]) + (y1[i][j] - y1[0][j]) * (y1[i][j] - y1[0][j]));
			double SqrtValue2 = sqrt((x2[i][j] - x2[0][j]) * (x2[i][j] - x2[0][j]) + (y2[i][j] - y2[0][j]) * (y2[i][j] - y2[0][j]));

			maxValue1.push_back(SqrtValue1);
			maxValue2.push_back(SqrtValue2);
		}
		std::cout << "Max value1:" << *max_element(maxValue1.begin(), maxValue1.end()) << std::endl;	
		std::cout << "Max value2:" << *max_element(maxValue2.begin(), maxValue2.end()) << std::endl;
	}
	return 0;
}

// only for matrix, not for image data.
CMatrix ConvertMat2CMatrix(cv::Mat& cvMatrix)
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

void InitUndistortionRectifyMap(CMatrix &CameraMatrix, CMatrix &DistCoeffs, CMatrix &Rotation,
	CMatrix& NewCameraMatrix, size_t ImageRow, size_t ImageCols, CFloatImage &Map1, CFloatImage &Map2, vector<float>& xValue, vector<float>& yValue)
{
	// before enter this function, need to check all matrix is not empty.
	CMatrix R(3, 3);
	if (!(Rotation.GetRowSize() == 0))
		R.MakeUnit();

	const int Size = ImageRow * ImageCols;

	CMatrix A(3, 3), Ar(3, 4);
	A = CameraMatrix;

	// check if matrix is empty.
	if (NewCameraMatrix.GetColSize() > 0)
		Ar = NewCameraMatrix;
	else
	{
		// use default camera matrix.
		Ar = A;
	}

	assert(A.GetSize() == (3 * 3) && A.GetSize() == R.GetSize());
	assert(Ar.GetSize() == (3 * 3) || Ar.GetSize() == (3 * 4));

	CMatrix _Ar(3, 3);
	for (int i = 0; i < _Ar.GetRowSize(); i++)
	{
		for (int j = 0; j < _Ar.GetColSize(); j++)
		{
			_Ar[i][j] = Ar[i][j];
		}
	}

	CMatrix iRotation = (_Ar * R).Inverse();
	const double* ir = iRotation.GetData();

	double u0 = A[0][2], v0 = A[1][2];
	double fx = A[0][0], fy = A[1][1];

	// check distortion size (k1,k2,p1,p2,k3)
	// only use K1, K2, and P1 = 0, P2 =0.
	assert(DistCoeffs.GetSize() == 4 || DistCoeffs.GetSize() == 5 ||
		DistCoeffs.GetSize() == 8 || DistCoeffs.GetSize() == 12);

	if (DistCoeffs.GetColSize() != 1)
		DistCoeffs = DistCoeffs.Transpose();

	double* DistPtr = DistCoeffs.GetData();
	double K1 = DistPtr[0];
	double K2 = DistPtr[1];
	double P1 = DistPtr[2];
	double P2 = DistPtr[3];

	double K3 = DistCoeffs.GetColSize() + DistCoeffs.GetRowSize() - 1 >= 5 ? DistPtr[4] : 0;
	double K4 = DistCoeffs.GetColSize() + DistCoeffs.GetRowSize() - 1 >= 8 ? DistPtr[5] : 0;
	double K5 = DistCoeffs.GetColSize() + DistCoeffs.GetRowSize() - 1 >= 8 ? DistPtr[6] : 0;
	double K6 = DistCoeffs.GetColSize() + DistCoeffs.GetRowSize() - 1 >= 8 ? DistPtr[7] : 0;

	// Sx not use in this model.
	double S1 = 0, S2 = 0, S3 = 0, S4 = 0;
	for (size_t i = 0; i < ImageRow; i++)
	{
		float* m1f = Map1.GetImageLine(i);
		float* m2f = Map2.GetImageLine(i);

		short* m1 = (short*)m1f;
		unsigned short* m2 = (unsigned short*)m2f;

		double _x = i * ir[1] + ir[2], _y = i * ir[4] + ir[5], _w = i * ir[7] + ir[8];

		for (size_t j = 0; j < ImageCols; j++, _x += ir[0], _y += ir[3], _w += ir[6])
		{
			double w = 1. / _w, x = _x * w, y = _y * w;

			double x2 = x * x, y2 = y * y;
			double r2 = x2 + y2, _2xy = 2 * x * y;
			double kr = (1 + ((K3 * r2 + K2) * r2 + K1) * r2) / (1 + ((K6 * r2 + K5) * r2 + K4) * r2);
			
			double u = fx * (x * kr + P1 * _2xy + P2 * (r2 + 2 * x2) + S1 * r2 + S2 * r2 * r2) + u0;
			double v = fy * (y * kr + P1 * (r2 + 2 * y2) + P2 * _2xy + S3 * r2 + S4 * r2 * r2) + v0;

			// for float data type of the remap matrix.
			xValue.push_back((float)u);
			yValue.push_back((float)v);
		}
	}

	if (!DistPtr)
	{
		delete[]DistPtr;
		DistPtr = nullptr;
	}
}
