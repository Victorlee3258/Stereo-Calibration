#pragma once
#include "../ImgLib/Filter.h"
#include "../mathlib/Vector.h"
#include "../mathlib/Matrix.h"

#include "StereoCalibration.h"
#include <memory>
#include <time.h>
#include <deque>

int UnitTestDistortImagePixelDistance();

bool ReadStringList(const string& filename, vector<string>& l);

int PrintHelp();

CMatrix ConvertMat2CMatrix(cv::Mat& cvMatrix);

void InitUndistortionRectifyMap(CMatrix &CameraMatrix, CMatrix &DistCoeffs, CMatrix &Rotation,
	CMatrix& NewCameraMatrix, size_t ImageRow, size_t ImageCols, CFloatImage &Map1, CFloatImage &Map2, vector<float>& xValue, vector<float>& yValue);