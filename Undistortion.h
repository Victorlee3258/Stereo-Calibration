#pragma once

#include "../mathlib/DualCameraGeometry.h"
#include "../mathlib/Matrix.h"
#include "../mathlib/Vector.h"
#include "../ImgLib/Filter.h"

#include <algorithm>
#include <iostream>

//#define CV_CN_MAX 512

namespace DualCalib
{
	class Undistortion
	{
	public:
		Undistortion();

		virtual ~Undistortion();

		void RunUndistortion();

		void BilinearInterp(CRGBYUVImage& pSrc, float newx, float newy, BYTE BGR[3]);

		// NewCameraMatrix can get from calibration, use P1 and P2 value.
		void InitUndistortionRectifyMap(CMatrix &_CameraMatrix, CMatrix &DistCoeffs, CMatrix &Rotation,
			CMatrix &NewCameraMatrix, size_t Row, size_t Cols, CFloatImage &Map1, CFloatImage &Map2);

		void Remap(CRGBYUVImage &Src, CRGBYUVImage &Dst, CFloatImage& Map1, CFloatImage& Map2);

		void RemapInvoker(CRGBYUVImage &Src, CRGBYUVImage &Dst, CFloatImage& m1, CFloatImage& m2);

		void RemapBilinear(CRGBYUVImage &Src, CRGBYUVImage &Dst, CFloatImage &_xy, CFloatImage &_fxy);

		CMatrix mK1, mK2, mD1, mD2, mP1, mP2, mR0, mR1;

		CFloatImage mRmap1, mRmap2, mRmap3, mRmap4;

		CRGBYUVImage mImage1, mImage2, mDist1, mDist2;

		size_t mImageHeight, mImageWidth;

	protected:



	private:



	};

}// NAMESPACE DUALCALIB


