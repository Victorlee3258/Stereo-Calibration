#include "Undistortion.h"


namespace DualCalib
{

// construction function.
Undistortion::Undistortion()
{
	// run remap function.
	
}

void Undistortion::RunUndistortion()
{
	// compute remap matrix.
	InitUndistortionRectifyMap(mK1, mD1, mR0, mP1, mImageHeight, mImageWidth, mRmap1, mRmap2);
	InitUndistortionRectifyMap(mK2, mD2, mR1, mP2, mImageHeight, mImageWidth, mRmap3, mRmap4);
	
	// remap.
	Remap(mImage1, mDist1, mRmap1, mRmap2);
	Remap(mImage2, mDist2, mRmap3, mRmap4);

	mDist1.SaveToBMPFile("mDist1.bmp");
    mDist2.SaveToBMPFile("mDist2.bmp");
}

void Undistortion::BilinearInterp(CRGBYUVImage& pSrc, float newx, float newy, BYTE BGR[3])
{
	int w = pSrc.GetWidth();
	int h = pSrc.GetHeight();

	if (newx < 0)newx = 0;
	if (newx > w - 1)newx = w - 1;
	if (newy < 0)newy = 0;
	if (newy > h - 1)newy = h - 1;

	int x0 = (int)newx;
	int x1 = x0 + 1;
	int y0 = (int)newy;
	int y1 = y0 + 1;

	float sumBGR[3] = { 0,0,0 };

	for (int i = 0; i < 3; i++)
		sumBGR[i] += pSrc.GetPixelAt(x0, y0)[i] * (x1 - newx)*(y1 - newy);

	for (int i = 0; i < 3; i++)
		sumBGR[i] += pSrc.GetPixelAt(x0, y1)[i] * (x1 - newx)*(newy - y0);

	for (int i = 0; i < 3; i++)
		sumBGR[i] += pSrc.GetPixelAt(x1, y0)[i] * (newx - x0)*(y1 - newy);

	for (int i = 0; i < 3; i++)
		sumBGR[i] += pSrc.GetPixelAt(x1, y1)[i] * (newx - x0)*(newy - y0);

	for (int i = 0; i < 3; i++)
	{
		sumBGR[i] += 0.5;
		if (sumBGR[i] < 0)sumBGR[i] = 0;
		if (sumBGR[i] > 255) sumBGR[i] = 255;
		BGR[i] = (int)sumBGR[i];
	}
}

void Undistortion::InitUndistortionRectifyMap(CMatrix &CameraMatrix, CMatrix &DistCoeffs, CMatrix &Rotation,
						CMatrix& NewCameraMatrix, size_t ImageRow, size_t ImageCols, CFloatImage &Map1,CFloatImage &Map2) 
{
	// before enter this function, need to check all matrix is not empty.
	// CMatrix CameraMatrix = CameraMatrix, DistCoeffs = DistCoeffs;
	// CMatrix NewCameraMatrix = NewCameraMatrix, Rotation = Rotation;

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
			m1f[j] = (float)u;
			m2f[j] = (float)v;
		}
	}

	if (!DistPtr)
	{
		delete[]DistPtr;
		DistPtr = nullptr;
	}
		
}

// use default Bilinear.
void Undistortion::Remap(CRGBYUVImage &Src, CRGBYUVImage &Dst, CFloatImage& Map1, CFloatImage& Map2)
{
	// check all data is ready.
	assert(Map1.GetHeight() > 0 || Map1.GetWidth() > 0);
	assert((Map2.GetHeight() * Map2.GetWidth()) > 0 || (Map2.GetHeight() * Map2.GetWidth()) == (Map1.GetHeight() * Map1.GetWidth()));
	
	Dst.Create(Src.GetWidth(), Src.GetHeight());

	// remap invoker.
	RemapInvoker(Src, Dst, Map1, Map2);
}

//use remap matrix to remap from src to dst image.
void Undistortion::RemapInvoker(CRGBYUVImage &Src, CRGBYUVImage &Dst, CFloatImage& m1, CFloatImage& m2)
{
	int x, y;
	int Height = Dst.GetHeight();
	int Width = Dst.GetWidth();

	int Range = Height * Width;
	const int BufSize = 1 << 14;
	
	CFloatImage Buffxy, Buffa;
	Buffxy.Create(Width * 2, Height);
	Buffa.Create(Width, Height);

	for (y = 0; y < Dst.GetHeight(); y++)
	{
		float* XY = Buffxy.GetImageLine(y);
		float* A = Buffa.GetImageLine(y);

		for (x = 0; x < Dst.GetWidth(); x++)
		{
			const float* sX = m1.GetPixelAt(x, y);
			const float* sY = m2.GetPixelAt(x, y);
			
			XY[x * 2] = sX[0];
			XY[x * 2 + 1] = sY[0];
		}
	}
	RemapBilinear(Src, Dst, Buffxy, Buffa);
}

void Undistortion::RemapBilinear(CRGBYUVImage &Src, CRGBYUVImage &Dst, CFloatImage &_xy, CFloatImage &_fxy)
{
	int Sheight = Src.GetHeight();
	int Swidth = Src.GetWidth();
	int Ssize = Sheight * Swidth;

	int Dheight = Dst.GetHeight();
	int Dwidth = Dst.GetWidth();
	int Dsize = Dheight * Dwidth;

	const unsigned char* S0 = Src.GetImageData();

	int k, cn = Src.GetDim();

	size_t Sstep = Src.GetWidth() * 3;
	unsigned char Cval[512];
	const int BoardValue[3] = {255, 0, 0};

	int dx, dy;
    unsigned Width1 = max(Swidth - 1, 0), Height1 = max(Sheight - 1, 0);
	assert(Ssize > 0);
	
	for (k = 0; k < cn; k++)
	{
		Cval[k] = static_cast<unsigned char>(BoardValue[k]);
	}

	for (dy = 0; dy < Dheight; dy++)
	{
		unsigned char* D = Dst.GetImageLine(dy);
		float* XY = _xy.GetImageLine(dy);
		unsigned char* FXY = (unsigned char*)_fxy.GetImageLine(dy);		

		int X0 = 0;
		bool PreInlier = false;

		for (dx = 0; dx <= Dwidth; dx++)
		{
			if (((unsigned)XY[dx * 2] > 0 && (unsigned)XY[dx * 2 + 1] > 0) && ((unsigned)XY[dx * 2] < Width1 && (unsigned)XY[dx * 2 + 1] < Height1))
			{
				
				int newx = (int)XY[dx * 2], newy = (int)XY[dx * 2 + 1];
				int sx0 = newx, sy0 = newy;
				int sx1 = sx0 + 1, sy1 = sy0 + 1;

				float sumBGR[3] = { 0,0,0 };

				for (int i = 0; i < 3; i++)
					sumBGR[i] += Src.GetPixelAt(sx0, sy0)[i] * (sx1 - newx)*(sy1 - newy);

				for (int i = 0; i < 3; i++)
					sumBGR[i] += Src.GetPixelAt(sx0, sy1)[i] * (sx1 - newx)*(newy - sy0);

				for (int i = 0; i < 3; i++)
					sumBGR[i] += Src.GetPixelAt(sx1, sy0)[i] * (newx - sx0)*(sy1 - newy);

				for (int i = 0; i < 3; i++)
					sumBGR[i] += Src.GetPixelAt(sx1, sy1)[i] * (newx - sx0)*(newy - sy0);

				for (int i = 0; i < 3; i++)
				{
					sumBGR[i] += 0.5;
					if (sumBGR[i] < 0)sumBGR[i] = 0;
					if (sumBGR[i] > 255) sumBGR[i] = 255;
					Dst.GetPixelAt(dx, dy)[i] = (int)sumBGR[i];					
				}
			}
			else
			{
				// for data out of range, set to constant value.
				int sx = (int)XY[dx * 2], sy = (int)XY[dx * 2 + 1];

				for (k = 0; k < cn; k++)
				{
					Dst.GetPixelAt(dx, dy)[k] = Cval[k];					
				}					
			}	
		}
	}
}

Undistortion::~Undistortion()
{
	// deconstruction function.
}

}// NAMESPACE DUALCALIB