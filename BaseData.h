#pragma once

#include <iostream>
#include "opencv2/core.hpp"

// control the output of debug.
#define __DEBUG
//#undef  __DEBUG

//������������
//��p����ʾָ���������� "n"��ʾint����
//��b����ʾbool���� "s"��ʾset����
//��v����ʾvector�������� 'l'��ʾlist��������
//��m����ʾ���Ա����

namespace DualCalib 
{

	//int LibsTest()
	//{

	//	char* pFileName = "main1.bmp";
	//	CRGBYUVImage  image;

	//	if (!image.LoadFromBMPFile(pFileName))
	//	{
	//		printf("Load image file ERROR !!! \n");
	//		return -1;
	//	}

	//	BYTE* ImageData = image.GetImageData();
	//	if (!image.Rotate270())
	//	{
	//		return -1;
	//	}

	//	printf("Dim:%d", image.GetDim());

	//	BYTE min, max;
	//	image.GetImageRange(min, max);

	//	printf("Image size:%d \n", image.GetDim());

	//	//CMatrix
	//	double data[3] = { 1.0, 2.0, 3.0 };
	//	double* pDate = data;
	//	int cols = 3;
	//	int rows = 1;

	//	CMatrix Mat = CMatrix(rows, cols, pDate);

	//	double *dddata = Mat.GetData();
	//	for (int i = 0; i < Mat.GetSize(); i++)
	//	{
	//		printf("data:%f", dddata[i]);
	//	}
	//	int iRows = Mat.GetRowSize();

	//	printf("Rows:%d \n", iRows);

	//	//vector
	//	int len = 3;
	//	CVector vector = CVector(len, COL_VECTOR);
	//	vector[0] = 20.0;
	//	vector[1] = 21.0;
	//	vector[2] = 22.0;

	//	printf("vector:%f \n", vector[0]);
	//	printf("vector:%f \n", vector[1]);

	//	return 0;
	//}

} // namespace DUALCALIB
