#pragma once
#include "opencv2/core.hpp"
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

class DLTAlgorithm
{
private:

	vector<Point2f>point1, point2;

	void norm_point(vector<Point2f>&point, Mat &T);//��һ������

	Mat A;
	Mat T1, T2;
	Mat _dlt();

public:
	double dlt(vector<Point2f>&_point1, vector<Point2f>&_point2, Mat &H);
	//HΪ��άӰ��仯�����ص�ֵ�����ҽ�����Ϊ|Ah|/|h|
};

