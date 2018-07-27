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

	void norm_point(vector<Point2f>&point, Mat &T);//归一化数据

	Mat A;
	Mat T1, T2;
	Mat _dlt();

public:
	double dlt(vector<Point2f>&_point1, vector<Point2f>&_point2, Mat &H);
	//H为二维影射变化，返回的值是误差，我将误差定义为|Ah|/|h|
};

