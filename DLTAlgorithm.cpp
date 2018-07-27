#include "DLTAlgorithm.h"

void DLTAlgorithm::norm_point(vector<Point2f>&point, Mat &T)

{

	double num = point.size();
	double E[2] = { 0,0 }, E2[2] = { 0,0 }, D[2] = { 0,0 };

	int i;
	for (i = 0; i<num; i++)

	{
		E[0] += point[i].x;
		E[1] += point[i].y;

		E2[0] += point[i].x * point[i].x;
		E2[1] += point[i].y * point[i].y;
	}
	for (i = 0; i<2; i++)
	{
		E[i] /= num;
		E2[i] /= num;

		D[i] = E2[i] - E[i] * E[i];
	}

	T = Mat::zeros(3, 3, CV_64FC1);
	T.at<double>(0, 0) = 1.0 / sqrt(D[0]);
	T.at<double>(0, 2) = -1.0 * E[0] / sqrt(D[0]);
	T.at<double>(1, 1) = 1.0 / sqrt(D[1]);
	T.at<double>(1, 2) = -1.0 * E[1] / sqrt(D[1]);
	T.at<double>(2, 2) = 1;

	for (i = 0; i<num; i++)
	{
		point[i].x = point[i].x * T.at<double>(0, 0) + T.at<double>(0, 2);
		point[i].y = point[i].y * T.at<double>(1, 1) + T.at<double>(1, 2);
	}
}

double DLTAlgorithm::dlt(vector<Point2f>&_point1, vector<Point2f>&_point2, Mat &H)
{
	point1.assign(_point1.begin(), _point1.end());
	point2.assign(_point2.begin(), _point2.end());

	H = Mat::ones(3, 3, CV_64FC1);
	Mat h = _dlt();
	int i;

	for (i = 0; i<9; i++)
		H.at<double>(i / 3, i % 3) = h.at<double>(0, i);

	H = T2.inv() * H * T1;
	H = H * (1.0 / H.at<double>(2, 2));
	h = h * (1.0 / h.at<double>(0, 8));

	double error = norm(A * (h.t())) / norm(h);

	return error;
}

Mat DLTAlgorithm::_dlt()
{
	norm_point(point1, T1);
	norm_point(point2, T2);

	A = Mat::zeros(point1.size() * 2, 9, CV_64FC1);
	int i;

	for (i = 0; i<point1.size(); i++)
	{

		A.at<double>(2 * i + 0, 3) = -point1[i].x;
		A.at<double>(2 * i + 0, 4) = -point1[i].y;
		A.at<double>(2 * i + 0, 5) = -1;
		A.at<double>(2 * i + 0, 6) = point2[i].y * point1[i].x;
		A.at<double>(2 * i + 0, 7) = point2[i].y * point1[i].y;
		A.at<double>(2 * i + 0, 8) = point2[i].y;

		A.at<double>(2 * i + 1, 0) = point1[i].x;
		A.at<double>(2 * i + 1, 1) = point1[i].y;
		A.at<double>(2 * i + 1, 2) = 1;
		A.at<double>(2 * i + 1, 6) = -point2[i].x * point1[i].x;
		A.at<double>(2 * i + 1, 7) = -point2[i].x * point1[i].y;
		A.at<double>(2 * i + 1, 8) = -point2[i].x;
	}
	Mat evalues, evector;
	eigen(A.t() * A, evalues, evector);
	Mat ans = evector.row(evector.rows - 1);

	return ans;
}
