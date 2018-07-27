#include "UnitTest.h"

using namespace std;
using namespace cv;

void computeNewRotation(Mat& R, Mat& T)
{
	// comoute new R1 R2.
	double _t1[3], _t2[3], _uu1[3] = { 0, 0, 0 }, _uu2[3] = { 0, 0, 0 };
	double _ww1[3], _ww2[3], _wr1[3][3], _wr2[3][3], _ri[3][3];

	Mat t1 = Mat(3, 1, CV_64F, _t1);
	Mat t2 = Mat(3, 1, CV_64F, _t2);

	Mat uu1 = Mat(3, 1, CV_64F, _uu1);
	Mat uu2 = Mat(3, 1, CV_64F, _uu2);

	Mat ww1 = Mat(3, 1, CV_64F, _ww1);
	Mat ww2 = Mat(3, 1, CV_64F, _ww2);

	Mat wR1 = Mat(3, 3, CV_64F, _wr1);
	Mat wR2 = Mat(3, 3, CV_64F, _wr2);

	Mat Ri = Mat(3, 3, CV_64F, _ri);
	Mat matI = Mat::eye(3, 3, CV_64F);

	Mat r, r1, r2;
	t1 = matI * T;
	t2 = R * T;

	int idx1 = fabs(_t1[0]) > fabs(_t1[1]) ? 0 : 1;
	int idx2 = fabs(_t2[0]) > fabs(_t2[1]) ? 0 : 1;

	double c1 = _t1[idx1], nt1 = cv::norm(t1, CV_L2);
	double c2 = _t2[idx2], nt2 = cv::norm(t2, CV_L2);

	_uu1[idx1] = c1 > 0 ? 1 : -1;
	_uu2[idx2] = c2 > 0 ? 1 : -1;

	// calculate global z rotation.
	ww1 = t1.cross(uu1);
	ww2 = t2.cross(uu2);

	double nw1 = cv::norm(ww1, CV_L2);
	double nw2 = cv::norm(ww2, CV_L2);

	if (nw1 > 0.0)
		ww1.convertTo(ww1, ww1.type(), acos(fabs(c1) / nt1) / nw1);

	if (nw2 > 0.0)
		ww2.convertTo(ww2, ww2.type(), acos(fabs(c2) / nt2) / nw2);

	Rodrigues(ww1, wR1);
	Rodrigues(ww2, wR2);

	// apply to both view.
	cv::gemm(wR1, matI, 1, 0, 0, r1, 0);
	cv::gemm(wR2, R, 1, 0, 0, r2, 0);
}


void CornerDetection(vector<Mat>& imgs)
{
	vector<Mat> DetectedEdges;
	int num = imgs.size();

	for (int i = 0; i < num; i++)
	{		
		Mat dst, colorDst;

		Canny(imgs[i], dst, 50, 200, 3);
		cvtColor(dst, colorDst, COLOR_GRAY2BGR);

		vector<Vec4i> lines;
		HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 10);

		for (size_t k = 0; k < lines.size(); k++)
		{
			line(colorDst, Point(lines[i][0], lines[i][1]),
				Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
		}

		imshow("ddd", colorDst);
		waitKey(0);
	}


}