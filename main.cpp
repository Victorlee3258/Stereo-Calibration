#pragma once

#include "../ImgLib/Filter.h"
#include "../mathlib/Vector.h"
#include "../mathlib/Matrix.h"

#include "StereoCalibration.h"
#include <memory>
#include <time.h>
#include <deque>

#include "UnitTestDistortImagePixelDitance.h"
#include "LM.h"

using namespace DualCalib;
using namespace std;
using namespace Eigen;

bool ReadStringList(const string& filename, vector<string>& l);

int PrintHelp();

int Calibration();

int main(int argc, char** argv)
{	

	//UnitTestDistortImagePixelDistance();

	Calibration();

	return 0;
}

int Calibration()
{
		long StartTime = GetTickCount();	
		string imagelistfn;
	
		if (imagelistfn == "")
		{
			imagelistfn = "./Input/calib.xml";
		}
	
		vector<string> imagelist;
		bool ok = ReadStringList(imagelistfn, imagelist);
	
		if (!ok || imagelist.empty())
		{
			printf("can not open image file, or the string list is empty \n");
			return PrintHelp();
		}
	
		DualCalibration Calibration(imagelist);
	
	 	Calibration.RunDualCalibration();
	
		long EndTime = GetTickCount();
	
		std::cout << "\n程序运行时间:" << (EndTime - StartTime) << "ms" << std::endl;

		return 0;
}

bool ReadStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;

	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;

	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string) * it);

	return true;
}

int PrintHelp()
{
	cout <<
		" Given a list of chessboard images, the number of corners (nx, ny)  on the chessboards \n"
		" Calibrate the cameras and display the\n"
		" rectified results along with the computed disparity images.   \n" << endl;
	return 0;
}

void EigenTest()
{
	//Eigen::VectorXf x(1);
	//x(0) = 2;
	//std::cout << "x:" << x << std::endl;
	//
	//LMFunctor functor;
	//
	//Eigen::LevenbergMarquardt<LMFunctor, float> lm(functor);
	//lm.minimize(x);
	//std::cout << "x that minimizes the function:" << x << std::endl;
}