#include "ReadImages.h"
#include "ImgLib\GrayImage.h"

static int PrintHelp()
{
	cout <<
		" Given a list of chessboard images, the number of corners (nx, ny)\n"
		" on the chessboards, and a flag: useCalibrated for \n"
		"   calibrated (0) or\n"
		"   uncalibrated \n"
		"     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
		"         matrix separately) stereo. \n"
		" Calibrate the cameras and display the\n"
		" rectified results along with the computed disparity images.   \n" << endl;
	cout << "Usage:\n ./stereo_calib -w board_width -h board_height [-nr /*dot not view results*/] <image list XML/YML file>\n" << endl;
	return 0;
}

bool ReadStringList()
{
	FILE *fp;
	char ImagePath[200];
	char *pFileName = "images.txt";

	//fp = fopen(IMAGE_LIST_FILE, "rb");
	//while (!feof(fp))

	//	while (fscanf(fp, "%s", ImagePath) > 0)
	//	{
	//		std::cout << "Image path:" << ImagePath << std::endl;
	//	}

	CGrayImage GrayImage;
	GrayImage.LoadFromTextFile(pFileName);

	return true;
}