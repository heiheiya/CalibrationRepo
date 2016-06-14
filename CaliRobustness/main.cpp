#include "DCSCalibrationRobustness.h"
#include <sstream>

using namespace std;
using namespace DCS;

const int FOLDERS = 3;
const string FOLDERNAME[FOLDERS] = { "phone2", "phone5", "phone13" };
const int GROUPPERFOLDER = 1;
int PICTUREPERGROUP1[GROUPPERFOLDER] = {18};
int PICTUREPERGROUP2[GROUPPERFOLDER] = { 17 };
const int BUFLEN = 100;
const int DEPTHMAX = 896;

int main()
{
	//variable definition
	DCSCalibrationRobustness test;
	char pathM[BUFLEN];
	char pathS[BUFLEN];
	char caliPath[BUFLEN];
	char pathPrefix[BUFLEN];

	DCS_ERR_CODE errCode;
	//test.setResource(CHESSBOARD);
	//for (int i = 0; i < FOLDERS; i++)
	//{
	//	int ret = sprintf_s(caliPath, BUFLEN, "resource\\%s\\CaliParams.yml", FOLDERNAME[i].c_str());
	//	test.getCaliParamsMat(caliPath);
	//	for (int j = 0; j < GROUPPERFOLDER; j++)
	//	{
	//		sprintf_s(pathPrefix, BUFLEN, "resource\\%s\\%d", FOLDERNAME[i].c_str(), j + 1);
	//		for (int k = 0; k < PICTUREPERGROUP1[j]; k++)
	//		{
	//			sprintf_s(pathM, BUFLEN, "resource\\%s\\%d\\main\\main%d.jpg", FOLDERNAME[i].c_str(), j + 1, k + 1);
	//			sprintf_s(pathS, BUFLEN, "resource\\%s\\%d\\sub\\sub%d.jpg", FOLDERNAME[i].c_str(), j + 1, k + 1);

	//			Mat leftImg, rightImg;
	//			leftImg = imread(pathM, -1);
	//			if (leftImg.empty())
	//			{
	//				cout << "failed to read left image!" << endl;
	//				continue;
	//			}
	//			rightImg = imread(pathS, -1);
	//			if (rightImg.empty())
	//			{
	//				cout << "failed to read right image!" << endl;
	//				continue;
	//			}

	//			int mainW = leftImg.cols;
	//			int mainH = leftImg.rows;
	//			int subW = rightImg.cols;
	//			int subH = rightImg.rows;
	//			float samplingM = (float)DEPTHMAX / max(mainW, mainH);
	//			float samplingS = (float)DEPTHMAX / max(subW, subH);

	//			errCode = test.scaleIntrinsicParams(mainW, mainH, subW, subH, samplingM, samplingS);
	//			if (errCode != DCS_SUCCESS)
	//			{
	//				continue;
	//			}

	//			Mat resizeImgL, resizeImgR;
	//			resize(leftImg, resizeImgL, Size(0,0), samplingM, samplingM, INTER_AREA);
	//			resize(rightImg, resizeImgR, Size(0, 0), samplingS, samplingS, INTER_AREA);

	//			cout << pathM << endl;
	//			cout << pathS << endl;

	//			float range = 0.008727;//rad, 0.5度
	//			float step = 0.0008727;//rad, 0.05度
	//			cout << "test x rotation" << endl;
	//			test.testRotation(resizeImgL, resizeImgR, 0, range, step, pathPrefix, k + 1);
	//			cout << "test y rotation" << endl;
	//			test.testRotation(resizeImgL, resizeImgR, 1, range, step, pathPrefix, k + 1);
	//			cout << "test z rotation" << endl;
	//			test.testRotation(resizeImgL, resizeImgR, 2, range, step, pathPrefix, k + 1);


	//			range = 1.5;//mm
	//			step = 0.1;//mm
	//			cout << "test x translation" << endl;
	//			test.testTranslation(resizeImgL, resizeImgR, 0, range, step, pathPrefix, k + 1);
	//			cout << "test y translation" << endl;
	//			test.testTranslation(resizeImgL, resizeImgR, 1, range, step, pathPrefix, k + 1);
	//			cout << "test z translation" << endl;
	//			test.testTranslation(resizeImgL, resizeImgR, 2, range, step, pathPrefix, k + 1);

	//			//range = 0.1;//值太大形变严重，提取角点失败
	//			//step = 0.01;
	//			//cout << "test main camera focallength" << endl;
	//			//test.testFocalLength(resizeImgL, resizeImgR, 0, range, step, pathPrefix, k + 1);
	//			//cout << "test sub camera focallength" << endl;
	//			//test.testFocalLength(resizeImgL, resizeImgR, 1, range, step, pathPrefix, k + 1);
	//			//cout << "test main and sub camera focallength" << endl;
	//			//test.testFocalLength(resizeImgL, resizeImgR, 2, range, step, pathPrefix, k + 1);

	//		}
	//	}
	//}

	test.setResource(NATURAL);
	for (int i = 0; i < FOLDERS; i++)
	{
		int ret = sprintf_s(caliPath, BUFLEN, "resource1\\%s\\CaliParams.yml", FOLDERNAME[i].c_str());
		test.getCaliParamsMat(caliPath);
		for (int j = 0; j < GROUPPERFOLDER; j++)
		{
			sprintf_s(pathPrefix, BUFLEN, "resource1\\%s\\%d", FOLDERNAME[i].c_str(), j + 1);
			for (int k = 0; k < PICTUREPERGROUP2[j]; k++)
			{
				sprintf_s(pathM, BUFLEN, "resource1\\%s\\%d\\main\\main%d.jpg", FOLDERNAME[i].c_str(), j + 1, k + 1);
				sprintf_s(pathS, BUFLEN, "resource1\\%s\\%d\\sub\\sub%d.jpg", FOLDERNAME[i].c_str(), j + 1, k + 1);

				Mat leftImg, rightImg;
				leftImg = imread(pathM, -1);
				if (leftImg.empty())
				{
					cout << "failed to read left image!" << endl;
					continue;
				}
				rightImg = imread(pathS, -1);
				if (rightImg.empty())
				{
					cout << "failed to read right image!" << endl;
					continue;
				}

				int mainW = leftImg.cols;
				int mainH = leftImg.rows;
				int subW = rightImg.cols;
				int subH = rightImg.rows;
				float samplingM = (float)DEPTHMAX / max(mainW, mainH);
				float samplingS = (float)DEPTHMAX / max(subW, subH);

				errCode = test.scaleIntrinsicParams(mainW, mainH, subW, subH, samplingM, samplingS);
				if (errCode != DCS_SUCCESS)
				{
					continue;
				}

				Mat resizeImgL, resizeImgR;
				resize(leftImg, resizeImgL, Size(0, 0), samplingM, samplingM, INTER_AREA);
				resize(rightImg, resizeImgR, Size(0, 0), samplingS, samplingS, INTER_AREA);

				cout << pathM << endl;
				cout << pathS << endl;

				float range = 0.008727;//rad, 0.5度
				float step = 0.0008727;//rad, 0.05度
				cout << "test x rotation" << endl;
				test.testRotation(resizeImgL, resizeImgR, 0, range, step, pathPrefix, k + 1);
				cout << "test y rotation" << endl;
				test.testRotation(resizeImgL, resizeImgR, 1, range, step, pathPrefix, k + 1);
				cout << "test z rotation" << endl;
				test.testRotation(resizeImgL, resizeImgR, 2, range, step, pathPrefix, k + 1);


				range = 1.5;//mm
				step = 0.1;//mm
				cout << "test x translation" << endl;
				test.testTranslation(resizeImgL, resizeImgR, 0, range, step, pathPrefix, k + 1);
				cout << "test y translation" << endl;
				test.testTranslation(resizeImgL, resizeImgR, 1, range, step, pathPrefix, k + 1);
				cout << "test z translation" << endl;
				test.testTranslation(resizeImgL, resizeImgR, 2, range, step, pathPrefix, k + 1);

				//range = 0.1;//值太大形变严重，提取角点失败
				//step = 0.01;
				//cout << "test main camera focallength" << endl;
				//test.testFocalLength(resizeImgL, resizeImgR, 0, range, step, pathPrefix, k + 1);
				//cout << "test sub camera focallength" << endl;
				//test.testFocalLength(resizeImgL, resizeImgR, 1, range, step, pathPrefix, k + 1);
				//cout << "test main and sub camera focallength" << endl;
				//test.testFocalLength(resizeImgL, resizeImgR, 2, range, step, pathPrefix, k + 1);

			}
		}
	}

	system("pause");
	return 0;
}
