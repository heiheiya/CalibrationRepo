#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>

#include "DTfilter/dtfilter_cpu.hpp"
#include "MBMElas/MBMElas.h"
#include "DCSCalibrationRobustness.h"
#include <direct.h>

using namespace std;
using namespace cv;
using namespace DCS;

const int LENGTH = 150;

DCS::DCSCalibrationRobustness::DCSCalibrationRobustness()
	:m_res(0)
{

}

DCS::DCSCalibrationRobustness::~DCSCalibrationRobustness()
{

}

void DCS::DCSCalibrationRobustness::getCaliParamsMat(const char* pCalipath)
{
	//read calibration parameters from file
	CaliParams cali;
	readCaliParams(pCalipath, cali);

	//convert parameters to opencv Mat
	cali.Intrinsic_L.convertTo(m_INL, CV_64FC1);
	cali.Intrinsic_R.convertTo(m_INR, CV_64FC1);
	cali.Distortion_L.convertTo(m_DL, CV_64FC1);
	cali.Distortion_R.convertTo(m_DR, CV_64FC1);
	cali.Rotation.convertTo(m_R, CV_64FC1);
	cali.Translation.convertTo(m_T, CV_64FC1);
}

DCS_ERR_CODE DCS::DCSCalibrationRobustness::findCheckerboardPoints(Mat& image, Size patternSize, Size win, vector<Point2f>& vPoint2DSet)
{
	if (image.empty())
	{
		return DCS_ERR_BUFFER_NULL;
	}

	vPoint2DSet.clear();
	int pointsNum = patternSize.height*patternSize.width;
	vector<Point2f> checkerpoints;
	int count = 0;

	Mat imageGray;
	if (image.channels() == 3)
	{
		cvtColor(image, imageGray, COLOR_BGR2GRAY);
	}
	else
	{
		image.copyTo(imageGray);
	}
	//提取像素级角点;
	if (false == findChessboardCorners(imageGray, patternSize, checkerpoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS))
	{
		cout << "can not find chessboard corners!" << endl;
		return DCS_ERR_FUNCTION_FAILED;
	}

	//亚像素角点优化;
	cornerSubPix(imageGray, checkerpoints, win, Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
	for (int i = 0; i < pointsNum; i++)
	{
		Point2f temp;
		temp.x = checkerpoints[i].x;
		temp.y = checkerpoints[i].y;
		if (temp.x>0 && temp.y > 0)
		{
			vPoint2DSet.push_back(temp);
		}
	}

	return DCS_SUCCESS;
}

void DCS::DCSCalibrationRobustness::drawImagePoint(Mat& pImg, Point2f& pts, Scalar color, int Num /*= -1*/)
{
	if (pts.x > 0 || pts.y > 0)
	{
		circle(pImg, Point(pts.x, pts.y), 1, color, 3);
		if (Num > 0)
		{
			char a[10];
			memset(a, 0, 10);
			sprintf_s(a, 10, "%d", Num);
			putText(pImg, a, Point(pts.x, pts.y), 1, 1, Scalar(255, 255, 0));
		}
	}
}

DCS_ERR_CODE DCS::DCSCalibrationRobustness::scaleIntrinsicParams(int mainW, int mainH, int subW, int subH, double resizeRatioLeft, double resizeRatioRight)
{
	double imgRadioL = (double)mainW / (double)mainH;
	double imgRadioR = (double)subW / (double)subH;
	if (imgRadioL != imgRadioR)
	{
		return DCS_ERR_INVALIDATE_IMAGE_SIZE;
	}
	Size img_size = Size(mainH, mainW);

	m_INL.copyTo(m_M1);
	m_INR.copyTo(m_M2);

	m_M1.at<double>(0, 0) = m_INL.at<double>(0, 0) * resizeRatioLeft;
	m_M1.at<double>(0, 2) = m_INL.at<double>(0, 2) * resizeRatioLeft;
	m_M1.at<double>(1, 1) = m_INL.at<double>(1, 1) * resizeRatioLeft;
	m_M1.at<double>(1, 2) = m_INL.at<double>(1, 2) * resizeRatioLeft;

	m_M2.at<double>(0, 0) = m_INR.at<double>(0, 0) * resizeRatioRight;
	m_M2.at<double>(0, 2) = m_INR.at<double>(0, 2) * resizeRatioRight;
	m_M2.at<double>(1, 1) = m_INR.at<double>(1, 1) * resizeRatioRight;
	m_M2.at<double>(1, 2) = m_INR.at<double>(1, 2) * resizeRatioRight;

	if (abs(imgRadioL - 0.5625) < 0.0001)
	{
		m_M1.at<double>(0, 2) = m_M1.at<double>(0, 2) - mainW * 0.125;
		m_M2.at<double>(0, 2) = m_M2.at<double>(0, 2) - subW * 0.125;
	}
	else if (abs(imgRadioL - 1.7777) < 0.0001)
	{
		m_M1.at<double>(1, 2) = m_M1.at<double>(1, 2) - mainW * 0.125;
		m_M2.at<double>(1, 2) = m_M2.at<double>(1, 2) - subW * 0.125;
	}

	return DCS_SUCCESS;
}

void DCS::DCSCalibrationRobustness::rectify(Mat& leftImg, Mat& rightImg, Mat& M1, Mat& M2, Mat& D1, Mat& D2, Mat& R, Mat &T, Mat &map11, Mat &map12, int &maxDisparity)
{
	Mat img1, img2;
	leftImg.copyTo(img1);
	rightImg.copyTo(img2);

	Size imgSize = img1.size();

	Mat R1, R2, P1, P2, Q;

	stereoRectify(M1, D1, M2, D2, imgSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, m_res, imgSize);

	//compute max disparity
	double bassline = sqrt((T.at<double>(0, 0)*T.at<double>(0, 0)) + (T.at<double>(1, 0)*T.at<double>(1, 0)) +
		(T.at<double>(2, 0)*T.at<double>(2, 0)));
	double fPixel = Q.at<double>(2, 3);
	maxDisparity = cvRound(bassline*fPixel / 300);

	Mat map21, map22;
	//  
	initUndistortRectifyMap(M1, D1, R1, P1, imgSize, CV_32FC1, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, imgSize, CV_32FC1, map21, map22);

	Mat img1r, img2r;
	remap(img1, img1r, map11, map12, INTER_AREA);
	remap(img2, img2r, map21, map22, INTER_AREA);
	leftImg = img1r;
	rightImg = img2r;
}



void DCS::DCSCalibrationRobustness::readCaliParams(const char* pCalipath, CaliParams& cali)
{
	FileStorage CP(pCalipath, FileStorage::READ);
	CP["IntrinsicL"] >> cali.Intrinsic_L;
	CP["IntrinsicR"] >> cali.Intrinsic_R;
	CP["DistortionL"] >> cali.Distortion_L;
	CP["DistortionR"] >> cali.Distortion_R;
	CP["Rotation"] >> cali.Rotation;
	CP["Translation"] >> cali.Translation;
	cali.Err = (float)CP["Err"];
	CP["IntrinsicL_ini"] >> cali.Intrinsic_L_ini;
	CP["IntrinsicR_ini"] >> cali.Intrinsic_R_ini;
	CP["DistortionL_ini"] >> cali.Distortion_L_ini;
	CP["DistortionR_ini"] >> cali.Distortion_R_ini;
	CP["Rotation_ini"] >> cali.Rotation_ini;
	CP["Translation_ini"] >> cali.Translation_ini;
	CP.release();
}

void DCS::DCSCalibrationRobustness::testRotation(Mat& leftImage, Mat& rightImage, int index, double range, double step, string pathPrefix, int num)
{
	Mat imageL, imageR;
	Mat map11, map12;
	int maxDisparity;
	leftImage.copyTo(imageL);
	rightImage.copyTo(imageR);
	rectify(imageL, imageR, m_M1, m_M2, m_DL, m_DR, m_R, m_T, map11, map12, maxDisparity);

	Mat depthImg;
	switch (m_res)
	{
	case 0:
		saveResult(imageL, imageR, pathPrefix, num, "R", index, 0);
		break;
	case 1:
		generateDepth(imageL, imageR, depthImg, leftImage, map11, map12, maxDisparity);
		saveResult(imageL, imageR, depthImg, pathPrefix, num, "R", index, 0);
		break;
	default:
		saveResult(imageL, imageR, pathPrefix, num, "R", index, 0);
		break;
	}
	

	Mat R;
	m_R.copyTo(R);//backup
	Mat RVec = Mat::zeros(3, 1, CV_64FC1);
	Rodrigues(R, RVec);

	int counts = 2 * range / step + 1;
	RVec.at<double>(index, 0) -= range;

	int count = 0;
	//rotation params change
	while (count < counts)
	{
		Rodrigues(RVec, R);
		leftImage.copyTo(imageL);
		rightImage.copyTo(imageR);
		rectify(imageL, imageR, m_M1, m_M2, m_DL, m_DR, R, m_T, map11, map12, maxDisparity);
		count++;
		cout << fixed << setw(3) << count << endl;
		switch (m_res)
		{
		case 0:
			saveResult(imageL, imageR, pathPrefix, num, "R", index, count);
			break;
		case 1:
			generateDepth(imageL, imageR, depthImg, leftImage, map11, map12, maxDisparity);
			saveResult(imageL, imageR, depthImg, pathPrefix, num, "R", index, count);
			break;
		default:
			saveResult(imageL, imageR, pathPrefix, num, "R", index, count);
			break;
		}
		RVec.at<double>(index, 0) += step;
	}
}

void DCS::DCSCalibrationRobustness::testTranslation(Mat& leftImage, Mat& rightImage, int index, double range, double step, string pathPrefix, int num)
{
	Mat imageL, imageR;
	Mat map11, map12;
	int maxDisparity;
	leftImage.copyTo(imageL);
	rightImage.copyTo(imageR);
	rectify(imageL, imageR, m_M1, m_M2, m_DL, m_DR, m_R, m_T, map11, map12, maxDisparity);
	
	Mat depthImg;
	switch (m_res)
	{
	case 0:
		saveResult(imageL, imageR, pathPrefix, num, "T", index, 0);
		break;
	case 1:
		generateDepth(imageL, imageR, depthImg, leftImage, map11, map12, maxDisparity);
		saveResult(imageL, imageR, depthImg, pathPrefix, num, "T", index, 0);
		break;
	default:
		saveResult(imageL, imageR, pathPrefix, num, "T", index, 0);
		break;
	}

	Mat T;
	m_T.copyTo(T);//backup

	int counts = 2 * range / step + 2;
	T.at<double>(index, 0) -= range;

	int count = 0;
	//translation params change
	while (count<counts)
	{
		leftImage.copyTo(imageL);
		rightImage.copyTo(imageR);
		rectify(imageL, imageR, m_M1, m_M2, m_DL, m_DR, m_R, T, map11, map12, maxDisparity);
		count++;
		cout << fixed << setw(3) << count << endl;
		switch (m_res)
		{
		case 0:
			saveResult(imageL, imageR, pathPrefix, num, "T", index, count);
			break;
		case 1:
			generateDepth(imageL, imageR, depthImg, leftImage, map11, map12, maxDisparity);
			saveResult(imageL, imageR, depthImg, pathPrefix, num, "T", index, count);
			break;
		default:
			saveResult(imageL, imageR, pathPrefix, num, "T", index, count);
			break;
		}
		T.at<double>(index, 0) += step;
	}
}

void DCS::DCSCalibrationRobustness::testFocalLength(Mat& leftImage, Mat& rightImage, int index, double range, double step, string pathPrefix, int num)
{
	double multiple = 1.0 - range;
	int counts = 2 * range / step + 1;

	switch (index)
	{
	case 0:
	{
		Mat M1;
		m_M1.copyTo(M1);//backup
		double ax = M1.at<double>(0, 0); //focallength in pixel
		double ay = M1.at<double>(1, 1); //equivalent focallength

		int count = 0;
		//focallength params change
		while (count <= counts)
		{
			Mat imageL, imageR;
			Mat map11, map12;
			int maxDisparity;
			leftImage.copyTo(imageL);
			rightImage.copyTo(imageR);
			M1.at<double>(0, 0) = ax * multiple;
			M1.at<double>(1, 1) = ay * multiple;
			rectify(imageL, imageR, M1, m_M2, m_DL, m_DR, m_R, m_T, map11, map12, maxDisparity);
			count++;
			cout << fixed << setw(3) << count << endl;
			saveResult(imageL, imageR, pathPrefix, num, "F", index, count);
			multiple += step;
		}
	}
	break;
	case 1:
	{
		Mat M2;
		m_M2.copyTo(M2);//backup
		double ax = M2.at<double>(0, 0); //focallength in pixel
		double ay = M2.at<double>(1, 1); //equivalent focallength

		int count = 0;
		//focallength params change
		while (count <= counts)
		{
			Mat imageL, imageR;
			Mat map11, map12;
			int maxDisparity;
			leftImage.copyTo(imageL);
			rightImage.copyTo(imageR);
			M2.at<double>(0, 0) = ax * multiple;
			M2.at<double>(1, 1) = ay * multiple;
			rectify(imageL, imageR, m_M1, M2, m_DL, m_DR, m_R, m_T, map11, map12, maxDisparity);
			count++;
			cout << fixed << setw(3) << count << endl;
			saveResult(imageL, imageR, pathPrefix, num, "F", index, count);
			multiple += step;
		}
	}
	break;
	case 2:
	{
		Mat M1, M2;
		m_M1.copyTo(M1);
		m_M2.copyTo(M2);//backup
		double axM = M1.at<double>(0, 0); //focallength in pixel
		double ayM = M1.at<double>(1, 1); //equivalent focallength
		double axS = M2.at<double>(0, 0); //focallength in pixel
		double ayS = M2.at<double>(1, 1); //equivalent focallength

		int count = 0;
		//focallength params change
		while (count <= counts)
		{
			Mat imageL, imageR;
			Mat map11, map12;
			int maxDisparity;
			leftImage.copyTo(imageL);
			rightImage.copyTo(imageR);
			M1.at<double>(0, 0) = axM * multiple;
			M1.at<double>(1, 1) = ayM * multiple;
			M2.at<double>(0, 0) = axS * multiple;
			M2.at<double>(1, 1) = ayS * multiple;
			rectify(imageL, imageR, M1, M2, m_DL, m_DR, m_R, m_T, map11, map12, maxDisparity);
			count++;
			cout << fixed << setw(3) << count << endl;
			saveResult(imageL, imageR, pathPrefix, num, "F", index, count);
			multiple += step;
		}
	}
		break;
	default:
		break;
	}
}

void DCS::DCSCalibrationRobustness::saveResult(Mat& leftImage, Mat& rightImage, string pathPrefix, int num, char* mode, int index, int count)
{
	char name[LENGTH];
	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d", pathPrefix.c_str(), mode, index);
	mkDir(name);

	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\main%d_rectify_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, leftImage);

	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\sub%d_rectify_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, rightImage);

	Mat lineMat;
	drawImageLine(leftImage, rightImage, lineMat);
	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\mainSub%d_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, lineMat);

	vector<Point2f> point2DL, point2DR;
	Size patternSize = Size(8, 11);
	int num1 = 0, num2 = 0, num3 = 0, num4 = 0, num5 = 0, num6 = 0, total = 0;
	float disY = 0;
	//find corner points and draw it on image
	DCS_ERR_CODE errCode;
	errCode = findCheckerboardPoints(leftImage, patternSize, Size(15, 15), point2DL);
	if (errCode != DCS_SUCCESS)
	{
		return;
	}
	errCode = findCheckerboardPoints(rightImage, patternSize, Size(15, 15), point2DR);
	if (errCode != DCS_SUCCESS)
	{
		return;
	}

	if (point2DL.size() == point2DR.size() && point2DL.size() != 0 && point2DL.size() == patternSize.width*patternSize.height)
	{
		for (int j = 0; j < point2DL.size(); j++)
		{
			float dy = abs(point2DL[j].y - point2DR[j].y);
			if (abs(dy) < 1)
			{
				num1++;
				drawImagePoint(leftImage, point2DL[j], Scalar(0, 255, 0));//绿色
				drawImagePoint(rightImage, point2DR[j], Scalar(0, 255, 0));
			}
			else if (abs(dy) < 2)
			{
				num2++;
				drawImagePoint(leftImage, point2DL[j], Scalar(255, 0, 0));//蓝色
				drawImagePoint(rightImage, point2DR[j], Scalar(255, 0, 0));
			}
			else if (abs(dy) < 3)
			{
				num3++;
				drawImagePoint(leftImage, point2DL[j], Scalar(0, 255, 255));//黄色
				drawImagePoint(rightImage, point2DR[j], Scalar(0, 255, 255));
			}
			else if (abs(dy) < 4)
			{
				num4++;
				drawImagePoint(leftImage, point2DL[j], Scalar(0, 118, 238));//橙红
				drawImagePoint(rightImage, point2DR[j], Scalar(0, 118, 238));
			}
			else if (abs(dy) < 5)
			{
				num5++;
				drawImagePoint(leftImage, point2DL[j], Scalar(211, 0, 148));//紫色
				drawImagePoint(rightImage, point2DR[j], Scalar(211, 0, 148));
			}
			else
			{
				num6++;
				drawImagePoint(leftImage, point2DL[j], Scalar(0, 0, 255));//红色
				drawImagePoint(rightImage, point2DR[j], Scalar(0, 0, 255));
			}
			disY += dy;
			total++;
		}
	}

	//save image
	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\drawMain%d_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, leftImage);
	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\drawSub%d_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, rightImage);

	disY /= (float)total;
	float ratio1 = (float)num1 / (float)total;
	float ratio2 = (float)num2 / (float)total;
	float ratio3 = (float)num3 / (float)total;
	float ratio4 = (float)num4 / (float)total;
	float ratio5 = (float)num5 / (float)total; 
	float ratio6 = (float)num6 / (float)total;


	//save data
	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\result_%s_%d_%d.txt", pathPrefix.c_str(), mode, index, mode, index, count);
	fstream file;
	if (num != 1)
	{
		file.open(name, ios::out | ios::app);
	} 
	else
	{
		file.open(name, ios::out);
	}
	if (file.is_open())
	{
		file << fixed << setprecision(2) << disY << setw(15) << ratio1 
			<< setw(15) << ratio2 << setw(15) << ratio3 
			<< setw(15) << ratio4 << setw(15) << ratio5 
			<< setw(15) << ratio6 << endl;
	}
	file.close();
}

void DCS::DCSCalibrationRobustness::saveResult(Mat& leftImage, Mat& rightImage, Mat& depthImage, string pathPrefix, int num, char* mode, int index, int count)
{
	char name[LENGTH];
	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d", pathPrefix.c_str(), mode, index);
	mkDir(name);

	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\main%d_rectify_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, leftImage);

	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\sub%d_rectify_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, rightImage);

	//Mat lineMat;
	//drawImageLine(leftImage, rightImage, lineMat);
	//sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\mainSub%d_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	//imwrite(name, lineMat);

	sprintf_s(name, LENGTH, "%s\\result\\%s\\%d\\main%d_depth_%s_%d_%d.jpg", pathPrefix.c_str(), mode, index, num, mode, index, count);
	imwrite(name, depthImage);
}

void DCS::DCSCalibrationRobustness::drawImageLine(Mat& leftImage, Mat& rightImage, Mat& outImage)
{
	if (leftImage.rows != rightImage.rows || leftImage.cols != rightImage.cols)
		return; 

	int rows = leftImage.rows;
	int cols = leftImage.cols;
	int cols2 = 2 * cols;
	outImage = Mat::zeros(rows, cols2, CV_8UC3);  
	Rect r(0, 0, cols, rows);
	Mat dstroi = outImage(Rect(0, 0, r.width, r.height));
	leftImage(r).convertTo(dstroi, dstroi.type(), 1, 0);
	dstroi = outImage(Rect(cols, 0, cols, rows));
	rightImage(r).convertTo(dstroi, dstroi.type(), 1, 0);

	for (int j = 0; j < rows; j += 16)
		line(outImage, Point(0, j), Point(cols2 - 1, j), Scalar(0, 255, 0));
}

void DCS::DCSCalibrationRobustness::mkDir(char *path)
{

	int ipathLength = strlen(path);
	int ileaveLength = 0;
	int iCreatedLength = 0;
	char szPathTemp[LENGTH] = { 0 };
	for (int i = 0; (NULL != strchr(path + iCreatedLength, '\\')); i++)
	{
		ileaveLength = strlen(strchr(path + iCreatedLength, '\\')) - 1;
		iCreatedLength = ipathLength - ileaveLength;
		strncpy_s(szPathTemp, path, iCreatedLength);
		_mkdir(szPathTemp);
	}
	if (iCreatedLength < ipathLength)
	{
		_mkdir(path);
	}
}

DCS_ERR_CODE DCS::DCSCalibrationRobustness::generateDepth(Mat &leftImg, Mat &rightImg, Mat &outImg, Mat &resizeLeft, Mat &map11, Mat &map12, int &maxDisparity)
{
	//imput lImg and rImg is CV_8UC3
	Mat left, right;
	cvtColor(leftImg, left, CV_BGR2GRAY);  //0.085s
	cvtColor(rightImg, right, CV_BGR2GRAY);

	GaussianBlur(left, left, Size(3, 3), 0.9);  //0.05s
	GaussianBlur(right, right, Size(3, 3), 0.9);

	//init MBMElas object
	MBMElas::parameters param;
	MBMElas melas(param);

	// get image width and height
	int width = left.cols;
	int height = left.rows;

	//init disparity 
	Mat leftDisparity = Mat::zeros(left.size(), CV_8UC1);
	Mat rightDisparity = Mat::zeros(left.size(), CV_8UC1);
	Mat maxMncc = Mat::ones(left.size(), CV_32FC1) * (-1);


	//get left and right disparity (1s)
	melas.getDisparity(left, right, leftDisparity, rightDisparity, maxMncc, leftImg, rightImg);

	// allocate memory for disparity images
	const int dims[3] = { width, height, width }; // bytes per line = width
	float* D1_data = (float*)malloc(width*height*sizeof(float));
	float* D2_data = (float*)malloc(width*height*sizeof(float));
	float* D_data = (float*)malloc(width*height*sizeof(float));

	Mat initLeftDisparity, initRightDisparity;
	leftDisparity.convertTo(initLeftDisparity, CV_32FC1);
	rightDisparity.convertTo(initRightDisparity, CV_32FC1);

	memcpy(D1_data, initLeftDisparity.data, width * height * sizeof(float));
	memcpy(D2_data, initRightDisparity.data, width * height * sizeof(float));

	// refine left disparity (spend 0.053s)
	melas.refineLeftDisparity(D1_data, D2_data, D_data, dims, left, maxMncc, leftDisparity);

	Mat Dis = Mat(left.rows, left.cols, CV_32FC1, D_data);
	Mat temp;
	Dis.convertTo(temp, CV_8UC1);

	normalize(temp, temp, 0, 255, NORM_MINMAX, CV_8UC1);

	Mat oriDepth = translationDisparityMap(temp, map11, map12);
	dtFilter(resizeLeft, oriDepth, outImg, 120, 0.8 * 255, DTF_NC, 3);

	free(D1_data);
	free(D2_data);
	free(D_data);


	return DCS_SUCCESS;
}

cv::Mat DCS::DCSCalibrationRobustness::translationDisparityMap(Mat& depthmap, Mat& map11, Mat& map12)
{
	Mat dstmap = Mat::zeros(depthmap.rows, depthmap.cols, CV_8UC1);
	Mat refermap = Mat::zeros(depthmap.rows, depthmap.cols, CV_32SC1);
	if (depthmap.cols != map11.cols || depthmap.rows != map11.rows || depthmap.cols != map12.cols || depthmap.rows != map12.rows)
	{
		return dstmap;
	}
	for (int i = 0; i < depthmap.rows; i++)
	{
		for (int j = 0; j<depthmap.cols; j++)
		{
			int x = cvRound(map11.at<float>(i, j));
			int y = cvRound(map12.at<float>(i, j));
			if (x>depthmap.cols - 1)
			{
				continue;//x = depthmap.cols-1;
			}
			if (x < 0)
			{
				continue;
			}
			if (y > depthmap.rows - 1)
			{
				continue;//y = depthmap.rows-1;
			}
			if (y < 0)
			{
				continue;
			}
			uchar tmp = depthmap.at<uchar>(i, j);
			if (tmp < 0 || tmp>255)
			{
				continue;
			}
			dstmap.at<uchar>(y, x) = depthmap.at<uchar>(i, j);
			refermap.at<int>(y, x) = 1;
		}
	}
	for (int i = 0; i < refermap.rows; i++)
	{
		for (int j = 0; j < refermap.cols; j++)
		{
			if (refermap.at<int>(i, j) == 0)
			{

				for (int k = 1; k <= 5; k++)
				{
					int a = i;
					int b = j;
					if (a == refermap.rows - k)
					{
						a--;
					}
					else if (a == k - 1)
					{
						a++;
					}
					if (b == refermap.cols - k)
					{
						b--;
					}
					else if (b == k - 1)
					{
						b++;
					}
					//field-8;
					if (refermap.at<int>(a + k, b) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a + k, b);
						break;
					}
					else if (refermap.at<int>(a - k, b) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a - k, b);
						break;
					}
					else if (refermap.at<int>(a, b + k) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a, b + k);
						break;
					}
					else if (refermap.at<int>(a, b - k) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a, b - k);
						break;
					}
					else if (refermap.at<int>(a + k, b + k) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a + k, b + k);
						break;
					}
					else if (refermap.at<int>(a + k, b - k) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a + k, b - k);
						break;
					}
					else if (refermap.at<int>(a - k, b - k) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a - k, b - k);
						break;
					}
					else if (refermap.at<int>(a - k, b + k) == 1)
					{
						dstmap.at<uchar>(i, j) = dstmap.at<uchar>(a - k, b + k);
						break;
					}
				}
			}
		}
	}
	return dstmap;
}

void DCS::DCSCalibrationRobustness::setResource(int res)
{
	m_res = res;
}


