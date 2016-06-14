#ifndef _DCS_CALI_STRUCT_H_
#define _DCS_CALI_STRUCT_H_

#include <opencv2/opencv.hpp>

using namespace cv;

struct CaliParams
{
	cv::Mat Intrinsic_L;//left camera intrinsic matrix;
	cv::Mat Intrinsic_R;//right camera intrinsic matrix;
	cv::Mat Distortion_L;//left camera distort matrix;
	cv::Mat Distortion_R;//right camera distort matrix;
	cv::Mat Rotation;//rotation;
	cv::Mat Translation;//translation;
	float Err;//reprojection error;

	cv::Mat Intrinsic_L_ini;//左摄像头内参3*3矩阵;
	cv::Mat Intrinsic_R_ini;//右摄像头内参3*3;
	cv::Mat Distortion_L_ini;//左摄像头畸变参数4*1;
	cv::Mat Distortion_R_ini;//右摄像头畸变参数4*1;
	cv::Mat Rotation_ini;//旋转矩阵3*3矩阵;
	cv::Mat Translation_ini;//平移矩阵3*1矩阵;


	CaliParams()
	{
		Intrinsic_L = cv::Mat::zeros(3, 3, CV_32FC1);
		Intrinsic_R = cv::Mat::zeros(3, 3, CV_32FC1);
		Distortion_L = cv::Mat::zeros(8, 1, CV_32FC1);
		Distortion_R = cv::Mat::zeros(8, 1, CV_32FC1);
		Rotation = cv::Mat::zeros(3, 3, CV_32FC1);
		Translation = cv::Mat::zeros(3, 1, CV_32FC1);
		Err = -1.0;
		Intrinsic_L_ini = cv::Mat::zeros(3, 3, CV_32FC1);
		Intrinsic_R_ini = cv::Mat::zeros(3, 3, CV_32FC1);
		Distortion_L_ini = cv::Mat::zeros(8, 1, CV_32FC1);
		Distortion_R_ini = cv::Mat::zeros(8, 1, CV_32FC1);
		Rotation_ini = cv::Mat::zeros(3, 3, CV_32FC1);
		Translation_ini = cv::Mat::zeros(3, 1, CV_32FC1);
	}
	unsigned short checksum;
};

#endif //!_DCS_CALI_STRUCT_H_