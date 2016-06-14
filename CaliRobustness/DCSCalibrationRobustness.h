#ifndef _DCS_CALIBRATION_ROBUSTNESS_H_
#define _DCS_CALIBRATION_ROBUSTNESS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include "DCSDefine.h"
#include "DCSCaliStruct.h"

using namespace std;
using namespace cv;

#define CHESSBOARD 0
#define NATURAL    1

namespace DCS
{
	class DCSCalibrationRobustness
	{
	public:
		DCSCalibrationRobustness();
		~DCSCalibrationRobustness();
	
	public:
		//////////////////////////////////////////////////////////////////////////
		//read calibration parameters and convert it to opencv Mat
		//@param   :pCalipath calibration file path
		//////////////////////////////////////////////////////////////////////////
		void getCaliParamsMat(const char* pCalipath);

		//////////////////////////////////////////////////////////////////////////
		//find checkerboard corner points
		//@param image       :input image need to find corner points
		//@param image       :chessboard specification size
		//@param win         :sub pixel optimization window size
		//@param vPoint2DSet :output the found corner points
		//@return            :DCS_SUCCESS if success other failure
		//////////////////////////////////////////////////////////////////////////
		DCS_ERR_CODE findCheckerboardPoints(Mat& image, Size patternSize, Size win, vector<Point2f>& vPoint2DSet);

		//////////////////////////////////////////////////////////////////////////
		//draw the found corner point use different color for different precision
		//@param image :input orignal image and output image with color point
		//@param pts   :corner point coordinate
		//@param color :use color to draw point
		//@param num   :if needed, draw the point's serial number
		//////////////////////////////////////////////////////////////////////////
		void drawImagePoint(Mat& pImg, Point2f& pts, Scalar color, int Num = -1);

		//////////////////////////////////////////////////////////////////////////
		//draw lines on images to observe rectify result
		//@param leftImage  :input rectified main image
		//@param rightImage :input rectified sub image
		//@param outImage   :output draw line image
		//////////////////////////////////////////////////////////////////////////
		void drawImageLine(Mat& leftImage, Mat& rightImage, Mat& outImage);

		//////////////////////////////////////////////////////////////////////////
		//rectify the main image and sub image use calibration parameters
		//@param leftImg          :input orignal main image and output the rectified image
		//@param rightImg         :input orignal sub image  and output the rectified image
		//@param M1               :Intrinsic_L scale for main image
		//@param M2               :Intrinsic_L scale for sub image
		//@param D1               :Distortion_L for main image
		//@param D2               :Distortion_L for sub image
		//@param R                :Ratation Mat
		//@param T                :Translation Mat
		//@param map11            :output param for anti-rectify
		//@param map12            :output param for anti-rectify
		//@param maxDisparity     :max disparity
		////////////////////////////////////////////////////////////////////////// 
		void rectify(Mat& leftImg, Mat& rightImg, Mat& M1, Mat& M2, Mat& D1, Mat& D2, Mat& R, Mat &T, Mat &map11, Mat &map12, int &maxDisparity);

		//////////////////////////////////////////////////////////////////////////
		//scale intrinsic parameters for different image size
		//@param mainW            :main image width
		//@param mainH            :main image height
		//@param subW             :sub image width
		//@param subH             :sub image height
		//@param resizeRatioLeft  :calibration params need to scale resizeRatioLeft for main image
		//@param resizeRatioRight :calibration params need to scale resizeRatioRight for sub image
		//@return                 :DCS_SUCCESS if success other failure
		DCS_ERR_CODE scaleIntrinsicParams(int mainW, int mainH, int subW, int subH, double resizeRatioLeft, double resizeRatioRight);

		//////////////////////////////////////////////////////////////////////////
		//generate depth map 
		//@param leftImg          :input rectified main image
		//@param rightImg         :input rectified sub image
		//@param outImg           :output depth map
		//@param resizeLeft       :not rectified resized main image
		//@param map11            :input param for anti-rectify
		//@param map12            :input param for anti-rectify
		//@param maxDisparity     :max disparity
		//@return                 :DCS_SUCCESS if success other failure
		////////////////////////////////////////////////////////////////////////// 
		DCS_ERR_CODE generateDepth(Mat &leftImg, Mat &rightImg, Mat &outImg, Mat &resizeLeft, Mat &map11, Mat &map12, int &maxDisparity);

		//////////////////////////////////////////////////////////////////////////
		//anti-rectify depth map 
		//@param depthmap          :input depth map
		//@param map11             :input param for anti-rectify
		//@param map12             :input param for anti-rectify
		//@return                  :anti-rectify depth map
		//////////////////////////////////////////////////////////////////////////
		Mat translationDisparityMap(Mat& depthmap, Mat& map11, Mat& map12);

		//////////////////////////////////////////////////////////////////////////
		//set test resource
		//@param res  : = CHESSBOARD, find chessboard corners and compute deltaY
		//            : = NATURAL, generate depth
		void setResource(int res);

	public:
		//////////////////////////////////////////////////////////////////////////
		//test ratation params influence for rectify
		//@param leftImage     :input original main image
		//@param rightImage    :input original sub image
		//@param index         : = 0, direction x ratation
		//                     : = 1, direction y ratation
		//                     : = 2, direction z ratation
		//@param range         :params change range
		//@param step          :params change step
		//@param pathPrefix    :path to save result
		//@param num           :image index
		//////////////////////////////////////////////////////////////////////////
		void testRotation(Mat& leftImage, Mat& rightImage, int index, double range, double step, string pathPrefix, int num);

		//////////////////////////////////////////////////////////////////////////
		//test translation params influence for rectify
		//@param leftImage     :input original main image
		//@param rightImage    :input original sub image
		//@param index         : = 0, direction x translation
		//                     : = 1, direction y translation
		//                     : = 2, direction z translation
		//@param range         :params change range
		//@param step          :params change step
		//@param pathPrefix    :path to save result
		//@param num           :image index
		//////////////////////////////////////////////////////////////////////////
		void testTranslation(Mat& leftImage, Mat& rightImage, int index, double range, double step, string pathPrefix, int num);

		//////////////////////////////////////////////////////////////////////////
		//test focal length params influence for rectify
		//@param leftImage     :input original main image
		//@param rightImage    :input original sub image
		//@param index         : = 0, main camera focal length change
		//                     : = 1, sub camera focal length change
		//                     : = 2, main and sub camera focal length change
		//@param range         :params change range
		//@param step          :params change step
		//@param pathPrefix    :path to save result
		//@param num           :image index
		//////////////////////////////////////////////////////////////////////////
		void testFocalLength(Mat& leftImage, Mat& rightImage, int index, double range, double step, string pathPrefix, int num);

		//////////////////////////////////////////////////////////////////////////
		//read calibration parameters from file
		//@param  pCalipath:calibration file path
		//@param  cali     :output calibration parameters
		//////////////////////////////////////////////////////////////////////////
		void readCaliParams(const char* pCalipath, CaliParams& cali);

		//////////////////////////////////////////////////////////////////////////
		//draw corner points use different color for different precision andsave result to disk
		//@param leftImage     :input rectified main image
		//@param rightImage    :input rectified sub image
		//@param pathPrefix    :path to save result
		//@param num           :image index
		//@param mode          :test mode, "R":rotation, "T":translation, 
		//                                 "F":camera focallength
		//@param index         :index of test direction
		//@param count         :number of test param result
		//////////////////////////////////////////////////////////////////////////
		void saveResult(Mat& leftImage, Mat& rightImage, string pathPrefix, int num, char* mode, int index, int count);

		//////////////////////////////////////////////////////////////////////////
		//draw corner points use different color for different precision andsave result to disk
		//@param leftImage     :input rectified main image
		//@param rightImage    :input rectified sub image
		//@param depthImage     :input depth image
		//@param pathPrefix     :path to save result
		//@param num            :image index
		//@param mode           :test mode, "R":rotation, "T":translation, 
		//                                 "F":camera focallength
		//@param index          :index of test direction
		//@param count          :number of test param result
		//////////////////////////////////////////////////////////////////////////
		void saveResult(Mat& leftImage, Mat& rightImage, Mat& depthImage, string pathPrefix, int num, char* mode, int index, int count);

		//////////////////////////////////////////////////////////////////////////
		//create folder
		//@param path :folder path
		//////////////////////////////////////////////////////////////////////////
		void mkDir(char *path);

	private:
		Mat       m_INL;  //Intrinsic_L
		Mat       m_INR;  //Intrinsic_R
		Mat       m_M1;   //scaled Intrinsic_L Mat
		Mat       m_M2;   //scaled Intrinsic_R Mat
		Mat       m_DL;   //Distortion_L
		Mat       m_DR;   //Distortion_R
		Mat       m_R;    //Rotation
		Mat       m_T;    //Translation
		int       m_res;  //test mode
	};
}

#endif // !_DCS_CALIBRATION_ROBUSTNESS_H_