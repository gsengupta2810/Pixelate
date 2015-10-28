#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<C:\Pixelate_project\BeliefState\BeliefState.h>

using namespace BS;
using namespace std;
using namespace cv;

namespace Kalman
{
	KalmanFilter kalman(Mat img,Point pos)
	{
		KalmanFilter kf(4,2,0);
		//Point pos;
		kf.transitionMatrix=*(Mat_<float>(4,4)<<1,0,1,0,   0,1,0,1,   0,0,1,0,  0,0,0,1);
		Mat_<float> measurement(2,1);measurement.setTo(Scalar(0));

		kf.statePre.at<float>(0) =pos.x;
		kf.statePre.at<float>(1)=pos.y;
		kf.statePre.at<float>(2)=0;
		kf.statePre.at<float>(3)=0;

		setIdentity(kf.measurementMatrix);
		setIdentity(kf.processNoiseCov,Scalar::all(1e-4));
		setIdentity(kf.measurementNoiseCov,Scalar::all(10));
		setIdentity(kf.errorCovPost,Scalar::all(.1));
		return kf;
	}
}