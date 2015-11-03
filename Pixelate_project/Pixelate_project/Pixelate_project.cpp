// Pixelate_project.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include<thread>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/video/tracking.hpp"
#include<Windows.h>

//#include<C:\Pixelate_project\Kalman\Kalman.h>
//#include<C:\Pixelate_project\BeliefState\BeliefState.h>
#include<C:\Pixelate_project\Comm\Comm.h>
#include<C:\Pixelate_project\Planner\planner.h>


#include<iostream>
#include <stdio.h>
 
#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )
 

using namespace cv;
using namespace std;
using namespace Kalman;
using namespace BS;
using namespace Plan;

int main()
{
	VideoCapture cap(0);
	BeliefState state;
	
	//***********************initialize kalman*************************
	
	//Mat frame_init;
	//cap>>frame_init;
	//
	//state.update(frame_init);
	///*Planner plan(state,frame_init);
	//cout<<"destinations "<<plan.at_start<<" "<<plan.des1.x<<" "<<plan.des1.y<<" "<<plan.des3.x<<" "<<plan.des3.y<<endl;
	//*/
	//Point init_pos;
	//init_pos.x=state.botPosX;
	//init_pos.y=state.botPosY;
	//KalmanFilter kf=kalman(frame_init,init_pos); 
	//Mat_<float> measurement(2,1);measurement.setTo(Scalar(0));
	//		
	////image to show tracking
	//Mat img(600,800,CV_8SC3);
	//vector<Point> posv,kalmanv;
	//posv.clear();
	//kalmanv.clear();

	//*****************************************************
	
	Mat frame;
	frame=imread("field.png",CV_LOAD_IMAGE_COLOR);
	imshow("src",frame);
	state.update(frame);
	Planner plan(state,frame); 
	cout<<"grid position of bot "<<plan.bot_pos.x<<","<<plan.bot_pos.y<<endl;
	cout<<"destinations "<<plan.at_start<<" 1: "<<plan.des1.x<<","<<plan.des1.y<<" 2: "<<plan.des2.x<<","<<plan.des2.y<<" 3: "<<plan.des3.x<<","<<plan.des3.y<<endl;
	cout<<"botPos : ("<<state.botPosX<<","<<state.botPosY<<") ballPos: ("<<state.ballPosX<<","<<state.ballPosY<<") boxPos: ("<<state.boxPosX<<","<<state.boxPosY<<") boxDestPos: ("<<state.boxDestPosX<<","<<state.boxDestPosY<<")"<<endl;
	

	//**************************************************

	//while(1)
	//{
	//	
	//	Mat frame;
	//	cap>>frame;
	//	imshow("test",frame);
	//	
	//	// updating the belief state

	//	state.update(frame);
	//	cout<<"botPos : ("<<state.botPosX<<","<<state.botPosY<<") ballPos: ("<<state.ballPosX<<","<<state.ballPosY<<") boxPos: ("<<state.boxPosX<<","<<state.boxPosY<<")"<<endl;
	//	
	//	//*****************************using the kalman filter*******************************
	//	
	//	Point pos;
	//	pos.x=state.botPosX;
	//	pos.y=state.ballPosY;
	//	

	//	Mat prediction=kf.predict();
	//	Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
	//	
	//	measurement(0)=pos.x;
	//	measurement(1)=pos.y;

	//	Mat estimated=kf.correct(measurement);

	//	Point  statePt(estimated.at<float>(0),estimated.at<float>(1));
	//	Point  measPt(measurement(0),measurement(1));

	//	imshow("kalman Points",img);
	//	 img = Scalar::all(0);
	//	cout<<"statePt"<<statePt<<endl;

	//	posv.push_back(measPt);
	//	kalmanv.push_back(statePt);
	//	drawCross( statePt,Scalar(255,255,255),5);
	//	drawCross( measPt, Scalar(0,0,255), 5 );

	//	for (int i = 0; i < posv.size()-1; i++)
	//	{
	//		line(img,posv[i],posv[i+1],Scalar(255,255,0),1);

	//	}
	//	for (int i = 0; i < posv.size()-1; i++)
	//	{
	//		line(img,kalmanv[i],kalmanv[i+1],Scalar(0,155,255),1);

	//	}

	//	//***********************************************************************************
	//	if(waitKey(30) >= 0) break;
	//}
	
	waitKey(0);
	return 0;
}

