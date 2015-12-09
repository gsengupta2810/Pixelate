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
//#include<C:\Pixelate_project\Comm\arduino.cpp>
//#include<C:\Pixelate_project\Planner\planner.h>
#include<C:\Pixelate_project\Djikstras\Djikstras.h>

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
using namespace djikstras;


int main()
{
	/*BeliefState state;
		
	Mat frame;
	frame=imread("field.png",CV_LOAD_IMAGE_COLOR);
	imshow("src",frame);
	state.update(frame);
	Planner plan(state,frame);
	plan.update(state,frame);
	cout<<"grid position of bot "<<plan.bot_pos.x<<","<<plan.bot_pos.y<<endl;
	cout<<"destinations "<<plan.at_start<<" 1: "<<plan.des1.x<<","<<plan.des1.y<<" 2: "<<plan.des2.x<<","<<plan.des2.y<<" 3: "<<plan.des3.x<<","<<plan.des3.y<<endl;
	cout<<"bot angle :("<<state.botAngle<<") "<<"botPos : ("<<state.botPosX<<","<<state.botPosY<<") ballPos: ("<<state.ballPosX<<","<<state.ballPosY<<") boxPos: ("<<state.boxPosX<<","<<state.boxPosY<<") boxDestPos: ("<<state.boxDestPosX<<","<<state.boxDestPosY<<")"<<endl;
	
	plan.show_grid();*/

	//**************************************************
	//VideoCapture cap(0);
	//Mat frame_init;
	//cap>>frame_init;
	//BeliefState state;
	//state.update(frame_init);
	//Planner plan(state,frame_init);
	//plan.update(state,frame_init);
	//Djikstras  djiks;
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
	//	plan.update(state,frame);
	//	djiks.run(state);
	//
	//	if(waitKey(30) >= 0) break;
	//}
	
	waitKey(0);
	return 0;
}

