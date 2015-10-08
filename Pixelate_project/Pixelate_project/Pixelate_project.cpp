// Pixelate_project.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include<thread>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<C:\Pixelate_project\BeliefState\BeliefState.h>
#include<C:\Pixelate_project\Comm\Comm.h>
#include<C:\Pixelate_project\Planner\planner.h>

#include<iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

using namespace BS;


int main()
{
	VideoCapture cap(0);
	BeliefState state;
	Mat frame;
	frame=imread("field.png",CV_LOAD_IMAGE_COLOR);
	imshow("src",frame);
	state.update(frame);
	cout<<"botPos : ("<<state.botPosX<<","<<state.botPosY<<") ballPos: ("<<state.ballPosX<<","<<state.ballPosY<<") boxPos: ("<<state.boxPosX<<","<<state.boxPosY<<")"<<endl;
	
	/*while(1)
	{
		
		Mat frame;
		cap>>frame;
		state.update(frame);
		cout<<"botPos : ("<<state.botPosX<<","<<state.botPosY<<") ballPos: ("<<state.ballPosX<<","<<state.ballPosY<<") boxPos: ("<<state.boxPosX<<","<<state.boxPosY<<")"<<endl;

	}*/
	
	waitKey(0);
	return 0;
}

