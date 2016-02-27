#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<C:\Pixelate_project\Kalman\Kalman.h>

using namespace BS;
using namespace std;
using namespace cv;

namespace BS
{
	low_pass_filter::low_pass_filter(BeliefState state)
	{
		past=Point(0,0);//Point(state.botPosX,state.botPosY);
		smoothData=Point(0,0);//Point(state.botPosX,state.botPosY);
	}

	void low_pass_filter::lpf(BeliefState state)
	{
		float beta=0.025;
		
		//the working foemula is y[i-1]=beta*x[i]+(1-beta)*y[i-1] , it is ban exponentially weighted moving avg like formula
		// x[i] being the current reading and y[i-1] being the last value , beta being a constant factor 

		smoothData.x=past.x+(beta*(state.botPosX-past.x));
		smoothData.y=past.y+(beta*(state.botPosY-past.y));
		
		state.botPosX=smoothData.x;
		state.botPosY=smoothData.y;
		
		past=Point(state.botPosX,state.botPosY);

	}
}