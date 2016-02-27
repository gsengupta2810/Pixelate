//#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<C:\Pixelate_project\BeliefState\BeliefState.h>

using namespace std;
using namespace cv;

namespace BS
{
	// low pass filter class 
	class low_pass_filter
	{
		public:

		Point past;
		Point smoothData;
		
		//function for lpf
		void lpf(BeliefState);
		low_pass_filter(BeliefState);
	};
}