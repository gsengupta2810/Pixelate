#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

//#include<C:\Pixelate_project\BeliefState\BeliefState.h>
#include<C:\Pixelate_project\Kalman\Kalman.h>

#include<iostream>
#include<vector>
using namespace std;
using namespace BS;
using namespace cv;

namespace Plan
{
	class Planner
	{
		public:
			int grid[28][44]; //grid for path planning
			Point des1;
			Point des2;
			Point des3;
			Point end;
			Point start;
			Point bot_pos; 

			bool des1_reached;
			bool des2_reached;
		    bool des3_reached;
			bool at_start;
			//functions 
			Planner()
			{
			}
			Planner(BeliefState,Mat);
			void update(BeliefState,Mat);
			void show_grid(); 
			Mat wall_detect(Mat);
			Mat box(Mat);
			Mat box_dest(Mat);
			void update_grid(Mat,BeliefState);
	};
}