#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<C:\Pixelate_project\Planner\planner.h>

using namespace std;
using namespace BS;
using namespace Plan;

namespace djikstras
{
	class Djikstras : public Planner
	{
		public:
		//int heuristic_grid[28][44];
		Point start;
		Point destination;
		bool going_up;
		bool going_down;
		bool going_right;
		bool going_left;
		bool forward;
		bool backward;
		bool turn_r;
		bool turn_l;
		bool destination_1_in_range;
		bool destination_2_in_range;
		bool destination_3_in_range;
		bool return2path;
		bool dest_towards_right;
		bool dest_towards_left;
		bool dest_in_front;
		bool box_picked;
		bool box_released;
		bool ball_kicked;
		bool straight_check;

		//sending variable
		char data;
		//functions
		Djikstras();
		void motion(Planner);
		void initiate();
		void set(BeliefState,Planner);
		//void set_start_dest(Point,Point,Planner);
		void run(BeliefState,Planner);
		void com(Planner);
		void check_straight(BeliefState,Planner);
	};
}