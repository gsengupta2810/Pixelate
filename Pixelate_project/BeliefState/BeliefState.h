#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<vector>
using namespace std;
using namespace cv;

namespace BS
{
	// TODO: create a template class that contains the data types Point or Vector2D
	
	class BeliefState
	{
		 public:
		
		// position variables
		short int botPosX;
		short int botPosY;
		short int ballPosX;
		short int ballPosY;
		short int boxPosX;
		short int boxPosY;
		short int boxDestPosX;
		short int boxDestPosY;
		Point boxCorners[4];
		Point boxDestCorners[4];
		

		/*short int boxDestX;
		short int boxDestY;*/

		vector<short int> arrowPosX;
		vector<short int> arrowPosY;
		vector<short int> arrowDirection;
		
		//velocit variables
		float bot_velX;
		float bot_velY;
		float bot_omega;
	    
		
		//updation and initiation functions 
		
		BeliefState();
		void update(Mat);
		Point contour_finding(Mat,int,int,int,int,int,int,const string c);

		//calculation functions 

		void calc_vel(Mat);
		void calc_botPos(Mat,const string);
		void calc_ballPos(Mat,const string);
		void calc_boxPos(Mat,const string);
		void calc_arrowPos(Mat,const string);
		void calc_boxDestPos(Mat,const string);

	};

}