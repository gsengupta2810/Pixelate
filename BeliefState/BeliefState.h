//#include "stdafx.h"
#include<iostream>
#include<vector>
using namespace std;

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
		vector<short int> arrowPosX;
		vector<short int> arrowPosY;
		vector<short int> arrowDirection;
		
		//velocit variables
		float bot_velX;
		float bot_velY;
		float bot_omega;
	    
		//updation and initiation functions 
		BeliefState();
		void update();
		
		//calculation functions 

		void calc_vel();
		void calc_botPos();
		void calc_ballPos();
		void calc_boxPos();
		void calc_arrowPos();


	};

}