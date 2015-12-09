#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<C:\Pixelate_project\Djikstras\Djikstras.h>
//#include<C:\Pixelate_project\Comm\arduino.cpp>

#include<conio.h>
#include"tserial.h"
#include"bot_control.h"

#define turn 5


using namespace std;
using namespace BS;
using namespace cv;

namespace djikstras
{
	serial comm;
	void s_data(char);

	Djikstras::Djikstras()
	{
	}

	void Djikstras::initiate()
	{
		 going_up=false;
		 going_down=false;
		 going_right=false;
		 going_left=false;
		 forward=false;
		 backward=false;
		 turn_r=false;
		 turn_l=false;
		 destination_1_in_range=false;
		 destination_2_in_range=false;
		 destination_3_in_range=false;
		 return2path=false;
		 dest_towards_left=false;
		 dest_towards_right=false;
		 dest_in_front=false;
		 box_picked=false;
		 box_released=false;
		 ball_kicked=false;
		 

	}
	void Djikstras::run(BeliefState state)
	{
		initiate();
		set(state);
		motion();
		check_straight(state);
		com();
	}
	void Djikstras::check_straight(BeliefState state)
	{
		if(going_up==true && forward==true && turn_l==false && turn_r==false)
		{
            if(state.botAngle>-75 )
			{
				char data='a';
				s_data(data);
			}
			 if(state.botAngle<-105 )
			{
				char data='z';
				s_data(data);
			}
		}
		if(going_down==true && forward==true && turn_l==false && turn_r==false)
		{
            if(state.botAngle>105 )
			{
				char data='a';
				s_data(data);
			}
			 if(state.botAngle<75 )
			{
				char data='z';
				s_data(data);
			}
		}
		if(going_right==true && forward==true && turn_l==false && turn_r==false)
		{
            if(state.botAngle>15 )
			{
				char data='a';
				s_data(data);
			}
			 if(state.botAngle<-15 )
			{
				char data='z';
				s_data(data);
			}
		}
		if(going_left==true && forward==true && turn_l==false && turn_r==false)
		{
            if(state.botAngle<-165 )
			{
				char data='a';
				s_data(data);
			}
			 if(state.botAngle>165 )
			{
				char data='z';
				s_data(data);
			}
		}

	}
	void Djikstras::motion()
	{
		if(going_up==true)
			{
				if(grid[bot_pos.y-4][bot_pos.x]!=1 ) forward=true;
			}
		if(going_down==true)
			{
				if(grid[bot_pos.y+4][bot_pos.x]!=1) forward=true;
			}
		if(going_right==true)
			{
				if(grid[bot_pos.y][bot_pos.x+4]!=1) forward=true;
			}
		if(going_left==true)
			{
				if(grid[bot_pos.y][bot_pos.x-4]!=1) forward=true;
			}
		if(forward==true)
		{
			if(going_up==true)
			{
				if((grid[bot_pos.y-4][bot_pos.x]==1) || (grid[bot_pos.y-5][bot_pos.x]==1))
				{
					forward=false;
					if(grid[bot_pos.y][bot_pos.x+2]!=1 && grid[bot_pos.y][bot_pos.x+3]!=1 && grid[bot_pos.y][bot_pos.x+4]!=1 && grid[bot_pos.y][bot_pos.x+5]!=1)
						turn_r=true;
				
					
					if(grid[bot_pos.y][bot_pos.x-2]!=1 && grid[bot_pos.y][bot_pos.x-3]!=1 && grid[bot_pos.y][bot_pos.x-4]!=1 && grid[bot_pos.y][bot_pos.x-5]!=1)
						turn_l=true;
				}
			}
			if(going_down==true)
			{
				if((grid[bot_pos.y+5][bot_pos.x]==1)&&(grid[bot_pos.y+4][bot_pos.x]==1))
				{
					forward=false;
					if(grid[bot_pos.y][bot_pos.x+2]!=1 && grid[bot_pos.y][bot_pos.x+3]!=1 && grid[bot_pos.y][bot_pos.x+4]!=1 && grid[bot_pos.y][bot_pos.x+5]!=1)
						turn_r=true;
				
					
					if(grid[bot_pos.y][bot_pos.x-2]!=1 &&grid[bot_pos.y][bot_pos.x-3]!=1 &&grid[bot_pos.y][bot_pos.x-4]!=1 &&grid[bot_pos.y][bot_pos.x-5]!=1 )
						turn_l=true;
				}
			}
			if(going_right==true)
			{
				if((grid[bot_pos.y][bot_pos.x+5]==1)&&(grid[bot_pos.y][bot_pos.x+4]==1))
				{
					forward=false;
					if(grid[bot_pos.y-2][bot_pos.x]!=1&& grid[bot_pos.y-3][bot_pos.x]!=1 && grid[bot_pos.y-4][bot_pos.x]!=1 && grid[bot_pos.y-5][bot_pos.x]!=1)
						turn_l=true;
				
					if(grid[bot_pos.y+2][bot_pos.x]!=1 && grid[bot_pos.y+3][bot_pos.x]!=1 && grid[bot_pos.y+4][bot_pos.x]!=1 && grid[bot_pos.y+5][bot_pos.x]!=1)
						turn_r=true;
				}
			}
			if(going_left==true)
			{
				if((grid[bot_pos.y][bot_pos.x-5]==1)&&(grid[bot_pos.y][bot_pos.x-4]==1))
				{
					forward=false;
					if(grid[bot_pos.y-2][bot_pos.x]!=1&& grid[bot_pos.y-3][bot_pos.x]!=1 && grid[bot_pos.y-4][bot_pos.x]!=1 && grid[bot_pos.y-5][bot_pos.x]!=1)
						turn_r=true;
				
					if(grid[bot_pos.y+2][bot_pos.x]!=1 && grid[bot_pos.y+3][bot_pos.x]!=1 && grid[bot_pos.y+4][bot_pos.x]!=1 && grid[bot_pos.y+5][bot_pos.x]!=1)
						turn_l=true;
				}
			}
		}
		if(des1.y==bot_pos.y && abs(des1.x-bot_pos.x)<=5 && des1_reached==false) 
		{
			destination_1_in_range=true;

			if(des1.x>bot_pos.x && going_up==true)
				dest_towards_right=true;
			else dest_towards_left=true;

			if(des1.x>bot_pos.x && going_down==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des1.x<bot_pos.x && going_up==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des1.x<bot_pos.x && going_down==true)
				dest_towards_right=true;
			else dest_towards_left=true;
		}
		else if(abs(des1.y-bot_pos.y)<=2 && des1.y==bot_pos.y && des1_reached==false)
		{
			dest_in_front=true;
			destination_1_in_range=true;
		}
		if(des2.y==bot_pos.y && abs(des2.x-bot_pos.x)<=5 && des1_reached==false) 
		{
			destination_2_in_range=true;
			if(des2.x>bot_pos.x && going_up==true)
				dest_towards_right=true;
			else dest_towards_left=true;

			if(des2.x>bot_pos.x && going_down==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des2.x<bot_pos.x && going_up==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des2.x<bot_pos.x && going_down==true)
				dest_towards_right=true;
			else dest_towards_left=true;
		}
		else if(abs(des2.y-bot_pos.y)<=1 && des2.y==bot_pos.y && des2_reached==false)
		{
			dest_in_front=true;
			destination_2_in_range=true;
		}
		if(des3.y==bot_pos.y && abs(des3.x-bot_pos.x)<=5 && des3_reached==false) 
		{
			destination_3_in_range=true;
			if(des3.x>bot_pos.x && going_up==true)
				dest_towards_right=true;
			else dest_towards_left=true;

			if(des3.x>bot_pos.x && going_down==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des3.x<bot_pos.x && going_up==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des3.x<bot_pos.x && going_down==true)
				dest_towards_right=true;
			else dest_towards_left=true;
		}
		else if(abs(des3.x-bot_pos.y)<=2 && des3.y==bot_pos.y && des3_reached==false)
		{
			dest_in_front=true;
			destination_3_in_range=true;
		}

	}
	void Djikstras:: set(BeliefState state)
	{
		if(turn_l!= true && turn_l!= true)
		{
			if(state.botAngle>-50 && state.botAngle<50)
			{
				going_right=true;
			}
			else if(state.botAngle>50 && state.botAngle<130)
			{
				going_down=true;
			}
			else if((state.botAngle<-130 && state.botAngle>-180) && (state.botAngle>130 && state.botAngle<180))
			{
				going_left=true;
			}
			else if(state.botAngle<-50 && state.botAngle>-130)
			{
				going_up=true;
			}
		}
	}
	void Djikstras::com()
	{
		char data='S';
		//destination 1
		if(destination_1_in_range==true)
		{
			if(dest_towards_left==true)
			{
				int move=0; 
				data='l';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
				//TODO: go forward
				while(abs(bot_pos.x-des1.x)>2)
				{
					data='f';
					s_data(data);
					move++;
				}
				if(abs(bot_pos.x-des1.x)<=2)
				{
					data='g';
					s_data(data);
					box_picked=true;
					while(move--)
					{
						data='b';
						s_data(data);
					}
					data='r';
					s_data(data);
				}
				
			}
			else if(dest_towards_right==true)
			{
				int move=0; 
				data='r';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
				//TODO: go forward
				while(abs(bot_pos.x-des1.x)>2)
				{
					data='f';
					s_data(data);
					move++;
				}
				if(abs(bot_pos.x-des1.x)<=2)
				{
					data='g';
					s_data(data);
					box_picked=true;
					while(move--)
					{
						data='b';
						s_data(data);
					}
					data='l';
					s_data(data);
				}
				
			}
			else if(dest_in_front==true)
			{
				data='g';
				s_data(data);
			}
			if(box_picked==true) 
			{
				destination_1_in_range=false;
				des1_reached=true;
			}
		}
		//destination 2
		if(destination_2_in_range==true)
		{
			if(dest_towards_left==true)
			{
				int move=0; 
				data='l';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
				//TODO: go forward
				while(abs(bot_pos.x-des2.x)>2)
				{
					data='f';
					s_data(data);
					move++;
				}
				if(abs(bot_pos.x-des2.x)<=2)
				{
					data='c';
					s_data(data);
					box_released=true;
					while(move--)
					{
						data='b';
						s_data(data);
					}
					data='r';
					s_data(data);
				 }
				
			}
			else if(dest_towards_right==true)
			{
				int move=0; 
				data='r';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
				//TODO: go forward
				while(abs(bot_pos.x-des2.x)>2)
				{
					data='f';
					s_data(data);
					move++;
				}
				if(abs(bot_pos.x-des2.x)<=2)
				{
					data='c';
					s_data(data);
					box_released=true;
					while(move--)
					{
						data='b';
						s_data(data);
					}
					data='l';
					s_data(data);
				}
				
			}
			if(dest_in_front==true)
			{
				data='c';
				s_data(data);
			}
			if(box_released==true) 
			{
				destination_2_in_range=false;
				des2_reached=true;
			}
		}
		//**************destination 3******************
		if(destination_3_in_range==true)
		{
			if(dest_towards_left==true)
			{
				int move=0; 
				data='l';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
				//TODO: go forward
				while(abs(bot_pos.x-des3.x)>2)
				{
					data='f';
					s_data(data);
					move++;
				}
				if(abs(bot_pos.x-des3.x)<=2)
				{
					data='k';
					s_data(data);
					ball_kicked=true;
					while(move--)
					{
						data='b';
						s_data(data);
					}
					data='r';
					s_data(data);
				 }
				
			}
			else if(dest_towards_right==true)
			{
				int move=0; 
				data='r';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
				//TODO: go forward
				while(abs(bot_pos.x-des3.x)>2)
				{
					data='f';
					s_data(data);
					move++;
				}
				if(abs(bot_pos.x-des3.x)<=2)
				{
					data='k';
					s_data(data);
					ball_kicked=true;
					while(move--)
					{
						data='b';
						s_data(data);
					}
					data='l';
					s_data(data);
				}
				
			}
			if(dest_in_front==true)
			{
				data='k';
				s_data(data);
			}
			if(ball_kicked==true) 
			{
				destination_3_in_range=false;
				des3_reached=true;
			}
		}
		//******************************
		if( forward==true)
		{
			data='f';
			s_data(data);
		}
		if(forward==false)
		{
			if(turn_l==true)
			{
				data='l';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
			}
			if(turn_r==true)
			{
				data='r';
				for(int i=0;i<turn;i++)
				{
					s_data(data);
				}
			}
		}
	}
	void s_data(char data)
	{
	
	
		comm.startDevice("COM3",9600);
	
 		comm.send_data(data);
	
		comm.stopDevice();
	
	
		//getch();
	}
}