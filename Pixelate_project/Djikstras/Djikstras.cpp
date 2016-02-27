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

#define turn 15
#define wall_dist 10 
#define wall_dist_front 4
#define deviation_angle 12

using namespace std;
using namespace BS;
using namespace cv;

namespace djikstras
{
	serial comm;
	void s_data(char);
	static int tick=0;

	Djikstras::Djikstras()
	{
		//initiate();
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
		 straight_check=true;
		 

	}
// the execution function*****************************
	
	void Djikstras::run(BeliefState state,Planner plan)
	{
		if(((straight_check==false && tick!=0) && (turn_l==true || turn_r==true))||(tick==0 && (turn_l==true || turn_r==true)))
		{
			if(tick==0) tick++;

			if(turn_l==true)
			{
				if(going_up==true)
				{
					
					 going_up=false;
					 going_down=false;
					 going_right=false;
					 going_left=true;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_up=true;
				}
				if(going_down==true)
				{
					 going_up=false;
					 going_down=false;
					 going_right=true;
					 going_left=false;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_down=true;
				}
				if(going_right==true)
				{
					 going_up=true;
					 going_down=false;
					 going_right=false;
					 going_left=false;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_right=true;
				}
				if(going_left==true)
				{
					 going_up=false;
					 going_down=true;
					 going_right=false;
					 going_left=false;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_left=true;
				}
			}
			else if(turn_r==true)
			{
				if(going_up==true)
				{
					 going_up=false;
					 going_down=false;
					 going_right=true;
					 going_left=false;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_up=true;
				}
				if(going_down==true)
				{
					 going_up=false;
					 going_down=false;
					 going_right=false;
					 going_left=true;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_down=true;
				}
				if(going_right==true)
				{
					 going_up=false;
					 going_down=true;
					 going_right=false;
					 going_left=false;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_right=true;
				}
				if(going_left==true)
				{
					 going_up=true;
					 going_down=false;
					 going_right=false;
					 going_left=false;
					 check_straight(state,plan);//checking the allignment of the bot 
					 going_left=true;
				}
			}
			
		}
		else
		{
			tick=0;
			initiate();
		
			set(state,plan);//sets the present direction of motion the bot 
		
			motion(plan);// to calculate the direction of motion in the next iteration according to current position 

			if(turn_l==false && turn_r==false)
			check_straight(state,plan);//checking the allignment of the bot 
		
			if(straight_check==true) // if allignment is straight proceed with the directions as set by motion 
			com(plan);//to senf the data as calculated by motion

		}
	}

//**********************checking allignment******************************

	void Djikstras::check_straight(BeliefState state,Planner plan)
	{
		cout<<"entering check "<<endl;
		if(going_up==true && forward==true && turn_l==false && turn_r==false)
		{
			if(state.botAngle>(-90+deviation_angle) )
			{
				data='z';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			else if(state.botAngle<-(90+deviation_angle) )
			{
				data='a';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			 else straight_check=true;
		}
		else if(going_down==true && forward==true && turn_l==false && turn_r==false)
		{
            if(state.botAngle>(90+deviation_angle) )
			{
				data='z';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			else if(state.botAngle<(90-deviation_angle))
			{
				data='a';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			 else straight_check=true;
		}
		else if(going_right==true && forward==true && turn_l==false && turn_r==false)
		{
            if(state.botAngle>deviation_angle )
			{
				data='z';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			else if(state.botAngle<-deviation_angle )
			{
				data='a';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			 else straight_check=true;
		}
		else if(going_left==true && forward==true && turn_l==false && turn_r==false)
		{
            if(state.botAngle>-(180-deviation_angle) && state.botAngle<0)
			{
				data='z';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			 else if(state.botAngle<(180-deviation_angle) && state.botAngle>0 )
			{
				data='a';
				for(int i=0;i<5;i++)
				s_data(data);
				straight_check=false;
			}
			 else straight_check=true;
		}
		
		if(straight_check==false)
		cout<<"not straight"<<endl;
		else cout<<"straight"<<endl;

	}
	void Djikstras::motion(Planner plan)
	{
		//**************************************conditions for moving forward*************************************
		
		if(going_up==true)
			{
				forward=true;
			}
		if(going_down==true)
			{
				forward=true;
			}
		if(going_right==true)
			{
				forward=true;
			}
		if(going_left==true)
			{
				forward=true;
			}

		//**************************************condition for walls in front*************************************

			if(going_up==true)
			{
				//cout<<"1"<<endl;
				for(int m=0;m<wall_dist_front && (plan.bot_pos.y-m)>0;m++)
				{
					if((plan.grid[plan.bot_pos.y-m][plan.bot_pos.x]==1)|| (plan.grid[plan.bot_pos.y-m][plan.bot_pos.x+1]==1) || (plan.grid[plan.bot_pos.y-m][plan.bot_pos.x-1]==1))
					{
						cout<<"wall in front"<<endl;
						forward=false;

						int count=0;
						for(int i=0;i<wall_dist && (plan.bot_pos.x+i)<44;i++)
						{
						if(plan.grid[plan.bot_pos.y][plan.bot_pos.x+i]==1)
							count++;
						}	
						int count1=0;
						for(int i=0;i<wall_dist && (plan.bot_pos.x-i)>0;i++)
						{
						if(plan.grid[plan.bot_pos.y][plan.bot_pos.x-i]==1)
							count1++;
						}	
						cout<<count<<" "<<count1<<endl;
						if(count<count1)
						turn_r=true;
						else turn_l=true;

						break;
					}
				}
			}
			else if(going_down==true)
			{
				//cout<<"2"<<endl;
				for(int m=0;m<wall_dist_front ;m++)
				{				
					if((plan.grid[plan.bot_pos.y+m][plan.bot_pos.x]==1) || (plan.grid[plan.bot_pos.y+m][plan.bot_pos.x+1]==1) || (plan.grid[plan.bot_pos.y+m][plan.bot_pos.x-1]==1))
					{
						cout<<"wall in front"<<endl;
						forward=false;

						int count=0;
						for(int i=0;i<wall_dist;i++)
						{
						if(plan.grid[plan.bot_pos.y][plan.bot_pos.x-i]==1)
							count++;
						}	
						int count1=0;
						for(int i=0;i<wall_dist;i++)
						{
						if(plan.grid[plan.bot_pos.y][plan.bot_pos.x+i]==1)
							count1++;
						}	

						if(count<count1)
						turn_r=true;
						else turn_l=true;

						break;
					}
				}
			}
			else if(going_right==true)
			{
				//cout<<"3"<<endl;
				for(int m=0;m<wall_dist_front && (plan.bot_pos.x+m)<44;m++)
				{
					if((plan.grid[plan.bot_pos.y][plan.bot_pos.x+m]==1)|| (plan.grid[plan.bot_pos.y+1][plan.bot_pos.x+m]==1) || (plan.grid[plan.bot_pos.y-1][plan.bot_pos.x+m]==1))
					{
						cout<<"wall in front"<<endl;
						forward=false;

						int count=0;
						for(int i=0;i<wall_dist;i++)
						{
						if(plan.
							grid[plan.bot_pos.y+i][plan.bot_pos.x]==1)
							count++;
						}	
						int count1=0;
						for(int i=0;i<wall_dist;i++)
						{
						if(plan.grid[plan.bot_pos.y-i][plan.bot_pos.x]==1)
							count1++;
						}	

						if(count<count1)
						turn_r=true;
						else turn_l=true;

						break;
					}
				}
			}
			else if(going_left==true)
			{
				cout<<"4";
				for(int m=0;m<wall_dist_front && (plan.bot_pos.x-m)>0;m++)
				{
					cout<<"5";
					if((plan.grid[plan.bot_pos.y][plan.bot_pos.x-m]==1)|| (plan.grid[plan.bot_pos.y+1][plan.bot_pos.x-m]==1) || (plan.grid[plan.bot_pos.y-1][plan.bot_pos.x-m]==1))
					{
						cout<<"wall in front"<<endl;
						forward=false;

						int count=0;
						for(int i=0;i<wall_dist;i++)
						{
						if(plan.grid[plan.bot_pos.y-i][plan.bot_pos.x]==1)
							count++;
						}	
						int count1=0;
						for(int i=0;i<wall_dist;i++)
						{
						if(plan.grid[plan.bot_pos.y+i][plan.bot_pos.x]==1)
							count1++;
						}	

						if(count<count1)
						turn_r=true;
						else turn_l=true;

						break;
					}
				}
			}
	

		//*****************************conditions for destinations in range ************************************

		if(abs(des1.y-plan.bot_pos.y) && abs(des1.x-plan.bot_pos.x)<=5 && plan.des1_reached==false) 
		{
			destination_1_in_range=true;

			if(des1.x>plan.bot_pos.x && going_up==true)
				dest_towards_right=true;
			else dest_towards_left=true;

			if(des1.x>plan.bot_pos.x && going_down==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des1.x<plan.bot_pos.x && going_up==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des1.x<plan.bot_pos.x && going_down==true)
				dest_towards_right=true;
			else dest_towards_left=true;
		}
		else if(abs(des1.y-plan.bot_pos.y)<=2 && abs(des1.x-plan.bot_pos.x)<2 && plan.des1_reached==false)
		{
			dest_in_front=true;
			destination_1_in_range=true;
		}
		if(abs(des2.y-plan.bot_pos.y)<2 && abs(des2.x-plan.bot_pos.x)<=5 && plan.des1_reached==false) 
		{
			destination_2_in_range=true;
			if(des2.x>plan.bot_pos.x && going_up==true)
				dest_towards_right=true;
			else dest_towards_left=true;

			if(des2.x>plan.bot_pos.x && going_down==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des2.x<plan.bot_pos.x && going_up==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des2.x<plan.bot_pos.x && going_down==true)
				dest_towards_right=true;
			else dest_towards_left=true;
		}
		else if(abs(des2.y-plan.bot_pos.y)<=2 && abs(des2.x-plan.bot_pos.x)<2 && plan.des2_reached==false)
		{
			dest_in_front=true;
			destination_2_in_range=true;
		}
		if(abs(des3.y-plan.bot_pos.y)<2 && abs(des3.x-plan.bot_pos.x)<=5 && plan.des3_reached==false) 
		{
			destination_3_in_range=true;
			if(des3.x>plan.bot_pos.x && going_up==true)
				dest_towards_right=true;
			else dest_towards_left=true;

			if(des3.x>plan.bot_pos.x && going_down==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des3.x<plan.bot_pos.x && going_up==true)
				dest_towards_left=true;
			else dest_towards_right=true;

			if(des3.x<plan.bot_pos.x && going_down==true)
				dest_towards_right=true;
			else dest_towards_left=true;
		}
		else if(abs(des3.x-plan.bot_pos.y)<=2 && abs(des3.y-plan.bot_pos.y)<2 && plan.des3_reached==false)
		{
			dest_in_front=true;
			destination_3_in_range=true;
		}
	//**************************************
	}
	void Djikstras:: set(BeliefState state,Planner plan)
	{
		if(turn_l!= true && turn_l!= true)
		{
			if(state.botAngle>-55 && state.botAngle<55)
			{
				going_right=true;
				cout<<"going right"<<endl;
			}
			else if(state.botAngle>50 && state.botAngle<125)
			{
				cout<<"going down"<<endl;
				going_down=true;
			}
			else if((state.botAngle<-125 && state.botAngle>-180) || (state.botAngle>125 && state.botAngle<180))
			{
				cout<<"going left"<<endl;
				going_left=true;
			}
			else if(state.botAngle<-65 && state.botAngle>-125)
			{
				going_up=true;
				cout<<"going up"<<endl;
			}
		}
	}
	void Djikstras::com(Planner plan)
	{
		//data='S';
		//*************************destination 1
		//if(destination_1_in_range==true)
		//{
		//	if(dest_towards_left==true)
		//	{
		//		int move=0; 
		//		data='l';
		//		for(int i=0;i<turn;i++)
		//		{
		//			s_data(data);
		//		}
		//		//TODO: go forward
		//		while(abs(plan.bot_pos.x-des1.x)>2)
		//		{
		//			data='f';
		//			s_data(data);
		//			move++;
		//		}
		//		if(abs(plan.bot_pos.x-des1.x)<=2)
		//		{
		//			data='g';
		//			s_data(data);
		//			box_picked=true;
		//			while(move--)
		//			{
		//				data='b';
		//				s_data(data);
		//			}
		//			data='r';
		//			s_data(data);
		//		}
		//		
		//	}
		//	else if(dest_towards_right==true)
		//	{
		//		int move=0; 
		//		data='r';
		//		for(int i=0;i<turn;i++)
		//		{
		//			s_data(data);
		//		}
		//		//TODO: go forward
		//		while(abs(plan.bot_pos.x-des1.x)>2)
		//		{
		//			data='f';
		//			s_data(data);
		//			move++;
		//		}
		//		if(abs(plan.bot_pos.x-des1.x)<=2)
		//		{
		//			data='g';
		//			s_data(data);
		//			box_picked=true;
		//			while(move--)
		//			{
		//				data='b';
		//				s_data(data);
		//			}
		//			data='l';
		//			s_data(data);
		//		}
		//		
		//	}
		//	else if(dest_in_front==true)
		//	{
		//		data='g';
		//		s_data(data);
		//	}
		//	if(box_picked==true) 
		//	{
		//		destination_1_in_range=false;
		//		des1_reached=true;
		//	}
		//}
		////*********************destination 2
		//if(destination_2_in_range==true)
		//{
		//	if(dest_towards_left==true)
		//	{
		//		int move=0; 
		//		data='l';
		//		for(int i=0;i<turn;i++)
		//		{
		//			s_data(data);
		//		}
		//		//TODO: go forward
		//		while(abs(plan.bot_pos.x-des2.x)>2)
		//		{
		//			data='f';
		//			s_data(data);
		//			move++;
		//		}
		//		if(abs(plan.bot_pos.x-des2.x)<=2)
		//		{
		//			data='c';
		//			s_data(data);
		//			box_released=true;
		//			while(move--)
		//			{
		//				data='b';
		//				s_data(data);
		//			}
		//			data='r';
		//			s_data(data);
		//		 }
		//		
		//	}
		//	else if(dest_towards_right==true)
		//	{
		//		int move=0; 
		//		data='r';
		//		for(int i=0;i<turn;i++)
		//		{
		//			s_data(data);
		//		}
		//		//TODO: go forward
		//		while(abs(plan.bot_pos.x-des2.x)>2)
		//		{
		//			data='f';
		//			s_data(data);
		//			move++;
		//		}
		//		if(abs(plan.bot_pos.x-des2.x)<=2)
		//		{
		//			data='c';
		//			s_data(data);
		//			box_released=true;
		//			while(move--)
		//			{
		//				data='b';
		//				s_data(data);
		//			}
		//			data='l';
		//			s_data(data);
		//		}
		//		
		//	}
		//	if(dest_in_front==true)
		//	{
		//		data='c';
		//		s_data(data);
		//	}
		//	if(box_released==true) 
		//	{
		//		destination_2_in_range=false;
		//		des2_reached=true;
		//	}
		//}
		////**************destination 3******************
		//if(destination_3_in_range==true)
		//{
		//	if(dest_towards_left==true)
		//	{
		//		int move=0; 
		//		data='l';
		//		for(int i=0;i<turn;i++)
		//		{
		//			s_data(data);
		//		}
		//		//TODO: go forward
		//		while(abs(plan.bot_pos.x-des3.x)>2)
		//		{
		//			data='f';
		//			s_data(data);
		//			move++;
		//		}
		//		if(abs(plan.bot_pos.x-des3.x)<=2)
		//		{
		//			data='k';
		//			s_data(data);
		//			ball_kicked=true;
		//			while(move--)
		//			{
		//				data='b';
		//				s_data(data);
		//			}
		//			data='r';
		//			s_data(data);
		//		 }
		//		
		//	}
		//	else if(dest_towards_right==true)
		//	{
		//		int move=0; 
		//		data='r';
		//		for(int i=0;i<turn;i++)
		//		{
		//			s_data(data);
		//		}
		//		//TODO: go forward
		//		while(abs(plan.bot_pos.x-des3.x)>2)
		//		{
		//			data='f';
		//			s_data(data);
		//			move++;
		//		}
		//		if(abs(plan.bot_pos.x-des3.x)<=2)
		//		{
		//			data='k';
		//			s_data(data);
		//			ball_kicked=true;
		//			while(move--)
		//			{
		//				data='b';
		//				s_data(data);
		//			}
		//			data='l';
		//			s_data(data);
		//		}
		//		
		//	}
		//	if(dest_in_front==true)
		//	{
		//		data='k';
		//		s_data(data);
		//	}
		//	if(ball_kicked==true) 
		//	{
		//		destination_3_in_range=false;
		//		des3_reached=true;
		//	}
		//}
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
					
					//cout<<"turning left "<<endl;
					s_data(data);
				}
				Sleep(500);
			}
			if(turn_r==true)
			{
				data='r';
				for(int i=0;i<turn;i++)
				{
					//cout<<"turning right "<<endl;
					s_data(data);
				}
				Sleep(500);
			}
		}
	}

	//function to send the data through mentioned com port
	void s_data(char send)
	{
	
	
		comm.startDevice("COM3",9600);
	
 		comm.send_data(send) ;
	
		comm.stopDevice();
	
	
		//getch();
	}
}