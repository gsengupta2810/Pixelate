#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<C:\Pixelate_project\Planner\planner.h>
//#include<C:\Pixelate_project\BeliefState\BeliefState.h>
//#include<C:\Pixelate_project\Kalman\Kalman.h>

using namespace std;
using namespace BS;
using namespace cv;

#define hue_wall 175
#define sat_wall 87
#define lum_wall 197
#define thresh_hue 33
#define thresh_sat 103
#define thresh_lum 62
#define hue_boxD 42
#define sat_boxD 149
#define lum_boxD 113
#define thresh_boxD_hue 45
#define thresh_boxD_sat 26
#define thresh_boxD_lum 15
#define hue_box 26
#define sat_box 89
#define lum_box 197
#define thresh_hue_box 34
#define thresh_lum_box 62
#define thresh_sat_box 102


namespace Plan
{
	Planner::Planner(BeliefState state,Mat img)
	{
		for(int i=0;i<14;i++)
		{
			for(int j=0;j<22;j++)
			{
				grid[i][j]=1;
			}
		}

		des1_reached=false;
		des2_reached=false;
		des3_reached=false;
		at_start=true;

		//todo convert the following to grid values
		start.x=(int)(state.botPosX*22/img.cols);
		start.y=(int)(state.botPosY*14/img.rows);
		des3.x=(int)(state.ballPosX*22/img.cols);
		des3.y=(int)(state.ballPosY*14/img.rows);
		
		Mat img1=img.clone();
		update_grid(img1,state);
	}
	Mat Planner::wall_detect(Mat img)
	{
		cvtColor(img,img,CV_BGR2HLS);
		vector<Mat> channels;
		split(img,channels);
	
		Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
		
		for(int i=0;i<img.rows;i++)
		{
		
			for(int j=0;j<img.cols;j++)
			{
				if((channels[0].at<uchar>(i,j)<hue_wall+thresh_hue && channels[0].at<uchar>(i,j)>hue_wall-thresh_hue)  && (channels[1].at<uchar>(i,j)<sat_wall+thresh_sat && channels[1].at<uchar>(i,j)>sat_wall-thresh_sat) && (channels[2].at<uchar>(i,j)<lum_wall+thresh_lum && channels[2].at<uchar>(i,j)>lum_wall-thresh_lum) ) img1.at<uchar>(i,j)=255;
			}
	
		}

		 return img1;
	}
	Mat Planner::box_dest(Mat img)
	{
		cvtColor(img,img,CV_BGR2HLS);
		vector<Mat> channels;
		split(img,channels);
	
		Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
		
		for(int i=0;i<img.rows;i++)
		{
		
			for(int j=0;j<img.cols;j++)
			{
				if((channels[0].at<uchar>(i,j)<hue_boxD+thresh_boxD_hue && channels[0].at<uchar>(i,j)>hue_boxD-thresh_boxD_hue)  && (channels[1].at<uchar>(i,j)<sat_boxD+thresh_boxD_sat && channels[1].at<uchar>(i,j)>sat_box-thresh_boxD_sat) && (channels[2].at<uchar>(i,j)<lum_boxD+thresh_boxD_lum && channels[2].at<uchar>(i,j)>lum_boxD-thresh_boxD_lum) ) img1.at<uchar>(i,j)=255;
			}
	
		}

		 return img1;
	}
	Mat Planner::box(Mat img)
	{
		cvtColor(img,img,CV_BGR2HLS);
		vector<Mat> channels;
		split(img,channels);
	
		Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
		
		for(int i=0;i<img.rows;i++)
		{
		
			for(int j=0;j<img.cols;j++)
			{
				if((channels[0].at<uchar>(i,j)<hue_box+thresh_hue_box && channels[0].at<uchar>(i,j)>hue_box-thresh_hue_box)  && (channels[1].at<uchar>(i,j)<sat_box+thresh_sat_box && channels[1].at<uchar>(i,j)>sat_box-thresh_sat_box) && (channels[2].at<uchar>(i,j)<lum_box+thresh_lum_box && channels[2].at<uchar>(i,j)>lum_box-thresh_lum_box) ) img1.at<uchar>(i,j)=255;
			}
	
		}

		 return img1;
	}
	void Planner::update_grid(Mat img,BeliefState state)
	{
		//wall detection

		Mat img1=img.clone();
		Mat walls(img.rows,img.cols,CV_8UC1,Scalar(0));
		walls=wall_detect(img1);
		

		//*********************grid division and updation*****************************

		int thresh=10;
		int x=0,y=0,max=0,maxPosX=0,maxPosY=0,max1=0,maxPosX1=0,maxPosY1=0;
		for(int i=4;i<img.rows-8;i+=(int)(img.rows/14))
		{
			y=0;
			for(int j=4;j<img.cols-8;j+=(int)(img.cols/22))
			{
				int count_white_wall=0,count_black_wall=0,count_white_boxD=0,count_black_boxD=0,count_white_box=0,count_black_box=0;
				for(int k=0;k<(int)((img.cols)/22);k++)
				{
					for(int l=0;l<(int)((img.rows)/14);l++)
					{
						if(walls.at<uchar>(i+l,j+k)==255) count_white_wall++;
						else count_black_wall++;
					}
				}
				//for walls
				if(count_white_wall)
				{
					if(count_black_wall/count_white_wall>thresh)
					{
						grid[x][y]=0; // wall detected
						y++;
					}
				}
			}
			x++;
		}
		
			    

		//*************calculating grid of box**************
			Point temp[5];	
			int maxX,maxY;
			temp[0].x=(int)(state.boxPosX*22/img.cols);
			temp[0].y=(int)(state.boxPosY*14/img.rows);
			cout<<temp[0].y<<endl;
			for(int i=1;i<5;i++)
			{
				temp[i].x=(int)(state.boxCorners[i-1].x*22/img.cols);
				temp[i].y=(int)(state.boxCorners[i-1].y*14/img.rows);
				
			}
			
			int max_index=0;
			int count[5]={0};
			for(int i=0;i<5;i++)
			{
				for(int j=i;j<5;j++)
				{
					if((temp[i].x==temp[j].x)&&(temp[i].y=temp[j].y))
						count[i]++;
				}
			}
			for(int i=0;i<5;i++)
			{
				if(max_index<count[i]) 
				{
					max_index=count[i];

				}
				
			}	
			for(int i=0;i<5;i++)
			{
				if(count[i]>=2) 
					max_index=0;
			}
			maxX=temp[max_index].x;
			maxY=temp[max_index].y;
			des1.x=maxX;
			des1.y=maxY;
			grid[des1.x][des1.y]=2;
			//*************calculating grid of box destination**************
			
			temp[0].x=(int)(state.boxDestPosX*22/img.cols);
			temp[0].y=(int)(state.boxDestPosY*14/img.rows);
			cout<<temp[0].y<<endl;
			for(int i=1;i<5;i++)
			{
				temp[i].x=(int)(state.boxDestCorners[i-1].x*22/img.cols);
				temp[i].y=(int)(state.boxDestCorners[i-1].y*14/img.rows);
				
			}
			
		
			for(int i=0;i<5;i++)
			{
				for(int j=i;j<5;j++)
				{
					if((temp[i].x==temp[j].x)&&(temp[i].y=temp[j].y))
						count[i]++;
				}
			}
			for(int i=0;i<5;i++)
			{
				if(max_index<count[i]) 
				{
					max_index=count[i];

				}
				
			}	
			for(int i=0;i<5;i++)
			{
				if(count[i]>=2) 
					max_index=0;
			}
			maxX=temp[max_index].x;
			maxY=temp[max_index].y;
			des2.x=maxX;
			des2.y=maxY;
			grid[des2.x][des2.y]=3;
		
			//*************calculating grid position of bot**************
			
			bot_pos.x=(int)(state.botPosX*22/img.cols);
			bot_pos.y=(int)(state.botPosX*14/img.rows);

			//*********************************************************

   }
}