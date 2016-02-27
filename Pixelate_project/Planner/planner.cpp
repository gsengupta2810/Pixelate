#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<fstream>
#include<deque>
#include<C:\Pixelate_project\Planner\planner.h>
//#include<C:\Pixelate_project\BeliefState\BeliefState.h>
//#include<C:\Pixelate_project\Kalman\Kalman.h>

using namespace std;
using namespace BS;
using namespace cv;

#define hue_wall 208
#define sat_wall 132
#define lum_wall 149
#define thresh_hue 49
#define thresh_sat 54
#define thresh_lum 54
#define hue_boxD 37
#define sat_boxD 149
#define lum_boxD 212
#define thresh_boxD_hue 17
#define thresh_boxD_sat 22
#define thresh_boxD_lum 28
#define hue_box 67
#define sat_box 160
#define lum_box 154
#define thresh_hue_box 20
#define thresh_lum_box 31
#define thresh_sat_box 72

#define erode_n 1
#define dilate_n 5
#define wall_param 25

namespace Plan
{
	 void erosion(Mat);	 
	 void dilation(Mat);
	Planner::Planner(BeliefState state,Mat img)
	{
		for(int i=0;i<28;i++)
		{
			for(int j=0;j<44;j++)
			{
				grid[i][j]=0;
			}
		}

		des1_reached=false;
		des2_reached=false;
		des3_reached=false;
		at_start=true;

		//todo convert the following to grid values
		start.x=(int)(state.botPosX*44/img.cols);
		start.y=(int)(state.botPosY*28/img.rows);
		des3.x=(int)(state.ballPosX*44/img.cols);
		des3.y=(int)(state.ballPosY*28/img.rows);
		
		Mat img1=img.clone();
		update_grid(img1,state);
	}

	void Planner::update(BeliefState state,Mat img)
	{
		
		//todo convert the following to grid values
		
		des3.x=(int)(state.ballPosX*44/img.cols);
		des3.y=(int)(state.ballPosY*28/img.rows);
		
		Mat img1=img.clone();
		update_grid(img1,state);
		update_wall_properly();
	}
	void Planner::update_wall_properly()
	{
		for(int i=0;i<28;i++)
		{
			for(int j=0;j<44;j++)
			{
				if(grid[i][j]==1)
				{
					int countx=0,county=0;
					for(int k=0;k<5;k++)
					{
						if(grid[i+k][j]==1) countx++;
						if(grid[i-k][j]==1) countx++; 
						if(grid[i][j+k]==1) county++;
						if(grid[i][j-k]==1) county++;
					}
					if(countx<3 && county<3)
						grid[i][j]=0;
				}
				if(grid[i][j]==0)
				{
					int countx=0,county=0;
					for(int k=0;k<5;k++)
					{
						if(grid[i+k][j]==1) countx++;
						if(grid[i-k][j]==1) countx++;
						if(grid[i][j+k]==1) county++;
						if(grid[i][j-k]==1) county++;
					}
					if(countx>3 || county>3)
						grid[i][j]=0;
				}
			}
		}
		/*int arrx[44]={0},arry[28]={0};
		for(int i=0;i<28;i++)
		{
			if(grid[i][20]==1 ||grid[i][22]==1 || grid[i][18]==1 || grid[i][15]==1 || grid[i][28]==1) arry[i]++;
		}
		for(int i=0;i<44;i++)
		{
			if(grid[10][i]==1||grid[14][i]==1||grid[18][i]==1||grid[12][i]==1||grid[16][i]==1||grid[20][i]==1) arrx[i]++;
		}
		int last_wallx=0,last_wally=0,first_wallx=0,first_wally=0;
		int i=28;
		while(i--)
		{
			if(arry[i]>0)
				{
					last_wally=i;
					break;
				}
		}
		i=0;
		while(i++)
		{
			if(arry[i]>0)
				{
					first_wally=i;
					break;
				}
		}
		i=0;
		while(i++)
		{
			if(arrx[i]>0)
				{
					first_wallx=i;
					break;
				}
		}
		i=44;
		while(i--)
		{
			if(arrx[i]>0)
				{
					last_wallx=i;
					break;
				}
		}
		cout<<first_wallx<<" "<<last_wallx<<" "<<first_wally<<" "<<last_wally<<endl;*/	
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
		erosion(img1);
		dilation(img1);
		 return img1;
	}
	
	void Planner:: show_grid()
	{
		ofstream file;
		file.open("grid.txt",ios::trunc);
		
		for(int i=0;i<28;i++)
		{
			for(int j=0;j<44;j++)
			{
				//cout<<grid[i][j]<<"             ";
				file<<grid[i][j]<<"  ";
			}
			//cout<<endl;
			file<<endl;
		}

		file.close();
		

	}

	void Planner::update_grid(Mat img,BeliefState state)
	{
		//wall detection

		//*********************grid division and updation for walls*****************************
		
				Mat output1=img.clone(),output2=img.clone(); 

				Mat gray;
				gray=wall_detect(output2);
				
				imshow("walls",gray);
				for (int i=0;i<gray.rows;i++)
				{
					for(int j=0;j<gray.cols;j++)
					{
						if(gray.at<uchar>(i,j)>0) grid[i*28/gray.rows][j*44/gray.cols]++;
					}
				}
				for(int i=0;i<44;i++)
				{
					for(int j=0;j<28;j++)
					{
						if(grid[j][i]>=wall_param)grid[j][i]=1;
						else grid[j][i]=0;
					}
				}				    

		//*************calculating grid of box**************
			Point temp[5];	
			int maxX,maxY;
			temp[0].x=(int)(state.boxPosX*44/img.cols);
			temp[0].y=(int)(state.boxPosY*28/img.rows);
			//cout<<temp[0].y<<endl;
			for(int i=1;i<5;i++)
			{
				temp[i].x=(int)(state.boxCorners[i-1].x*44/img.cols);
				temp[i].y=(int)(state.boxCorners[i-1].y*28/img.rows);
				
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
			grid[des1.y][des1.x]=2;
			//*************calculating grid of box destination**************
			
			temp[0].x=(int)(state.boxDestPosX*44/img.cols);
			temp[0].y=(int)(state.boxDestPosY*28/img.rows);
			//cout<<temp[0].y<<endl;
			for(int i=1;i<5;i++)
			{
				temp[i].x=(int)(state.boxDestCorners[i-1].x*44/img.cols);
				temp[i].y=(int)(state.boxDestCorners[i-1].y*28/img.rows);
				
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
			grid[des2.y][des2.x]=3;
		
			//*************calculating grid position of bot**************
			
			bot_pos.x=(int)(state.botPosX*44/img.cols);
			bot_pos.y=(int)(state.botPosY*28/img.rows);
			grid[bot_pos.y][bot_pos.x]=5;
			grid[des3.y][des3.x]=4;
			//*********************************************************

   }
   void erosion(Mat img)
	 {
		 static int n=erode_n;
		 //namedWindow("eroding",WINDOW_AUTOSIZE);
		 //createTrackbar("erode_size","eroding",&n,7);
		 Mat element=getStructuringElement(MORPH_RECT,Size(2*n+1,2*n+1),Point(n,n));
		 erode(img,img,element);
		 //imshow("eroding",img);	
	 }
	 void dilation(Mat img)
	 { 
		 static int n=dilate_n;
		 //namedWindow("dilating",WINDOW_AUTOSIZE);
		 //createTrackbar("dilate_size","dilating",&n,7);
		 Mat element=getStructuringElement(MORPH_RECT,Size(2*n+1,2*n+1),Point(n,n));
		 dilate(img,img,element);
		 //imshow("eroding",img);	
	 }
	
}