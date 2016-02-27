#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<string.h>
#include<fstream>
#include<C:\Pixelate_project\BeliefState\BeliefState.h>

#define hue_bot 106
#define hue_bot1 9
#define hue_box 67
#define hue_ball 0
#define sat_bot 128
#define sat_bot1 158
#define sat_box 160
#define sat_ball 0
#define lum_bot 240
#define lum_bot1 238
#define lum_box 154
#define lum_ball 0
#define thresh_hue_bot 15
#define thresh_hue_bot1 8
#define thresh_hue_box 20
#define thresh_hue_ball 0
#define thresh_lum_bot 64
#define thresh_lum_bot1 30
#define thresh_lum_box 31
#define thresh_lum_ball 0
#define thresh_sat_bot 31
#define thresh_sat_bot1 18
#define thresh_sat_box 72
#define thresh_sat_ball 0
#define hue_arrow
#define sat_arrow
#define lum_arrow
#define thresh_hue_arrow
#define thresh_sat_arrow
#define thresh_lum_arrow
#define hue_boxD 37
#define sat_boxD 149
#define lum_boxD 212
#define thresh_boxD_hue 17
#define thresh_boxD_sat 28
#define thresh_boxD_lum 22

#define erode_n 1
#define dilate_n 2
using namespace std;
using namespace BS;
using namespace cv;

namespace BS
{
	Point contour_finding(Mat,int,int,int,int,int,int,const string c);
	void erosion(Mat);
	void dilation(Mat);

	BeliefState::BeliefState()
	{
		botPosX=0;
		botPosY=0;
		ballPosX=0;
		ballPosY=0;
		boxPosX=0;
		boxPosY=0;
		
		//initialising for lpf
		past=Point(botPosX,botPosY);  
		smoothData=Point(botPosX,botPosY);		
	}
	void BeliefState::update(Mat frame)
	{
		Mat frame1=frame.clone();
		Mat frame2=frame.clone();
		Mat frame3=frame.clone();
		Mat frame4=frame.clone();
		//Mat frame5=frame.clone();

		calc_botPos( frame1,"BOT");
		calc_ballPos(frame2,"BALL");
		calc_boxPos(frame3,"BOX");
		calc_boxDestPos(frame4,"BOX_DEST");
		calc_angle();

		//ofstream file;
		//file.open("botPos.txt",ios::app);
		//file<<botPosX<<" "<<botPosY<<" "; 
		//
		////using the low pass filter
 	//    lpf();
		//
		//file<<botPosX<<" "<<botPosY<<endl;
		//file.close();

	}

	void BeliefState::lpf()
	{
		float beta=0.25;
		/*
			**** the working foemula is y[i-1]=beta*x[i]+(1-beta)*y[i-1] , it is ban exponentially weighted moving avg like formula
			**** x[i] being the current reading and y[i-1] being the last value , beta being a constant factor 
		*/
		smoothData.x=past.x+(beta*(botPosX-past.x));
		smoothData.y=past.y+(beta*(botPosY-past.y));
		
		botPosX=smoothData.x;
		botPosY=smoothData.y;
		
		past=Point(botPosX,botPosY);
	}

	void BeliefState::calc_angle()
	{
		float angle;
		Point c1=Point(botPosX1,botPosY1);
		Point c2=Point(botPosX2,botPosY2);
		if(abs(c1.x-c2.x)<5) 
			{
				if(c1.y<c2.y)
				angle=90;
				else angle=-90;
			}
		else 
			{
				angle=atan2((c2.y-c1.y),(c2.x-c1.x));
				angle=angle*180/3.14;
			}
		botAngle=angle;
	}
	void BeliefState::calc_vel(Mat frame)
	{
		/*
		   calculates the velocity of the bot 
		*/
	}
	void BeliefState::calc_botPos(Mat frame,const string c)
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the bot position variables
		*/
		Mat frame1=frame.clone();
		Point center=contour_finding(frame,hue_bot,sat_bot,lum_bot,thresh_hue_bot,thresh_sat_bot,thresh_lum_bot,c);
		botPosX1=center.x;
		botPosY1=center.y;
		string c1="BOT1";
		center=contour_finding(frame1,hue_bot1,sat_bot1,lum_bot1,thresh_hue_bot1,thresh_sat_bot1,thresh_lum_bot1,c1);
		botPosX2=center.x;
		botPosY2=center.y;	
		botPosX=(botPosX1+botPosX2)/2;
		botPosY=(botPosY1+botPosY2)/2;
	}
	void BeliefState::calc_ballPos(Mat frame,const string c)
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the ball position variables
		*/
		Point center=contour_finding(frame,hue_ball,sat_ball,lum_ball,thresh_hue_ball,thresh_sat_ball,thresh_lum_ball,c);
		ballPosX=center.x;
		ballPosY=center.y;
	}
	void BeliefState::calc_boxPos(Mat frame,const string c)
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the box position variables
		*/
		Point center=contour_finding(frame,hue_box,sat_box,lum_box,thresh_hue_box,thresh_sat_box,thresh_lum_box,c);
		boxPosX=center.x;
		boxPosY=center.y;
	}
	void BeliefState::calc_boxDestPos(Mat frame,const string c)
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the box position variables
		*/
		Point center=contour_finding(frame,hue_boxD,sat_boxD,lum_boxD,thresh_boxD_hue,thresh_boxD_sat,thresh_boxD_lum,c);
		boxDestPosX=center.x;
		boxDestPosY=center.y;
	}
	void BeliefState::calc_arrowPos(Mat frame,const string c)
	{
		/*
		  1.color detection
		  2.contour finding
		  3.center determination
		  4.update the arrow position variables
		*/
	}

	////////////////////////////////////////////////////////////////////////

	Mat color_detection(Mat img,int h,int s,int l,int t_h,int t_s,int t_l)
{
	cvtColor(img,img,CV_BGR2HLS);
	vector<Mat> channels;
	split(img,channels);
	
	//Mat img2(img.rows,img.cols,CV_8UC1,Scalar(0));;
	
	
		Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
		
		for(int i=0;i<img.rows;i++)
		{
		
	
			for(int j=0;j<img.cols;j++)
			{
				if((channels[0].at<uchar>(i,j)<h+t_h && channels[0].at<uchar>(i,j)>h-t_h)  && (channels[1].at<uchar>(i,j)<s+t_s && channels[1].at<uchar>(i,j)>s-t_s) && (channels[2].at<uchar>(i,j)<l+t_l && channels[2].at<uchar>(i,j)>l-t_l) ) img1.at<uchar>(i,j)=255;
			}

		
		
		}
		//imshow("hsl color ",img);
		//imshow("hsl color detection",img1);
		return img1;
	
}

 Point BeliefState::contour_finding(Mat src,int h,int s,int l,int t_h,int t_s,int t_l,const string c)
{
	 Mat gray;
	
    //cvtColor(src, gray, CV_BGR2GRAY);
	 gray=color_detection(src,h,s,l,t_h,t_s,t_l);
	 erosion(gray);
	 dilation(gray);
    //threshold(gray, gray,200, 255,THRESH_BINARY_INV); //Threshold the gray
    //imshow("gray",gray);

	 //eroding
	 //erosion(gray);

	int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;
    vector<vector<Point>> contours; // Vector for storing contour
    vector<Vec4i> hierarchy;
    findContours( gray, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    // iterate through each contour.
    for( int i = 0; i< contours.size(); i++ )
    {
        //  Find the area of contour
        double a=contourArea( contours[i],false); 
        if(a>largest_area){
            largest_area=a;
			//cout<<i<<" area  "<<a<<endl;
            // Store the index of largest contour
            largest_contour_index=i;               
            // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
        }
    }
	//************************finding corners of the bounding_rect******************
	string check="BOX";
	if(!check.compare(c))
	{
		//cout<<"i am here :D  "<<c<<" "<<bounding_rect.width<<" "<<bounding_rect.height<<endl;
		boxCorners[0].x=bounding_rect.x;
		boxCorners[0].y=bounding_rect.y;
		boxCorners[1].x=bounding_rect.x+bounding_rect.width;
		boxCorners[1].y=bounding_rect.y;
		boxCorners[2].x=bounding_rect.x;
		boxCorners[2].y=bounding_rect.y+bounding_rect.height;
		boxCorners[3].x=bounding_rect.x+bounding_rect.width;
		boxCorners[3].y=bounding_rect.y+bounding_rect.height;

	}
	
	/*for(int i=0;i<4;i++)
	{
		cout<<boxCorners[i].x<<","<<boxCorners[i].y<<endl;
	}*/

	string check1="BOX_DEST";
	if(!check1.compare(c))
	{
		//cout<<"i am here :D  "<<c<<" "<<bounding_rect.width<<" "<<bounding_rect.height<<endl;
		boxDestCorners[0].x=bounding_rect.x;
		boxDestCorners[0].y=bounding_rect.y;
		boxDestCorners[1].x=bounding_rect.x+bounding_rect.width;
		boxDestCorners[1].y=bounding_rect.y;
		boxDestCorners[2].x=bounding_rect.x;
		boxDestCorners[2].y=bounding_rect.y+bounding_rect.height;
		boxDestCorners[3].x=bounding_rect.x+bounding_rect.width;
		boxDestCorners[3].y=bounding_rect.y+bounding_rect.height;

	}
	
	//************************* finding center of contour *******************************
	
	vector<Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ ){
		mu[i] = moments( contours[i], false );
	}
	
	

	// Mass center
	vector<Point2f> mc( contours.size() );
	  for( int i = 0; i < contours.size(); i++ ){ 
		mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
	  }

	  //displaying the center of the largest contour

	  Point center;

	  for (int i = 0; i < contours.size(); i++)
	  {
		  if(i==largest_contour_index) 
		    {
				center.x=mc[i].x;
				center.y=mc[i].y;

			}
	  }
	///////////////////////////////////////////////////

    //Scalar color( 255,255,255);  // color of the contour in the
    //Draw the contour and rectangle
	Mat output(src.rows,src.cols,CV_8UC3,Scalar(0,0,0));;
	//drawContours( output, contours,largest_contour_index, color, CV_FILLED,8,hierarchy);
    rectangle(output, bounding_rect,  Scalar(0,255,0),2, 8,0);
	namedWindow( c, CV_WINDOW_AUTOSIZE );
	imshow( c, output );
	
	return center;
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