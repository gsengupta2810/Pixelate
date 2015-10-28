#include "stdafx.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<iostream>
#include<C:\Pixelate_project\BeliefState\BeliefState.h>

#define hue_bot 71
#define hue_box 26
#define hue_ball 141
#define sat_bot 89
#define sat_box 89
#define sat_ball 89
#define lum_bot 197
#define lum_box 197
#define lum_ball 91
#define thresh_hue_bot 34 
#define thresh_hue_box 34
#define thresh_hue_ball 34
#define thresh_lum_bot 62
#define thresh_lum_box 62
#define thresh_lum_ball 62
#define thresh_sat_bot 102
#define thresh_sat_box 102
#define thresh_sat_ball 102
#define hue_arrow
#define sat_arrow
#define lum_arrow
#define thresh_hue_arrow
#define thresh_sat_arrow
#define thresh_lum_arrow




using namespace std;
using namespace BS;
using namespace cv;
namespace BS
{
	Point contour_finding(Mat,int,int,int,int,int,int,const string c);
	void erosion(Mat);

	BeliefState::BeliefState()
	{
		botPosX=0;
		botPosY=0;
		ballPosX=0;
		ballPosY=0;
		boxPosX=0;
		boxPosY=0;
		
	}
	void BeliefState::update(Mat frame)
	{
		Mat frame1=frame.clone();
		Mat frame2=frame.clone();
		Mat frame3=frame.clone();
		Mat frame4=frame.clone();

		calc_botPos( frame1,"BOT");
		calc_ballPos(frame2,"BALL");
		calc_boxPos(frame3,"BOX");
		//calc_arrowPos(frame4,"Arr"); 
		
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

		Point center=contour_finding(frame,hue_bot,sat_bot,lum_bot,thresh_hue_bot,thresh_sat_bot,thresh_lum_bot,c);
		botPosX=center.x;
		botPosY=center.y;
		 

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

 Point contour_finding(Mat src,int h,int s,int l,int t_h,int t_s,int t_l,const string c)
{
	 Mat gray;
    //cvtColor(src, gray, CV_BGR2GRAY);
	 gray=color_detection(src,h,s,l,t_h,t_s,t_l);
	 erosion(gray);
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
	
	//************************* finding center of contour *******************************
	vector<Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ ){
		mu[i] = moments( contours[i], false );
	}

	//Mass center
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
	 static int n=2;
	 namedWindow("eroding",WINDOW_AUTOSIZE);
	 createTrackbar("erode_size","eroding",&n,7);
	 Mat element=getStructuringElement(MORPH_RECT,Size(2*n+1,2*n+1),Point(n,n));
	 erode(img,img,element);
	 imshow("eroding",img);	
 }

}