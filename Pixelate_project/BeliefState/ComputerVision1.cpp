// ComputerVision1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>


#include <stdio.h>
using namespace cv;
using namespace std;
/*
void color_detection(Mat img)
{
	cvtColor(img,img,CV_BGR2HLS);
	vector<Mat> channels;
	split(img,channels);
	int thresh=10;
	namedWindow("hsl color detection",WINDOW_AUTOSIZE);
	createTrackbar("threshold","hsl color detection",&thresh,120);
	while(1)
	{
		Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
	for(int i=0;i<img.rows;i++)
	{
		
	
		for(int j=0;j<img.cols;j++)
		{
			if(channels[0].at<uchar>(i,j)<60+thresh && channels[0].at<uchar>(i,j)>60-thresh) img1.at<uchar>(i,j)=255;
		}

	}
	imshow("hsl color detection",img1);
	int key=waitKey(30);
	if(key==27) break;
  }
}
Mat edge_detection(Mat img)
{
	int thresh=10;
	//Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
	namedWindow("edge_detection",WINDOW_AUTOSIZE);
	Mat img2(img.rows,img.cols,CV_8UC1,Scalar(0));
	createTrackbar("thresh","edge_detection",&thresh,100);
	while (true)
	{

	Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
	for(int i=1;i<img.rows-1;i++)
	{
		 for(int i=1;i<img.rows-1;i++)
        {
            for(int j=1;j<img.cols-1;j++)
            {
                for(int k=-1;k<2;k++)
                {
                    for(int l=-1;l<2;l++)
                    {
                        if(abs(img.at<uchar>(i,j)-img.at<uchar>(i+k,j+l)>thresh))
                                img1.at<uchar>(i,j)=255;
                    }
                }
            }
        }
	}
	img2=img1;
	imshow("edge_detection",img1);
	int key = waitKey(10);
	if(key==27) break;

	}
	return img2;
}
void brightness(Mat img)
{
	int thresh=0;
	Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
	namedWindow("brightness",WINDOW_AUTOSIZE);
	createTrackbar("thresh","brightness",&thresh,100);
	while(1)
	{
		for(int i=0;i<img.rows;i++)
		{
			for(int j=0;j<img.cols;j++)
			{
				if(img.at<uchar>(i,j)+thresh<=255) 
				img1.at<uchar>(i,j)=img.at<uchar>(i,j)+thresh;
			}
		}
		int key = waitKey(10);
		if(key==27)break;
		imshow ("brightness",img1);
    }
}
void histogram(Mat img)
{
	//int x=(img.rows*img.cols);
	Mat img1(700,255,CV_8UC1,Scalar(255));
	int val;
	for(int i=1;i<img.rows-1;i++)
	{
		for(int j=1;j<img.cols-1;j++)
		{
			val=img.at<uchar>(i,j);
			int h=698;
			while(h>0)
			{	
				if(img1.at<uchar>(h,val)==255)
				{
					img1.at<uchar>(h,val)=0;
					break;
				}
				h--;
			}
		}
	 }
	imshow("histogram",img1);
}
void hough(Mat img)
{
	//Mat img2(img.rows,img.cols,CV_8UC1,Scalar(0));
	//img2=edge_detection(img);
	//imshow("edge detection img",img2);
	Mat img1(500,7,CV_8UC1,Scalar(0));
	float theta,r;	
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			if(img.at<uchar>(i,j)==255)
			{
				r=sqrt(i*i+j*j);
				theta=abs(atan(j/i));
				if(r>500)r=499;
				img1.at<uchar>((int)r,(int)theta)+=5;
			}
		}
	}
	imshow("hough graph",img1);
}
int z=1;
void my_blob_detect(int i,int j,Mat img,Mat img1)
{
	deque<Point> q; 
	try{
		for(int k=-1;k<2;k++)
		{
			for(int l=-1;l<2;l++)
			{
			
				if((img.at<uchar>(i+k,j+l)==255) && (img1.at<uchar>(i+k,j+l)==0)) 
					{
						Point p;
						p.x=i+k;
						p.y=j+l;
						q.push_back(p);
						img1.at<uchar>(i+k,j+l)=z;	
					}
			
			}
		}
	}
	catch(...)
			{
				cout<<"error 1"<<endl;
			}
		if(q.size()==0)
		{
			return;
		}
		else
		{
			int n=q.size()-1;
			while(n)
			{
				my_blob_detect(q[n].x,q[n].y,img,img1);
				n--;
			}
		}
}
void my_blob(Mat img)
{
	Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
	
	
		for (int i = 1; i < img.rows-1; i++)
	{
		for (int j = 1; j < img.cols-1; j++)
		{
			if(img.at<uchar>(i,j)==255 && img1.at<uchar>(i,j)==0)
			{
				img1.at<schar>(i,j)=z;
				my_blob_detect(i,j,img,img1);
				z++;
			}
		}
	}
	
	for(z;z>0;z--)
	{
		for (int i = 0; i < img.rows; i++)
		{
			for (int j = 0; j < img.cols; j++)
			{
				if(img1.at<uchar>(i,j)==z)
				{
					img.at<uchar>(i,j)=255/(1.25*z);
				}
			}
		} 
	}

	imshow("blob detection",img);
}

int main()
{
	Mat img;
	img=imread("blob.png",CV_LOAD_IMAGE_COLOR);
	//edge_detection(img);
	//Mat img1(500,500,CV_8UC3,Scalar(255,255,255));

	//for(int i=0;i<100;i++)
	//{
	//	for(int j=0;j<img1.cols;j++)
	//	{
	//		img1.at<uchar>(i,j,0)=128;
	//		img1.at<uchar>(i,j,1)=128;
	//		//img1.at<uchar>(i,j,2)=128;
	//	}
	//}
	imshow("flag",img);
	//namedWindow("flag",1);
	//color_detection(img);
	//histogram(img);
	//brightness(img);
	//edge_detection(img);
	blob(img);
	//hough(img);

	
	waitKey(0);
	getchar();
	return 0;
}
*/


///////////////////////////////////////////////////////////////////////

//************************************* color detection in hsl + contour finding + drawing rectangle + center finding ********************************* 

Mat color_detection(Mat img)
{
	cvtColor(img,img,CV_BGR2HLS);
	vector<Mat> channels;
	split(img,channels);
	int thresh_hue=10,thresh_sat=10,thresh_lum=10,hue=0,sat=0,lum=0;
	namedWindow("hsl color detection",WINDOW_AUTOSIZE);
	createTrackbar("hue","hsl color detection",&hue,240);
	createTrackbar("saturation","hsl color detection",&sat,240);
	createTrackbar("luminesence","hsl color detection",&lum,240);
	createTrackbar("threshold hue","hsl color detection",&thresh_hue,120);
	createTrackbar("threshold saturation","hsl color detection",&thresh_sat,120);
	createTrackbar("threshold lum","hsl color detection",&thresh_lum,120);
	Mat img2(img.rows,img.cols,CV_8UC1,Scalar(0));;
	
	while(1)
	{
		Mat img1(img.rows,img.cols,CV_8UC1,Scalar(0));
		
		for(int i=0;i<img.rows;i++)
		{
		
	
			for(int j=0;j<img.cols;j++)
			{
				if((channels[0].at<uchar>(i,j)<hue+thresh_hue && channels[0].at<uchar>(i,j)>hue-thresh_hue)  && (channels[1].at<uchar>(i,j)<sat+thresh_sat && channels[1].at<uchar>(i,j)>sat-thresh_sat) && (channels[2].at<uchar>(i,j)<lum+thresh_lum && channels[2].at<uchar>(i,j)>lum-thresh_lum) ) img1.at<uchar>(i,j)=255;
			}

		}
		img2=img1;
		imshow("hsl color detection",img1);
		int key=waitKey(30);
		if(key==27) break;
  }
	return img2;
}


int main( )
{
    Mat src;
    src = imread("field.png", CV_LOAD_IMAGE_COLOR);
    Mat gray;
    //cvtColor(src, gray, CV_BGR2GRAY);
	gray=color_detection(src);
    //threshold(gray, gray,200, 255,THRESH_BINARY_INV); //Threshold the gray
    imshow("gray",gray);
	
	////////////////////////////////////
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

	  for (int i = 0; i < contours.size(); i++)
	  {
		  if(i==largest_contour_index) cout<<" \n Center of the contour is : "<<(int)mc[i].x<<","<<(int)mc[i].y<<endl;;
	  }
	///////////////////////////////////////////////////

    Scalar color( 255,255,255);  // color of the contour in the
    //Draw the contour and rectangle
	Mat output(src.rows,src.cols,CV_8UC3,Scalar(0,0,0));;
	//drawContours( output, contours,largest_contour_index, color, CV_FILLED,8,hierarchy);
    rectangle(output, bounding_rect,  Scalar(0,255,0),2, 8,0);
    
	
	namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    imshow( "Display window", output );    
    waitKey(0);                                         
    return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////