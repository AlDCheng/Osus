#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "C:/Program Files/OpenNI2/Include/OpenNI.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Windows.h>

using namespace std;
using namespace openni;
using namespace cv;

void shapeFeatures(cv::Mat binaryImg, cv::Mat colorImg);

int main()
{
	Mat input = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2014\\SVM\\Shape Features\\ImageFeaturesBinary.bmp", 1);
	Mat gray(input.rows, input.cols, CV_8UC3);
	cvtColor(input, gray, CV_BGR2GRAY);
	shapeFeatures(gray, input);
	namedWindow("Image");
	imshow("Image", input);
	waitKey();
}

void shapeFeatures(cv::Mat binaryImg, cv::Mat colorImg)
{
	FILE *fptr = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv","w");
	fprintf(fptr, "Area, Perimeter, Circularity\n");

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat counterImg = binaryImg.clone();
	findContours( counterImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	/// Get the moments
	vector<Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{
		mu[i] = moments( contours[i], false );
	}

	///  Get the mass centers:
	vector<Point2f> mc( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	{
		mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
	}

	//Draw contours. NOW IN COLOR
	RNG rng(12345);

	double area;

	for( int i = 1; i< contours.size(); i++ )
    {
		int centerx = (mu[i].m10/mu[i].m00);
		int centery = (mu[i].m01/mu[i].m00);
	   
		area = contourArea(contours[i], false);
		Point2f encCenter;
		int perimeter = (int)arcLength(contours[i], true);
		double circularity = (4 * 3.14 * area)/(perimeter * perimeter);

		float furthestReach;
		///////////////////////////////////////////////////////
		//Draw stuff
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( colorImg, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point() );
		///////////////////////////////////////////////////////
		fprintf(fptr, "%f, %i, %f\n", area, perimeter, circularity);
	}
	fclose(fptr);
}