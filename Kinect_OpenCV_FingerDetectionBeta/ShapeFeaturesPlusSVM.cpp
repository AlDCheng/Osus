#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string.h>
#include "C:/Program Files/OpenNI2/Include/OpenNI.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Windows.h>

using namespace std;
using namespace openni;
using namespace cv;

void shapeFeatures(cv::Mat binaryImg, cv::Mat colorImg, int classtype);
void SVMClassification(cv::Mat binaryImg, cv::Mat colorImg, CvSVM SVM);

int main()
{
	FILE *fptrI = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv","w");
	fprintf(fptrI, "Classtype, Area, Perimeter, Circularity\n");
	fclose(fptrI);

	Mat input = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2014\\SVM\\Shape Features\\ImageFeaturesBinary.bmp", 1);
	Mat input2 = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2014\\SVM\\Shape Features\\ImageFeaturesBinary2.bmp", 1);
	Mat inputF = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2014\\SVM\\Shape Features\\ImageFeaturesBinaryF.bmp", 1);
	Mat gray(input.rows, input.cols, CV_8UC3);
	Mat gray2(input.rows, input.cols, CV_8UC3);
	Mat grayF(input.rows, input.cols, CV_8UC3);
	cvtColor(input, gray, CV_BGR2GRAY);
	cvtColor(input2, gray2, CV_BGR2GRAY);
	cvtColor(inputF, grayF, CV_BGR2GRAY);
	shapeFeatures(gray, input, 0);
	shapeFeatures(gray2, input2, 1);
	namedWindow("Image");
	imshow("Image", input);
	namedWindow("Image2");
	imshow("Image2", input2);
	waitKey();

	//------------------------------------------------------
	//--------[SVM]--------
	// Read input data from file created above
	double parameters[5];
	vector<double> svmI, svmA, svmP, svmC, svmE;
	int size = 1;
	double index = 0; double area = 0; double perimeter = 0; double circularity = 0;
	char buffer[1024];
	char *record, *line;
	FILE* fptrR = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv", "r");
	fscanf(fptrR, "%*[^\n]\n", NULL);

	svmI.resize(size); svmA.resize(size); svmP.resize(size); svmC.resize(size); 

	while((line=fgets(buffer, sizeof(buffer), fptrR))!=NULL)
	{
		size++;
		svmI.resize(size);
		svmA.resize(size);
		svmP.resize(size);
		svmC.resize(size);
		svmE.resize(size);

		record = strtok(line, ";");
		for(int i = 0; i < 5; i++);
		{
			double value = atoi(record);
			record = strtok(line,";");
		}
		char *lineCopy = record;
		char *pch;

		pch = strtok(lineCopy, ",");
		parameters[0] = atoi(pch);
		
		int j = 1;
		while( j < 5 )
		{
			pch = strtok (NULL, ",");
			parameters[j] = atof(pch);
			j++;
		}
		svmI[size-1] = parameters[0];
		svmA[size-1] = parameters[1];
		svmP[size-1] = parameters[2];
		svmC[size-1] = parameters[3];
		svmE[size-1] = parameters[4];
	}
	fclose(fptrR);
	//---------------------
	// Data for visual representation
    int width = 512, height = 512;
    Mat image = Mat::zeros(height, width, CV_8UC3);

    // Set up training data
    //float labels[8] = {1.0, -1.0, -1.0, -1.0};
	float labels[10];
	for(int i = 0; i < 10; i++)
	{
		labels[i] = svmI[i+1];
	}
    Mat labelsMat(10, 1, CV_32FC1, labels);

    float trainingData[10][2];
	for(int i = 0; i < 10; i++)
	{
		trainingData[i][0] = svmE[i+1];
		trainingData[i][1] = svmC[i+1];
	}
    Mat trainingDataMat(10, 2, CV_32FC1, trainingData);

    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    // Train the SVM
    CvSVM SVM;
    SVM.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), params);

	Mat sampleMat = (Mat_<float>(1,2) << .7, .9);
	float response = SVM.predict(sampleMat);

	SVMClassification(grayF, inputF, SVM);

    /*Vec3b green(0,255,0), blue (255,0,0);
	//--------------Training Done-----------------
    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            Mat sampleMat = (Mat_<float>(1,2) << j,i);
            float response = SVM.predict(sampleMat);

            if (response == 1)
                image.at<Vec3b>(i,j)  = green;
            else if (response == 0)
                image.at<Vec3b>(i,j)  = blue;
        }

    // Show the training data
    /*int thickness = -1;
    int lineType = 8;
    circle( image, Point(501,  10), 5, Scalar(  0,   0,   0), thickness, lineType);
    circle( image, Point(255,  10), 5, Scalar(255, 255, 255), thickness, lineType);
    circle( image, Point(501, 255), 5, Scalar(255, 255, 255), thickness, lineType);
    circle( image, Point( 10, 501), 5, Scalar(255, 255, 255), thickness, lineType);

    // Show support vectors
    thickness = 2;
    lineType  = 8;
    int c     = SVM.get_support_vector_count();

    for (int i = 0; i < c; ++i)
    {
        const float* v = SVM.get_support_vector(i);
        circle( image,  Point( (int) v[0], (int) v[1]),   6,  Scalar(128, 128, 128), thickness, lineType);
    }

    imwrite("result.png", image);        // save the image

    imshow("SVM Simple Example", image); // show it to the user
    waitKey(0);*/
	namedWindow("ImageF");
	imshow("ImageF", inputF);
	waitKey();
	destroyWindow("Image");
	destroyWindow("Image2");
	destroyWindow("ImageF");
	exit(0);
	return 0;
}

void shapeFeatures(cv::Mat binaryImg, cv::Mat colorImg, int classtype)
{
	FILE *fptr = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv","a");

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
		CvBox2D box;
		box = minAreaRect(contours[i]);
		double extent = (area/(box.size.height*box.size.width));

		float furthestReach;
		///////////////////////////////////////////////////////
		//Draw stuff
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( colorImg, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point() );
		///////////////////////////////////////////////////////
		fprintf(fptr, "%d,%f,%d,%f,%f\n", classtype, area, perimeter, circularity, extent);
	}
	fclose(fptr);
}

void SVMClassification(cv::Mat binaryImg, cv::Mat colorImg, CvSVM SVM)
{
	FILE *fptr = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv","a");

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

		CvBox2D box;
		box = minAreaRect(contours[i]);
		double extent = (area/(box.size.height*box.size.width));

		Vec3b green(0,255,0), blue (255,0,0);
		//--------------Training Done-----------------
		// Show the decision regions given by the SVM
		Mat sampleMat = (Mat_<float>(1,2) << extent, circularity);
		float response = SVM.predict(sampleMat);

		if (response == 1)
		{
			drawContours( colorImg, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point() );
		}
		else if (response == 0)
		{
			drawContours( colorImg, contours, i, Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point() );
		}
		else
		{
			drawContours( colorImg, contours, i, Scalar(255, 0, 0), 2, 8, hierarchy, 0, Point() );
		}
	}
	fclose(fptr);
}