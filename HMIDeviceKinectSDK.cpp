#include "stdafx.h"
#include <Windows.h>
#include <Ole2.h>
#include <ShlObj.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\contrib\contrib.hpp>

using namespace std;
using namespace cv;

#define COLOR_WIDTH 640
#define COLOR_HEIGHT 480
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240    
#define CHANNEL 3

#define CVX_RED		CV_RGB(0xff,0x00,0x00)
#define CVX_GREEN	CV_RGB(0x00,0xff,0x00)
#define CVX_BLUE	CV_RGB(0x00,0x00,0xff)

vector<int> OutX, OutY;
int UpX, UpY, DownX, DownY, CurX, CurY;
CvSVM SVMFinger;

BYTE buf[DEPTH_WIDTH * DEPTH_HEIGHT * CHANNEL];

long depthToRgbMap[DEPTH_WIDTH * DEPTH_HEIGHT * 2];

void MouseMove(int x, int y);
void LeftClick();
void LeftHold();
void LeftRelease();

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	CurX = x;
	CurY = y;
    if  ( event == EVENT_LBUTTONDOWN )
    {
        //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		UpX = x;
		UpY = y;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		DownX = x;
		DownY = y;
    }
}


int drawColor(HANDLE h, IplImage* color)    
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame);
    if (FAILED(hr)) 
    {
        cout << "Get Image Frame Failed" << endl;
        return -1;
    }
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch != 0)
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;
        cvSetData(color, pBuffer, LockedRect.Pitch);
    }
    NuiImageStreamReleaseFrame(h, pImageFrame);
    return 0;
}

int storeBackground(HANDLE h, cv::Mat backgroundMap)
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame);
    if (FAILED(hr))
    {
        cout << "Get Image Frame Failed" << endl;
        return -1;
    }
    
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    if (LockedRect.Pitch != 0)
    {
        USHORT * pBuff = (USHORT*) LockedRect.pBits;

        for (int i = 0; i < DEPTH_WIDTH * DEPTH_HEIGHT; i++)
        {
			int x = i/DEPTH_WIDTH;
			int y = i%DEPTH_WIDTH;

            BYTE index = pBuff[i] & 0x07;
            USHORT realDepth = (pBuff[i] & 0xFFF8) >> 3;

			double depth = (double)realDepth;

			backgroundMap.at<double>(x, y) = depth+5;
		}
	}
    NuiImageStreamReleaseFrame(h, pImageFrame);
    return 0;
}

int drawBinary(HANDLE h, cv::Mat depth, cv::Mat backgroundMap)
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame);
    if (FAILED(hr))
    {
        cout << "Get Image Frame Failed" << endl;
        return -1;
    }
    
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    if (LockedRect.Pitch != 0)
    {
        //USHORT * pBuff = (USHORT*) LockedRect.pBits;

		int whitePxlCount=0;
		Mat backgroundCur(DEPTH_HEIGHT, DEPTH_WIDTH, CV_64F);

		LONG colorX;
		LONG colorY;

		//NUI_IMAGE_VIEW_AREA viewArea = { NUI_IMAGE_DIGITAL_ZOOM_1X, UpY, UpX };

		const USHORT* curr = (const USHORT*) LockedRect.pBits;
        for (int j = 0; j < DEPTH_HEIGHT; ++j) {
            for (int i = 0; i < DEPTH_WIDTH; ++i) {
                // Get depth of pixel in millimeters
                USHORT depthVal = NuiDepthPixelToDepth(*curr);
                // Store the index into the color array corresponding to this pixel
                NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
                    NUI_IMAGE_RESOLUTION_320x240, // color frame resolution
                    NUI_IMAGE_RESOLUTION_320x240, // depth frame resolution
                    NULL,                         // pan/zoom of color image
                    i, j, *curr,                  // Column, row, and depth in depth image
                    &colorX, &colorY        // Output: column and row (x,y) in the color image
                );
                int backDepth = (int)backgroundMap.at<double>(j, i);
                *curr++;
				int realDepth = (int)depthVal;
				if(((UpX) <= (int)colorX*2) && (((int)colorX*2) < (DownX)) && ((UpY) <= int(colorY*2)) && (int(colorY*2) < (DownY)))
				{
					if((realDepth > (backDepth - 30)) && (realDepth <= (backDepth - 15)))
					{
						depth.at<uchar>((int)colorY, (int)colorX) = 255;
						whitePxlCount++;
					}
					else
					{
						depth.at<uchar>((int)colorY, (int)colorX) = 0;
					}
				}
            }
		}
		/*if(((double)whitePxlCount > 0.25*(double)(DEPTH_HEIGHT*DEPTH_WIDTH))||((double)whitePxlCount < 0.0001*(double)(DEPTH_HEIGHT*DEPTH_WIDTH)))
		{
			backgroundCur.copyTo(backgroundMap);
		}*/
	}
    NuiImageStreamReleaseFrame(h, pImageFrame);
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



	for( int i = 0; i< contours.size(); i++ )
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

void fingerDetection(cv::Mat binaryImg, cv::Mat colorImg, vector<int> OldOutX, vector<int> OldOutY)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat counterImg = binaryImg.clone();
	findContours( counterImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(20, 35) );

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

	for( int i = 0; i< contours.size(); i++ )
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

		//if((area > 15) && (area < 200) && (circularity > 0.1) && (circularity < 0.9))
		Mat sampleMat = (Mat_<float>(1,4) << extent, circularity, area, perimeter);
		float response = SVMFinger.predict(sampleMat);
		if((response == 1) && (area > 10) && (area < 200))
		{
			float furthestReach;
			///////////////////////////////////////////////////////
			//Draw stuff
			Point pt, ptx1, ptx2, pty1, pty2;
			centerx = (double)centerx/**0.97+10*/;
			centery = (double)centery/**.95-35+10*/;
			pt.x = centerx;
			pt.y = centery;
			cout << centerx << ", " << centery << endl;
			ptx1.x = centerx - 10; ptx1.y = centery;
			ptx2.x = centerx + 10; ptx2.y = centery;
			pty1.x = centerx; pty1.y = centery - 10;
			pty2.x = centerx; pty2.y = centery + 10;
			ptx1.x += 25;
			ptx1.y += 20;
			ptx2.x += 25;
			ptx2.y += 20;

			pty1.x += 25;
			pty1.y += 20;
			pty2.x += 25;
			pty2.y += 20;
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			Scalar white = (255, 255, 255);
			Scalar black = (0, 0, 0);

			if( OldOutX.size() > 0)
			{
				for(int k = 0; k < OldOutX.size(); k++)
				{
					int distance = sqrt(pow((double)(centerx-OldOutX[k]),2) + pow((double)(centery-OldOutY[k]),2) );

					if(distance < 20)
					{
						//drawContours( colorImg, contours, i, color, 2, 8, hierarchy, 0, Point() );
						line( colorImg, ptx1, ptx2, color, 2 );
						line( colorImg, pty1, pty2, color, 2 );
						///////////////////////////////////////////////////////
						OutX.push_back(centerx);
						OutY.push_back(centery);
					}
				}
			}
			else
			{
				OutX.push_back(centerx);
				OutY.push_back(centery);
			}
		}
	}
}


int main(int argc, char * argv[])
{
	//---[SVM]---
	FILE *fptrI = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv","w");
	fprintf(fptrI, "Classtype, Area, Perimeter, Circularity, Extent\n");
	fclose(fptrI);

	Mat input = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2015\\SVM\\Shape Features\\Fingers2015.bmp", 1);
	Mat input2 = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2015\\SVM\\Shape Features\\NotFingers2015.bmp", 1);
	Mat inputF = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2015\\SVM\\Shape Features\\ImageFeaturesBinaryF.bmp", 1);
	Mat gray(input.rows, input.cols, CV_8UC3);
	Mat gray2(input.rows, input.cols, CV_8UC3);
	Mat grayF(input.rows, input.cols, CV_8UC3);
	cvtColor(input, gray, CV_BGR2GRAY);
	cvtColor(input2, gray2, CV_BGR2GRAY);
	cvtColor(inputF, grayF, CV_BGR2GRAY);
	shapeFeatures(gray, input, 1);
	shapeFeatures(gray2, input2, 2);
	namedWindow("Image");
	imshow("Image", input);
	namedWindow("Image2");
	imshow("Image2", input2);

	//------------------------------------------------------
	//--------[SVM]--------
	// Read input data from file created above
	double parameters[5];
	vector<double> svmI, svmA, svmP, svmC, svmE;
	int size = 1;
	double index = 0; double area = 0; double perimeter = 0; double circularity = 0;
	char buffer[1024];
	char *record, *lineData;
	FILE* fptrR = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv", "r");
	fscanf(fptrR, "%*[^\n]\n", NULL);

	svmI.resize(size); svmA.resize(size); svmP.resize(size); svmC.resize(size); 

	while((lineData=fgets(buffer, sizeof(buffer), fptrR))!=NULL)
	{
		size++;
		svmI.resize(size);
		svmA.resize(size);
		svmP.resize(size);
		svmC.resize(size);
		svmE.resize(size);

		record = strtok(lineData, ";");
		for(int i = 0; i < 5; i++);
		{
			double value = atoi(record);
			record = strtok(lineData,";");
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
	float labels[1000];
	for(int i = 0; i < svmI.size()-1; i++)
	{
		labels[i] = svmI[i+1];
	}
    Mat labelsMat(1000, 1, CV_32FC1, labels);

    float trainingData[1000][4];
	for(int i = 0; i < svmE.size()-1; i++)
	{
		trainingData[i][0] = svmE[i+1];
		trainingData[i][1] = svmC[i+1];
		trainingData[i][2] = svmA[i+1];
		trainingData[i][3] = svmP[i+1];
	}
    Mat trainingDataMat(1000, 4, CV_32FC1, trainingData);

    // Set up SVM's parameters
    CvSVMParams params;
	params = SVMFinger.get_params();

    // Train the SVM
    SVMFinger.train_auto(trainingDataMat, labelsMat, Mat(), Mat(), params);

//	Mat sampleMat = (Mat_<float>(1,2) << 138.5, 57);
//	float response = SVMFinger.predict(sampleMat);

	waitKey();
	destroyWindow("Image");
	destroyWindow("Image2");

	//------------------------------------------
	//-----------
    IplImage* color = cvCreateImageHeader(cvSize(COLOR_WIDTH, COLOR_HEIGHT), IPL_DEPTH_8U, 4);

    HRESULT hr = NuiInitialize(
            NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
            | NUI_INITIALIZE_FLAG_USES_COLOR
            | NUI_INITIALIZE_FLAG_USES_SKELETON);

    if (hr != S_OK)
    {
        cout << "NuiInitialize failed" << endl;
        return hr;
    }

    HANDLE h1 = CreateEvent(NULL, TRUE, FALSE, NULL);
    HANDLE h2 = NULL;
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
            0, 2, h1, &h2);
    if (FAILED(hr))
    {
        cout << "Could not open image stream video" << endl;
        return hr;
    }

    HANDLE h3 = CreateEvent(NULL, TRUE, FALSE, NULL);
    HANDLE h4 = NULL;
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
            NUI_IMAGE_RESOLUTION_320x240, 0, 2, h3, &h4);
    if (FAILED(hr))
    {
        cout << "Could not open depth stream video" << endl;
        return hr;
    }
	Mat mImageThres(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC1 );
	Mat binary(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1 );
	Mat backgroundMap(DEPTH_HEIGHT, DEPTH_WIDTH, CV_64F);

	namedWindow("Color Image", CV_WINDOW_AUTOSIZE);
	
	//------------------------------------------------------------
	//--------------------[Initiation Process]--------------------
	while(1)
	{
		WaitForSingleObject(h1, INFINITE);
        drawColor(h2, color);
		WaitForSingleObject(h3, INFINITE);
        storeBackground(h4, backgroundMap);
		Mat colorImage(color);

		char CurrentXY[8], UpXY[8], DownXY[8];
		sprintf_s(CurrentXY, "%d,%d", CurX, CurY);
		sprintf_s(UpXY, "%d,%d", UpX, UpY);
		sprintf_s(DownXY, "%d,%d", DownX, DownY);

		setMouseCallback("Color Image", CallBackFunc, NULL);
		putText(colorImage, CurrentXY, cvPoint(COLOR_WIDTH-100, COLOR_HEIGHT-30), 
			FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 255, 0));
		putText(colorImage, UpXY, cvPoint(UpX, UpY-20), 
			FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 255, 0));
		putText(colorImage, DownXY, cvPoint(DownX, DownY-20), 
			FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 255, 0));
		rectangle(colorImage, cvPoint(UpX, UpY), cvPoint(DownX, DownY), cvScalar(255,255,0), 2);
		//---------------------------------------------------------------

		cv::imshow( "Color Image", colorImage );

		int c = cvWaitKey(1);
        if (c == 27 || c == 'q' || c == 'Q')
            break;
	}
	destroyWindow("Color Image");

	//namedWindow("color image", CV_WINDOW_AUTOSIZE);
	namedWindow("binary image", CV_WINDOW_AUTOSIZE);


	OutX.clear();
	OutY.clear();
	vector<int> OldOutX, OldOutY;
	OldOutX.clear();
	OldOutY.clear();

	//------------------------------------------------------------
	// Kalman Filter:
    KalmanFilter KF(4, 2, 0);
    Mat_<float> state(4, 1); /* (x, y, Vx, Vy) */
    Mat processNoise(4, 1, CV_32F);
    Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
    char code = (char)-1;
	int averageX = 0;
	int averageY = 0;
	int resetCounter = 0;

	KF.statePre.at<float>(0) = averageX;
	KF.statePre.at<float>(1) = averageY;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
		
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(/*1e-1*/1e-3));
    setIdentity(KF.errorCovPost, Scalar::all(.1));

	vector<Point> mousev,kalmanv;

	mousev.clear();
	kalmanv.clear();

	Mat canvas(720, 1280, CV_8UC3);

	bool mouseClick = false;
	
	//1366 x 768
	double windowLength = DownX - UpX;
	double windowHeight = DownY - UpY;

	double shiftX, shiftY;
	shiftX = 1280/windowLength;
	shiftY = 720/windowHeight;	

	int mX = -18;
	int mY = 23;

    while (1)
    {
        WaitForSingleObject(h1, INFINITE);
        drawColor(h2, color);
        WaitForSingleObject(h3, INFINITE);
        drawBinary(h4, binary, backgroundMap);
		resize(binary, mImageThres, cvSize(COLOR_WIDTH, COLOR_HEIGHT));

		Mat colorImage(color);

		fingerDetection( mImageThres, colorImage, OldOutX, OldOutY);

		//----------------------------------
		//Simulate mouse/touch here:

		if((OutX.size() > 0) && (resetCounter < 1))
		{
			resetCounter = 0;
			averageX = 0;
			averageY = 0;
			for( int i = 0; i < OutX.size(); i++ )
			{
				averageX += OutX[i];
				averageY += OutY[i];
			}
			
			averageX = ((averageX / OutX.size()) - UpX);
			averageY = ((averageY / OutY.size()) - UpY);

			averageY = windowHeight - averageY;

			averageX += mX;
			if(averageX < 0)
				averageX = 0;
			averageY += mY;
			if(averageY > windowHeight)
				averageY = windowHeight;
		}
		else if((resetCounter > 1) && (OutX.size() > 0))
		{
			resetCounter = 0;
			averageX = 0;
			averageY = 0;
			for( int i = 0; i < OutX.size(); i++ )
			{
				averageX += OutX[i];
				averageY += OutY[i];
			}
			averageX = ((averageX / OutX.size()) - UpX);
			averageY = ((averageY / OutY.size()) - UpY);

			//averageX = windowLength - averageX;
			averageY = windowHeight - averageY;

			averageX += mX;
			if(averageX < 0)
				averageX = 0;
			averageY += mY;
			if(averageY > windowHeight)
				averageY = windowHeight;

			KF.statePre.at<float>(0) = averageX;
			KF.statePre.at<float>(1) = averageY;
			KF.statePre.at<float>(2) = 0;
			KF.statePre.at<float>(3) = 0;
			setIdentity(KF.measurementMatrix);
			setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
			setIdentity(KF.measurementNoiseCov, Scalar::all(/*1e-1*/1e-3));
			setIdentity(KF.errorCovPost, Scalar::all(.1));
			mousev.clear();
			kalmanv.clear();
		}
		else if(resetCounter > 1)
		{
			LeftRelease( );
			mouseClick = false;
		}
		else
		{
			resetCounter++;
		}

		//------------------------------------------------
		Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

        measurement(0) = averageX;
		measurement(1) = averageY;
			
		Point measPt(measurement(0),measurement(1));
		mousev.push_back(measPt);
        // generate measurement
        //measurement += KF.measurementMatrix*state;

		Mat estimated = KF.correct(measurement);

		if(estimated.at<float>(0) > windowLength)
		{
			estimated.at<float>(0) = windowLength;
		}
		if(estimated.at<float>(1) > windowHeight)
		{
			estimated.at<float>(1) = windowHeight;
		}

		Point statePt(estimated.at<float>(0),estimated.at<float>(1));
		kalmanv.push_back(statePt);

		
		for (int i = 0; i < kalmanv.size()-1; i++) {
			line(canvas, kalmanv[i]*shiftX*0.5, kalmanv[i+1]*shiftY*0.5, Scalar(0,255,0), 1);
		}

		//------------------------------------------------


		int ptX = statePt.x*shiftX;
		int ptY = statePt.y*shiftY;

		int MptX = measPt.x*shiftX;
		int MptY = measPt.y*shiftY;

		canvas = Scalar::all(0);
		
		line( canvas, Point( ptX/2 - 5, ptY/2 - 5 ), Point( ptX/2 + 5, ptY/2 + 5 ), 
			Scalar(255,255,255), 2, CV_AA, 0);
		line( canvas, Point( ptX/2 + 5,  ptY/2 - 5 ), Point( ptX/2 - 5, ptY/2 + 5 ),
			Scalar(255,255,255), 2, CV_AA, 0 );

		line( canvas, Point( MptX/2 - 5, MptY/2 - 5 ), Point( MptX/2 + 5, MptY/2 + 5 ), 
			Scalar(0,0,255), 2, CV_AA, 0);
		line( canvas, Point( MptX/2 + 5,  MptY/2 - 5 ), Point( MptX/2 - 5, MptY/2 + 5 ),
			Scalar(0,0,255), 2, CV_AA, 0 );

		for (int i = 0; i < mousev.size()-1; i++)
		{
			line(canvas, Point(mousev[i].x*shiftX/2, mousev[i].y*shiftY/2),
				Point(mousev[i+1].x*shiftX/2, mousev[i+1].y*shiftY/2), Scalar(255,255,0), 1);
		}
		for (int i = 0; i < kalmanv.size()-1; i++)
		{
			line(canvas, Point(kalmanv[i].x*shiftX/2, kalmanv[i].y*shiftY/2),
				Point(kalmanv[i+1].x*shiftX/2, kalmanv[i+1].y*shiftY/2), Scalar(0,255,0), 1);
		}

		resize(canvas, canvas, Size(400,300));
		imshow( "Mouse Canvas", canvas );

		if(OutX.size() > 0)
		{

			//circle(cImageBGR, Point(averageX, averageY), 10, Scalar(0, 255, 255), 3);
			MouseMove( (statePt.x*shiftX), (statePt.y*shiftY) );
			//LeftClick();
			if(mouseClick == false)
			{
				LeftHold( );
				mouseClick = true;
			}
		}
		//resize(cImageBGR, cImageBGR, Size(320,240));

		imshow("binary image", mImageThres);
		//imshow("color image", colorImage);

		OldOutX.clear();
		OldOutY.clear();
		OldOutX = OutX;
		OldOutY = OutY;
		OutX.clear();
		OutY.clear();

        //exit
        int c = cvWaitKey(1);
        if (c == 27 || c == 'q' || c == 'Q')
            break;
    }

    cvReleaseImageHeader(&color);
    destroyWindow("color image");
	destroyWindow("binary image");
    NuiShutdown();

    return 0;

}

//-------------[Mouse emulation]---------------
void MouseMove ( int x, int y )
{  
	double fScreenWidth    = ::GetSystemMetrics( SM_CXSCREEN )-1; 
	double fScreenHeight  = ::GetSystemMetrics( SM_CYSCREEN )-1; 
	double fx = x*(65535.0f/fScreenWidth);
	double fy = y*(65535.0f/fScreenHeight);
	INPUT  Input={0};
	Input.type      = INPUT_MOUSE;
	Input.mi.dwFlags  = MOUSEEVENTF_MOVE|MOUSEEVENTF_ABSOLUTE;
	Input.mi.dx = fx;
	Input.mi.dy = fy;
	::SendInput(1,&Input,sizeof(INPUT));
}

void LeftClick ( )
{  
	INPUT    Input={0};
	// left down 
	Input.type      = INPUT_MOUSE;
	Input.mi.dwFlags  = MOUSEEVENTF_LEFTDOWN;
	::SendInput(1,&Input,sizeof(INPUT));

	// left up
	::ZeroMemory(&Input,sizeof(INPUT));
	Input.type      = INPUT_MOUSE;
	Input.mi.dwFlags  = MOUSEEVENTF_LEFTUP;
	::SendInput(1,&Input,sizeof(INPUT));
}

void LeftHold( )
{
	INPUT    Input={0};
	// left down 
	Input.type      = INPUT_MOUSE;
	Input.mi.dwFlags  = MOUSEEVENTF_LEFTDOWN;
	::SendInput(1,&Input,sizeof(INPUT));
}

void LeftRelease( )
{
	// left up
	INPUT    Input={0};
	Input.type      = INPUT_MOUSE;
	Input.mi.dwFlags  = MOUSEEVENTF_LEFTUP;
	::SendInput(1,&Input,sizeof(INPUT));
}