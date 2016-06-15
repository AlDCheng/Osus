// Kinect_OpenCV_FingerDetectionBeta.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <vector>
#include "OpenNI.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace openni;
using namespace cv;

#define CVX_RED		CV_RGB(0xff,0x00,0x00)
#define CVX_GREEN	CV_RGB(0x00,0xff,0x00)
#define CVX_BLUE	CV_RGB(0x00,0x00,0xff)

void Calibration(cv::Mat binaryImg, cv::Mat colorImg, int* centerxOut, int* centeryOut);

int _tmain(int argc, _TCHAR* argv[])
{
	OpenNI::initialize();

	Device devAnyDevice;
    devAnyDevice.open(ANY_DEVICE);
	devAnyDevice.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	//----------------[Define Video Settings]-------------------
	//Set Properties of Depth Stream
	VideoMode mModeDepth;
	mModeDepth.setResolution( 640, 480 );
	mModeDepth.setFps( 30 );
	mModeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_100_UM );

	//Set Properties of Color Stream
	VideoMode mModeColor;
    mModeColor.setResolution( 640, 480 );
    mModeColor.setFps( 30 );
    mModeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );
	//----------------------------------------------------------

	if( devAnyDevice.isImageRegistrationModeSupported(
        IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        devAnyDevice.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }

	//----------------------[Initial Streams]---------------------
	VideoStream streamInitDepth;
    streamInitDepth.create( devAnyDevice, SENSOR_DEPTH );

	VideoStream streamInitColor;
    streamInitColor.create( devAnyDevice, SENSOR_COLOR );

	streamInitDepth.setVideoMode( mModeDepth );
	streamInitColor.setVideoMode( mModeColor );

	namedWindow( "Depth Image (Init)",  CV_WINDOW_AUTOSIZE );
	namedWindow( "Color Image (Green)",  CV_WINDOW_AUTOSIZE );
    namedWindow( "Color Image (Init)",  CV_WINDOW_AUTOSIZE );
	namedWindow( "Color Image (Green-Binary)", CV_WINDOW_AUTOSIZE );
	namedWindow( "Depth Image (Init-Binary)", CV_WINDOW_AUTOSIZE );

	VideoFrameRef  frameDepthInit;
    VideoFrameRef  frameColorInit;

	streamInitDepth.start();
	streamInitColor.start();
	cv::Mat BackgroundFrame;

	int avgDist = 0;
	int iMaxDepthInit = streamInitDepth.getMaxPixelValue();
	int centerxA, centeryA, centerxB, centeryB;

	vector<Mat> frameRGB;

	//vector<int> backgroundDepth;
	//int backgroundDepth[480][640];

	//------------------------------------------------------------
	//--------------------[Initiation Process]--------------------
	while( true )
	{
		streamInitDepth.readFrame( &frameDepthInit );
		streamInitColor.readFrame( &frameColorInit );

		const cv::Mat mImageColor( frameDepthInit.getHeight(), frameDepthInit.getWidth(), CV_8UC3, (void*)frameColorInit.getData());
		const cv::Mat mImageDepth( frameDepthInit.getHeight(), frameDepthInit.getWidth(), CV_16UC1, (void*)frameDepthInit.getData());

		split( mImageColor, frameRGB );

		cv::imshow( "Color Image (Green)", frameRGB[1]);

		cv::Mat frameColorThres( frameDepthInit.getHeight(), frameDepthInit.getWidth(), CV_8UC1 );
		threshold( frameRGB[1], frameColorThres, 100, 255, THRESH_BINARY );
		cv::imshow( "Color Image (Green-Binary)", frameColorThres );

        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepthInit );

        cv::imshow( "Depth Image (Init)", mScaledDepth );

        const cv::Mat mImageRGB(frameColorInit.getHeight(), frameColorInit.getWidth(), CV_8UC3, (void*)frameColorInit.getData());

        cv::Mat cImageBGR;
        cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );

		//--------------------[Get Average Distance]---------------------
		int depthVal = 0;
		int frameHeight = frameDepthInit.getHeight();
		int frameWidth = frameDepthInit.getWidth();
		//------------
		//backgroundDepth.resize(frameHeight * frameWidth);
		//---------------------------------------------------------------
		
		cv::Mat mImageThres( frameDepthInit.getHeight(), frameDepthInit.getWidth(), CV_8UC1 );
		for(int i = 0; i < frameDepthInit.getHeight(); i++)
		{
			for(int j = 0; j < frameDepthInit.getWidth(); j++)
			{
				int depthVal = mImageDepth.at<unsigned short>(i, j);

				if((depthVal <= 590) && (depthVal > 100))
				{
					mImageThres.data[mImageThres.step[0]*i + mImageThres.step[1]*j] = 0;
				}
				else
				{
					mImageThres.data[mImageThres.step[0]*i + mImageThres.step[1]*j] = 255;
				}
			}
		}
		//---------------------------------------------------------------

		cv::imshow( "Depth Image (Init-Binary)", mImageThres );

		int initCount = 0;
		for(int i = 0; i < frameHeight; i++)
		{
			for(int j = 0; j < frameWidth; j++)
			{
				depthVal = mImageDepth.at<unsigned short>(i, j) + depthVal;
				//backgroundDepth[initCount] = mImageDepth.at<unsigned short>(i, j);
				//backgroundDepth[i][j] = mImageDepth.at<unsigned short>(i, j);
				initCount++;
			}
		}
		//avgDist = depthVal / ( frameDepthInit.getHeight() * frameDepthInit.getWidth() );
		avgDist = depthVal / ((frameHeight) * (frameWidth));

		cout << "Average Distance: " << avgDist << endl;

		//---------------------------------------------------------------
		centerxA = 0; centeryA = 0; centerxB = 0; centeryB = 0;
		Calibration( frameColorThres, cImageBGR, &centerxA, &centeryA );
		Calibration( mImageThres, cImageBGR, &centerxB, &centeryB );

		cv::imshow( "Color Image (Init)", cImageBGR );

		cout << "CenterA: (" << centerxA << "," << centeryA << ")    " <<
			"CenterB: (" << centerxB << "," << centeryB << ")" << endl;

		if( cv::waitKey(1) == 'q')
		{
			mImageDepth.copyTo(BackgroundFrame);
            break;
		}
	}

	streamInitDepth.destroy();
	streamInitColor.destroy();
	//------------------------------------------------------------
	//------------------------------------------------------------
    devAnyDevice.close();

    openni::OpenNI::shutdown();

	return 0;
}

//----------------------------------------------------------------
//----------------------------------------------------------------

void Calibration(cv::Mat binaryImg, cv::Mat colorImg, int* centerxOut, int* centeryOut)
{
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

		if((area > 500) && (area < 5000))
		{
			///////////////////////////////////////////////////////
			//Draw stuff
			Point pt, ptx1, ptx2, pty1, pty2;
			*centerxOut = centerx;
			*centeryOut = centery;
			pt.x = centerx;
			pt.y = centery;
			ptx1.x = centerx - 10; ptx1.y = centery;
			ptx2.x = centerx + 10; ptx2.y = centery;
			pty1.x = centerx; pty1.y = centery - 10;
			pty2.x = centerx; pty2.y = centery + 10;

			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( colorImg, contours, i, color, 2, 8, hierarchy, 0, Point() );

			line( colorImg, ptx1, ptx2, color, 2 );
			line( colorImg, pty1, pty2, color, 2 );
			///////////////////////////////////////////////////////
		}
	}
}