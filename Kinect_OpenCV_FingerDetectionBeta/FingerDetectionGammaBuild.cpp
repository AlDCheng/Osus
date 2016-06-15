// Kinect_OpenCV_FingerDetectionBeta.cpp : Defines the entry point for the console application.
//

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

#define CVX_RED		CV_RGB(0xff,0x00,0x00)
#define CVX_GREEN	CV_RGB(0x00,0xff,0x00)
#define CVX_BLUE	CV_RGB(0x00,0x00,0xff)

void fingerDetection(cv::Mat binaryImg, cv::Mat colorImg, vector<int> OldOutX, vector<int> OldOutY);
void Calibration(cv::Mat binaryImg, cv::Mat colorImg, int* centerxOut, int* centeryOut);
void sendCoord();

void MouseMove(int x, int y);
void LeftClick();

vector<int> OutX, OutY;

int _tmain(int argc, _TCHAR* argv[])
{
	OpenNI::initialize();

	Device devAnyDevice;
    devAnyDevice.open(ANY_DEVICE);
	devAnyDevice.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	/*
	if (devAnyDevice.isValid())
		if (devAnyDevice.getImageRegistrationMode() == IMAGE_REGISTRATION_DEPTH_TO_COLOR)
			devAnyDevice.setImageRegistrationMode(IMAGE_REGISTRATION_OFF);
		else
			devAnyDevice.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);*/

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
    namedWindow( "Color Image (Init)",  CV_WINDOW_AUTOSIZE );
	//namedWindow( "Thresholded Image (Init)", CV_WINDOW_AUTOSIZE );

	VideoFrameRef  frameDepthInit;
    VideoFrameRef  frameColorInit;

	streamInitDepth.start();
	streamInitColor.start();
	cv::Mat BackgroundFrame;

	int avgDist = 0;
	int iMaxDepthInit = streamInitDepth.getMaxPixelValue();

	//vector<int> backgroundDepth;
	//int backgroundDepth[480][640];

	//------------------------------------------------------------
	//--------------------[Initiation Process]--------------------
	while( true )
	{
		streamInitDepth.readFrame( &frameDepthInit );
		streamInitColor.readFrame( &frameColorInit );

		const cv::Mat mImageDepth( frameDepthInit.getHeight(), frameDepthInit.getWidth(), CV_16UC1, (void*)frameDepthInit.getData());

        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepthInit );

        cv::imshow( "Depth Image (Init)", mScaledDepth );

        const cv::Mat mImageRGB(frameColorInit.getHeight(), frameColorInit.getWidth(), CV_8UC3, (void*)frameColorInit.getData());

        cv::Mat cImageBGR;
        cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );

        cv::imshow( "Color Image (Init)", cImageBGR );

		//--------------------[Get Average Distance]---------------------
		int depthVal = 0;
		int frameHeight = frameDepthInit.getHeight();
		int frameWidth = frameDepthInit.getWidth();
		//------------
		//backgroundDepth.resize(frameHeight * frameWidth);
		//---------------------------------------------------------------
		
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

		if( cv::waitKey(1) == 'q')
		{
			mImageDepth.copyTo(BackgroundFrame);
            break;
		}
	}

	streamInitDepth.destroy();
	streamInitColor.destroy();

	destroyWindow( "Depth Image (Init)" );
	destroyWindow( "Color Image (Init)" );

	//------------------------------------------------------------
	//------------------------------------------------------------
	VideoStream streamCaliDepth;
    streamCaliDepth.create( devAnyDevice, SENSOR_DEPTH );

	VideoStream streamCaliColor;
    streamCaliColor.create( devAnyDevice, SENSOR_COLOR );

	streamCaliDepth.setVideoMode( mModeDepth );
	streamCaliColor.setVideoMode( mModeColor );

	streamCaliDepth.start();
    streamCaliColor.start();

	//------------------------------------------------------------
	//-------------------[Calibration Process]--------------------

	namedWindow( "Depth Image (Init)",  CV_WINDOW_AUTOSIZE );
	namedWindow( "Color Image (Green)",  CV_WINDOW_AUTOSIZE );
    namedWindow( "Color Image (Init)",  CV_WINDOW_AUTOSIZE );
	namedWindow( "Color Image (Green-Binary)", CV_WINDOW_AUTOSIZE );
	namedWindow( "Depth Image (Init-Binary)", CV_WINDOW_AUTOSIZE );

	VideoFrameRef  frameDepthCali;
    VideoFrameRef  frameColorCali;

	streamCaliDepth.start();
	streamCaliColor.start();

	int centerxA, centeryA, centerxB, centeryB;

	vector<Mat> frameRGB;

	while( true )
	{
		streamCaliDepth.readFrame( &frameDepthCali );
		streamCaliColor.readFrame( &frameColorCali );

		const cv::Mat mImageColor( frameDepthCali.getHeight(), frameDepthCali.getWidth(), CV_8UC3, (void*)frameColorCali.getData());
		const cv::Mat mImageDepth( frameDepthCali.getHeight(), frameDepthCali.getWidth(), CV_16UC1, (void*)frameDepthCali.getData());

		split( mImageColor, frameRGB );

		cv::imshow( "Color Image (Green)", frameRGB[1]);

		cv::Mat frameColorThres( frameDepthCali.getHeight(), frameDepthCali.getWidth(), CV_8UC1 );
		threshold( frameRGB[1], frameColorThres, 100, 255, THRESH_BINARY );
		cv::imshow( "Color Image (Green-Binary)", frameColorThres );

        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepthInit );

        cv::imshow( "Depth Image (Init)", mScaledDepth );

        const cv::Mat mImageRGB(frameColorCali.getHeight(), frameColorCali.getWidth(), CV_8UC3, (void*)frameColorCali.getData());

        cv::Mat cImageBGR;
        cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
		
		cv::Mat mImageThres( frameDepthCali.getHeight(), frameDepthCali.getWidth(), CV_8UC1 );
		for(int i = 0; i < frameDepthCali.getHeight(); i++)
		{
			for(int j = 0; j < frameDepthCali.getWidth(); j++)
			{
				avgDist = BackgroundFrame.at<unsigned short>(i, j);
				int depthVal = mImageDepth.at<unsigned short>(i, j);

				if((depthVal <= avgDist-10) && (depthVal > 100))
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

	int displacementX = centerxA - centerxB;
	int displacementY = centeryA - centeryB;

	streamCaliDepth.destroy();
	streamCaliColor.destroy();

	destroyWindow( "Depth Image (Init)" );
	destroyWindow( "Color Image (Green)" );
    destroyWindow( "Color Image (Init)" );
	destroyWindow( "Color Image (Green-Binary)" );
	destroyWindow( "Depth Image (Init-Binary)" );

	//------------------------------------------------------------
	//------------------------------------------------------------

	VideoStream streamDepth;
    streamDepth.create( devAnyDevice, SENSOR_DEPTH );

	VideoStream streamColor;
    streamColor.create( devAnyDevice, SENSOR_COLOR );

	streamDepth.setVideoMode( mModeDepth );
	streamColor.setVideoMode( mModeColor );

	streamDepth.start();
    streamColor.start();

	namedWindow( "Depth Image",  CV_WINDOW_AUTOSIZE );
    namedWindow( "Color Image",  CV_WINDOW_AUTOSIZE );
	namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );

	int iMaxDepth = streamDepth.getMaxPixelValue();

    VideoFrameRef  frameColor;
	VideoFrameRef  frameDepth;

	OutX.clear();
	OutY.clear();

	vector<int> OldOutX, OldOutY;
	OldOutX.clear();
	OldOutY.clear();

	//------------------------------------------------------------
	//-----------------------[Main Process]-----------------------
	while( true ) 
    {
        streamDepth.readFrame( &frameDepth );
        streamColor.readFrame( &frameColor );

        const cv::Mat mImageDepth( frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());

        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepth );

		////////////////////////////////////////////////////////////////////////////////////////////
		//---------------------[Downsampling]-------------------------------------------------------
		double min;
		double max;
		cv::minMaxIdx(mImageDepth, &min, &max);
		cv::Mat adjMap;
		// expand your range to 0..255. Similar to histEq();
		float scale = 255 / (max-min);
		mImageDepth.convertTo(adjMap,CV_8UC1, scale, -min*scale); 

		// this is great. It converts your grayscale image into a tone-mapped one, 
		// much more pleasing for the eye
		// function is found in contrib module, so include contrib.hpp 
		// and link accordingly
		cv::Mat falseColorsMap;
		applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);

		cv::imshow("Out", falseColorsMap);
		//------------------------------------------------------------------------------------------
		////////////////////////////////////////////////////////////////////////////////////////////

        cv::imshow( "Depth Image", mScaledDepth );
		cv::imshow( "Depth Image2", adjMap );

        const cv::Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());

        cv::Mat cImageBGR;
        cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
		
		int lowestVal = 10000;

		//-------------[Threshold]-----------------
		cv::Mat mImageThres( frameDepth.getHeight(), frameDepth.getWidth(), CV_8UC1 );

		int backgroundPxlCount = 0;
		for(int i = 0; i < frameDepth.getHeight(); i++)
		{
			for(int j = 0; j < frameDepth.getWidth(); j++)
			{
				int depthVal = mImageDepth.at<unsigned short>(i, j);

				avgDist = BackgroundFrame.at<unsigned short>(i, j);

				int xShift = j+displacementX;
				int yShift = i+displacementY;
				int frameHeight = frameDepth.getHeight();
				int frameWidth = frameDepth.getWidth();

				if((depthVal > (avgDist-8)) && (depthVal <= (avgDist-5)))
				{
					if((frameWidth > xShift ) && (xShift >= 0) && (frameHeight > yShift) && (yShift >= 0))
						mImageThres.data[mImageThres.step[0]*yShift + mImageThres.step[1]*xShift] = 255;
					//mImageThres.data[mImageThres.step[0]*i + mImageThres.step[1]*j] = 255;
				}
				else
				{
					if((frameWidth > xShift ) && (xShift >= 0) && (frameHeight > yShift) && (yShift >= 0))
						mImageThres.data[mImageThres.step[0]*yShift + mImageThres.step[1]*xShift] = 0;
					//mImageThres.data[mImageThres.step[0]*i + mImageThres.step[1]*j] = 0;
				}

				backgroundPxlCount++;
				//if((depthVal < lowestVal) && (depthVal > 0))
				//	lowestVal = depthVal;
			}
		}

		//cout << lowestVal << endl;

		//blur( mImageThres, mImageThres, Size(3,3) );
		GaussianBlur( mImageThres, mImageThres, Size(3,3), 0, 0 );
		
		fingerDetection( mImageThres, cImageBGR, OldOutX, OldOutY );

		cv::imshow("Thresholded Image", mImageThres);
		//----------------------------------------

        cv::imshow( "Color Image", cImageBGR );

		sendCoord();

        if( cv::waitKey(1) == 'q')
		{
            break;
		}
		//----------------------------------
		//Optimize touching here:
		/*for(int i = 0; i < OutX.size(); i++)
		{
			for(int j = 0; j < OldOutX.size(); i++)
			{
				int distance = sqrt(pow((double)(OutX[i]-OldOutX[i]),2) + pow((double)(OutY[i]-OldOutY[i]),2) );

				if(distance < 20)
				{

				}
			}
		}*/
		//----------------------------------
		//Simulate mouse/touch here:
		if( OutX.size() > 0 )
		{
			MouseMove( OutX[0], OutY[0] );
			//LeftClick();
		}
		//----------------------------------
		OldOutX.clear();
		OldOutY.clear();
		OldOutX = OutX;
		OldOutY = OutY;
		OutX.clear();
		OutY.clear();
    }
	//------------------------------------------------------------
	//------------------------------------------------------------

    streamDepth.destroy();
    streamColor.destroy();

    devAnyDevice.close();

    openni::OpenNI::shutdown();

	return 0;
}

//----------------------------------------------------------------
//----------------------------------------------------------------

void fingerDetection(cv::Mat binaryImg, cv::Mat colorImg, vector<int> OldOutX, vector<int> OldOutY)
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

	for( int i = 0; i< contours.size(); i++ )
    {
		int centerx = (mu[i].m10/mu[i].m00);
		int centery = (mu[i].m01/mu[i].m00);
	   
		area = contourArea(contours[i], false);
		Point2f encCenter;

		if((area > 50) && (area < 1000))
		{
			float furthestReach;
			///////////////////////////////////////////////////////
			//Draw stuff
			Point pt, ptx1, ptx2, pty1, pty2;
			pt.x = centerx;
			pt.y = centery;
			ptx1.x = centerx - 10; ptx1.y = centery;
			ptx2.x = centerx + 10; ptx2.y = centery;
			pty1.x = centerx; pty1.y = centery - 10;
			pty2.x = centerx; pty2.y = centery + 10;

			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			Scalar white = (255, 255, 255);
			Scalar black = (0, 0, 0);

			if( OldOutX.size() > 0)
			{
				for(int i = 0; i < OldOutX.size(); i++)
				{
					int distance = sqrt(pow((double)(centerx-OldOutX[i]),2) + pow((double)(centery-OldOutY[i]),2) );

					if(distance < 20)
					{
						drawContours( colorImg, contours, i, color, 2, 8, hierarchy, 0, Point() );
					}
				}
			}

			line( colorImg, ptx1, ptx2, color, 2 );
			line( colorImg, pty1, pty2, color, 2 );
			///////////////////////////////////////////////////////

			OutX.push_back(centerx);
			OutY.push_back(centery);
		}
	}
}

//----------------------------------------------------------------
//----------------------------------------------------------------

void Calibration(cv::Mat binaryImg, cv::Mat colorImg, cv::Point cornerC, cv::Point cornerD)
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

	for( int i = 0; i< contours.size(); i++ )
    {
		int centerx = (mu[i].m10/mu[i].m00);
		int centery = (mu[i].m01/mu[i].m00);
	   
		area = contourArea(contours[i], false);
		Point2f encCenter;

		if(area > 100)
		{
			float furthestReach;
			///////////////////////////////////////////////////////
			//Draw stuff
			Point pt, ptx1, ptx2, pty1, pty2;
			pt.x = centerx;
			pt.y = centery;
			ptx1.x = centerx - 10; ptx1.y = centery;
			ptx2.x = centerx + 10; ptx2.y = centery;
			pty1.x = centerx; pty1.y = centery - 10;
			pty2.x = centerx; pty2.y = centery + 10;

			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			Scalar white = (255, 255, 255);
			Scalar black = (0, 0, 0);
			drawContours( colorImg, contours, i, color, 2, 8, hierarchy, 0, Point() );

			line( colorImg, ptx1, ptx2, color, 2 );
			line( colorImg, pty1, pty2, color, 2 );
			///////////////////////////////////////////////////////
		}
	}
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

void sendCoord()
{
	
	for(int i = 0; i < OutX.size(); i++)
	{
		int centerX = OutX[i];
		int centerY = OutY[i];
	}

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