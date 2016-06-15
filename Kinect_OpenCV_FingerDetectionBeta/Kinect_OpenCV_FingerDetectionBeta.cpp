// Kinect_OpenCV_FingerDetectionBeta.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "OpenNI.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace openni;
using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
{
	OpenNI::initialize();

	Device devAnyDevice;
    devAnyDevice.open(ANY_DEVICE );

	VideoStream streamDepth;
    streamDepth.create( devAnyDevice, SENSOR_DEPTH );

	VideoStream streamColor;
    streamColor.create( devAnyDevice, SENSOR_COLOR );

	VideoMode mModeDepth;

	int depthWidth = mModeDepth.getResolutionX();
	int depthHeight = mModeDepth.getResolutionY();

	mModeDepth.setResolution( 640, 480 );
	mModeDepth.setFps( 30 );
	mModeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );

	streamDepth.setVideoMode( mModeDepth);

	VideoMode mModeColor;
    mModeColor.setResolution( 640, 480 );
    mModeColor.setFps( 30 );
    mModeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );

	streamColor.setVideoMode( mModeColor);

	if( devAnyDevice.isImageRegistrationModeSupported(
        IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        devAnyDevice.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }

	streamDepth.start();
    streamColor.start();

	namedWindow( "Depth Image",  CV_WINDOW_AUTOSIZE );
    namedWindow( "Color Image",  CV_WINDOW_AUTOSIZE );

	int iMaxDepth = streamDepth.getMaxPixelValue();

	VideoFrameRef  frameDepth;
    VideoFrameRef  frameColor;

	//------------[File Debug]--------------



	//-------------------------------------- 
	while( true ) 
    {
        streamDepth.readFrame( &frameDepth );
        streamColor.readFrame( &frameColor );

        const cv::Mat mImageDepth( frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());

		//--------------[ROI Output Numerical Values]--------------
		 


		///--------------------------------------------------------



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

        cv::imshow( "Color Image", cImageBGR );

        if( cv::waitKey(1) == 'q')
            break;
    }

    streamDepth.destroy();
    streamColor.destroy();

    devAnyDevice.close();

    openni::OpenNI::shutdown();

	return 0;
}

