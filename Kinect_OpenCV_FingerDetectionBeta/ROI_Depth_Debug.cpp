// Kinect_OpenCV_FingerDetectionBeta.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
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
	mModeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_100_UM );

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
	FILE *fptr = fopen("C:\\Users\\Alan\\Documents\\ROIDepthDebug.txt","w");

	int dArray[100][100];
	
	//-------------------------------------- 
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

		//-------------[Draw ROI]-----------------
		Point pt1, pt2;
		pt1.x = 270; pt1.y = 190;
		pt2.x = 370; pt2.y = 290;
		rectangle(cImageBGR, pt1, pt2, CVX_RED, 2);
		//----------------------------------------

        cv::imshow( "Color Image", cImageBGR );

        if( cv::waitKey(1) == 'q')
		{
			//--------------[ROI Output Numerical Values]--------------

			for(int i = 270; i < 370; i++)
			{
				for(int j = 190; j < 290; j++)
				{
					int depthVal;
					//depthVal = mImageDepth.data[i+j*480];
					depthVal = mImageDepth.at<unsigned short>(j, i);
					//unsigned char* depthMap_data_ptr = mImageDepth.data; 
					//depthVal = depthMap_data_ptr[i + j*480]; 

					fprintf(fptr, "%d ", depthVal);
					dArray[j-190][i-270] = depthVal;
				}
				fprintf(fptr, "\n");
			}
			//--------------------------------------------------------

            break;
		}
    }
	
	//------------[File Close]-------------
	fclose(fptr);
	cv::Mat mImageTest( 100, 100, CV_8UC1 );
	for(int i = 0; i < 100; i++)
	{
		for(int j = 0; j < 100; j++)
		{
			if((dArray[i][j] > 950)||(dArray[i][j] < 900))
			{
				mImageTest.data[mImageTest.step[0]*i + mImageTest.step[1]*j + 0] = 255;
			}
			else
				mImageTest.data[mImageTest.step[0]*i + mImageTest.step[1]*j + 0] = 0;
		}
	}
	namedWindow( "Test Image", CV_WINDOW_AUTOSIZE );
	cv::imshow("Test Image", mImageTest);
	waitKey();
	//-------------------------------------

    streamDepth.destroy();
    streamColor.destroy();

    devAnyDevice.close();

    openni::OpenNI::shutdown();

	return 0;
}

