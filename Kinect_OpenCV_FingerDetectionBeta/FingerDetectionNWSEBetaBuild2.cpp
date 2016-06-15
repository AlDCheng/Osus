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
void shapeFeatures(cv::Mat binaryImg, cv::Mat colorImg, int classtype);
void sendCoord();

void MouseMove(int x, int y);
void LeftClick();
void LeftHold();
void LeftRelease();

vector<int> OutX, OutY;
int UpX, UpY, DownX, DownY, CurX, CurY;
CvSVM SVMFinger;

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


int _tmain(int argc, _TCHAR* argv[])
{
	FILE *fptrI = fopen("C:\\Users\\Alan\\Documents\\ShapeFeatures.csv","w");
	fprintf(fptrI, "Classtype, Area, Perimeter, Circularity, Extent\n");
	fclose(fptrI);

	Mat input = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2014\\SVM\\Shape Features\\Fingers.bmp", 1);
	Mat input2 = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2014\\SVM\\Shape Features\\NotFingers.bmp", 1);
	Mat inputF = imread("C:\\Users\\Alan\\Pictures\\Science Fair 2014\\SVM\\Shape Features\\ImageFeaturesBinaryF.bmp", 1);
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

	OpenNI::initialize();

	Device devAnyDevice;
    devAnyDevice.open(ANY_DEVICE);

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

	
	//----------------------[Initial Streams]---------------------
	VideoStream streamInitDepth;
    streamInitDepth.create( devAnyDevice, SENSOR_DEPTH );

	VideoStream streamInitColor;
    streamInitColor.create( devAnyDevice, SENSOR_COLOR );

	/*if( devAnyDevice.isImageRegistrationModeSupported(
        IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        devAnyDevice.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }*/


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
				initCount++;
			}
		}
		avgDist = depthVal / ((frameHeight) * (frameWidth));

		cout << "Average Distance: " << avgDist << endl;
		//---------------------------------------------------------------
		char CurrentXY[8], UpXY[8], DownXY[8];
		sprintf(CurrentXY, "%d,%d", CurX, CurY);
		sprintf(UpXY, "%d,%d", UpX, UpY);
		sprintf(DownXY, "%d,%d", DownX, DownY);

		setMouseCallback("Color Image (Init)", CallBackFunc, NULL);
		putText(cImageBGR, CurrentXY, cvPoint(frameWidth-100, frameHeight-30), 
			FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 255, 0));
		putText(cImageBGR, UpXY, cvPoint(UpX, UpY-20), 
			FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 255, 0));
		putText(cImageBGR, DownXY, cvPoint(DownX, DownY-20), 
			FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(255, 255, 0));
		rectangle(cImageBGR, cvPoint(UpX, UpY), cvPoint(DownX, DownY), cvScalar(255,255,0), 2);
		//---------------------------------------------------------------

		cv::imshow( "Color Image (Init)", cImageBGR );

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
	UpY = UpY - 30;
	UpX = UpX - 40;
	DownY = DownY - 5;
	DownX = DownX + 0;
	//------------------------------------------------------------

	VideoStream streamDepth;
    streamDepth.create( devAnyDevice, SENSOR_DEPTH );

	VideoStream streamColor;
    streamColor.create( devAnyDevice, SENSOR_COLOR );

	streamDepth.setVideoMode( mModeDepth );
	streamColor.setVideoMode( mModeColor );

	streamDepth.start();
    streamColor.start();

    namedWindow( "Color Image",  CV_WINDOW_AUTOSIZE );
	namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );

	namedWindow( "Mouse Canvas", CV_WINDOW_AUTOSIZE );

	int iMaxDepth = streamDepth.getMaxPixelValue();

    VideoFrameRef  frameColor;
	VideoFrameRef  frameDepth;

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

	Mat canvas(768, 1366, CV_8UC3);

	bool mouseClick = false;

	//------------------------------------------------------------
	//-----------------------[Main Process]-----------------------
	while( true ) 
    {
        streamDepth.readFrame( &frameDepth );
        streamColor.readFrame( &frameColor );

        const cv::Mat mImageDepth( frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());

        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepth );

        const cv::Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());

        cv::Mat cImageBGR;
        cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
		
		//-------------[Threshold]-----------------
		cv::Mat mImageThres( frameDepth.getHeight(), frameDepth.getWidth(), CV_8UC1 );

		int backgroundPxlCount = 0;
		for(int i = UpY; i < DownY; i++)
		{
			for(int j = UpX; j < DownX; j++)
			{
				int depthVal = mImageDepth.at<unsigned short>(i, j);

				avgDist = BackgroundFrame.at<unsigned short>(i, j)-2;

				if((depthVal > (avgDist-16)) && (depthVal <= (avgDist-5)))
				{
					//mImageThres.data[mImageThres.step[0]*i + mImageThres.step[1]*j] = 255;
					mImageThres.at<uchar>(i, j) = 255;
				}
				else
				{
					//mImageThres.data[mImageThres.step[0]*i + mImageThres.step[1]*j] = 0;
					mImageThres.at<uchar>(i, j) = 0;
				}

				backgroundPxlCount++;
			}
		}
		GaussianBlur( mImageThres, mImageThres, Size(3,3), 0, 0 );
		//GaussianBlur( mImageThres, mImageThres, Size(3,3), 0, 0 );
		
		fingerDetection( mImageThres, cImageBGR, OldOutX, OldOutY);

		cv::imshow("Thresholded Image", mImageThres);
		//----------------------------------------

		sendCoord();

        if( cv::waitKey(1) == 'q')
		{
            break;
		}
		
		//----------------------------------
		
		//----------------------------------
		//Simulate mouse/touch here:
		
		double windowLength = DownX - UpX;
		double windowHeight = DownY - UpY;

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
			
			averageX = windowLength - ((averageX / OutX.size()) - UpX);
			averageY = ((averageY / OutY.size()) - UpY);

			averageX = windowLength - averageX;
			averageY = windowHeight - averageY;
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
			averageX = windowLength - ((averageX / OutX.size()) - UpX);
			averageY = ((averageY / OutY.size()) - UpY);

			averageX = windowLength - averageX;
			averageY = windowHeight - averageY;

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
			line(canvas, kalmanv[i], kalmanv[i+1], Scalar(0,255,0), 1);
		}

		//------------------------------------------------

		//1366 x 768
		double shiftX, shiftY;
		shiftX = 1366/windowLength;
		shiftY = 768/windowHeight;

		int ptX = statePt.x *shiftX;
		int ptY = statePt.y *shiftY;

		int MptX = measPt.x *shiftX;
		int MptY = measPt.y *shiftY;

		canvas = Scalar::all(0);
		
		line( canvas, Point( ptX - 5, ptY - 5 ), Point( ptX + 5, ptY + 5 ), 
			Scalar(255,255,255), 2, CV_AA, 0);
		line( canvas, Point( ptX + 5,  ptY - 5 ), Point( ptX - 5, ptY + 5 ),
			Scalar(255,255,255), 2, CV_AA, 0 );

		line( canvas, Point( MptX - 5, MptY - 5 ), Point( MptX + 5, MptY + 5 ), 
			Scalar(0,0,255), 2, CV_AA, 0);
		line( canvas, Point( MptX + 5,  MptY - 5 ), Point( MptX - 5, MptY + 5 ),
			Scalar(0,0,255), 2, CV_AA, 0 );

		for (int i = 0; i < mousev.size()-1; i++)
		{
			line(canvas, Point(mousev[i].x*shiftX, mousev[i].y*shiftY),
				Point(mousev[i+1].x*shiftX, mousev[i+1].y*shiftY), Scalar(255,255,0), 1);
		}
		for (int i = 0; i < kalmanv.size()-1; i++)
		{
			line(canvas, Point(kalmanv[i].x*shiftX, kalmanv[i].y*shiftY),
				Point(kalmanv[i+1].x*shiftX, kalmanv[i+1].y*shiftY), Scalar(0,255,0), 1);
		}

		imshow( "Mouse Canvas", canvas );

		if(OutX.size() > 0)
		{

			//circle(cImageBGR, Point(averageX, averageY), 10, Scalar(0, 255, 255), 3);
			MouseMove( (/*1366 - */(statePt.x *shiftX)), (/*768 - */(statePt.y *shiftY)) );
			//LeftClick();
			if(mouseClick == false)
			{
				LeftHold( );
				mouseClick = true;
			}
		}
		
		cv::imshow( "Color Image", cImageBGR );
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
		int perimeter = (int)arcLength(contours[i], true);
		double circularity = (4 * 3.14 * area)/(perimeter * perimeter);

		CvBox2D box;
		box = minAreaRect(contours[i]);
		double extent = (area/(box.size.height*box.size.width));

		//if((area > 15) && (area < 200) && (circularity > 0.1) && (circularity < 0.9))
		Mat sampleMat = (Mat_<float>(1,4) << extent, circularity, area, perimeter);
		float response = SVMFinger.predict(sampleMat);
		if((response == 1) && (area > 30) && (area < 400))
		{
			float furthestReach;
			///////////////////////////////////////////////////////
			//Draw stuff
			Point pt, ptx1, ptx2, pty1, pty2;
			centerx = (double)centerx*0.97+3/*+10*/;
			centery = (double)centery*1.05+20/*+10*/;
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
				for(int k = 0; k < OldOutX.size(); k++)
				{
					int distance = sqrt(pow((double)(centerx-OldOutX[k]),2) + pow((double)(centery-OldOutY[k]),2) );

					if(distance < 20)
					{
						drawContours( colorImg, contours, i, color, 2, 8, hierarchy, 0, Point() );
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