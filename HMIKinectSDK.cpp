// HMIKinectSDK.cpp : Defines the entry point for the console application.
//

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
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\contrib\contrib.hpp>

using namespace std;
using namespace cv;
 
int main(int argc,char * argv[])  
{  
    IplImage *colorImage=NULL;  
    colorImage = cvCreateImage(cvSize(640, 480), 8, 3);  

    //Initialize NUI
    HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR);  
    if( hr != S_OK )  
    {  
        cout<<"NuiInitialize failed"<<endl;  
        return hr;  
    }  
    //Crete Handles (color and depth streams)
    HANDLE h1 = CreateEvent( NULL, TRUE, FALSE, NULL );
    HANDLE h2 = NULL;

    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480,0,2,h1,&h2);//Open KINECT stream

    if( FAILED( hr ) )//Check for failure
    {  
        cout<<"Could not open color image stream video"<<endl;  
        NuiShutdown();  
        return hr;  
    }  

    //Main Loop
    while(1)  
    {  
        const NUI_IMAGE_FRAME * pImageFrame = NULL;  

        if (WaitForSingleObject(h1, INFINITE)==0)//判断是否得到了新的数据  
        {  
            NuiImageStreamGetNextFrame(h2, 0, &pImageFrame);//得到该帧数据  
            INuiFrameTexture *pTexture = pImageFrame->pFrameTexture;  
            NUI_LOCKED_RECT LockedRect;  
            pTexture->LockRect(0, &LockedRect, NULL, 0);//提取数据帧到LockedRect，它包括两个数据对象：pitch每行字节数，pBits第一个字节地址  
            if( LockedRect.Pitch != 0 )  
            {  
                cvZero(colorImage);  
                for (int i=0; i<480; i++)  
                {  
                    uchar* ptr = (uchar*)(colorImage->imageData+i*colorImage->widthStep);  
                    BYTE * pBuffer = (BYTE*)(LockedRect.pBits)+i*LockedRect.Pitch;//每个字节代表一个颜色信息，直接使用BYTE  
                    for (int j=0; j<640; j++)  
                    {  
                        ptr[3*j] = pBuffer[4*j];//内部数据是4个字节，0-1-2是BGR，第4个现在未使用  
                        ptr[3*j+1] = pBuffer[4*j+1];  
                        ptr[3*j+2] = pBuffer[4*j+2];  
                    }  
                }  

                cvShowImage("colorImage", colorImage);//Display Image RGB

            }  
            else  
            {  
                cout<<"Buffer length of received texture is bogus\r\n"<<endl;  
            }   
            NuiImageStreamReleaseFrame( h2, pImageFrame );  
        }  

        if (cvWaitKey(30) == 27)  
            break;  
    }    
    NuiShutdown();  
    return 0;  
}  