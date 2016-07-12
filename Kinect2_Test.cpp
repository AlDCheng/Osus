//Adaptation from Japanese sample code

#include "opencv2/opencv.hpp"
#include <math.h>
#include <Windows.h>
#include <Kinect.h>

IKinectSensor* pSensor;
HRESULT hResult;
IColorFrameSource* pColorSource;
IColorFrameReader* pColorReader;
IFrameDescription* pColorDescription;
IDepthFrameSource* pDepthSource;
IDepthFrameReader* pDepthReader;
IFrameDescription* pDescription;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

inline int KinectInit()
{
	cv::setUseOptimized(true);

	// Sensor
	hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)) {
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	//--------------------[Color]---------------------------
	// Source
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	//--------------------[Depth]---------------------------
	// Source
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	hResult = pDepthSource->get_FrameDescription(&pDescription);
	if (FAILED(hResult)) {
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

}

int main()
{
	KinectInit();

	//-------------------------[Color Initialization]--------------------------
	int cWidth = 0;
	int cHeight = 0;
	pColorDescription->get_Width(&cWidth); // 1920
	pColorDescription->get_Height(&cHeight); // 1080
	unsigned int colorBufferSize = cWidth * cHeight * 4 * sizeof(unsigned char);

	cv::Mat colorBufferMat(cHeight, cWidth, CV_8UC4);
	cv::Mat colorMat(cHeight / 2, cWidth / 2, CV_8UC4);

	//-------------------------[Depth Initialization]--------------------------
	int dWidth = 0;
	int dHeight = 0;
	pDescription->get_Width(&dWidth); // 512
	pDescription->get_Height(&dHeight); // 424
	unsigned int bufferSize = dWidth * dHeight * sizeof(unsigned short);

	// Range ( Range of Depth is 500-8000[mm], Range of Detection is 500-4500[mm] ) 
	unsigned short min = 0;
	unsigned short max = 0;
	pDepthSource->get_DepthMinReliableDistance(&min); // 500
	pDepthSource->get_DepthMaxReliableDistance(&max); // 4500
	std::cout << "Range : " << min << " - " << max << std::endl;

	//-------------------------------------------------------------------------
	
	cv::Mat backgroundMat(dHeight, dWidth, CV_16UC1);
	cv::Mat bufferMat(dHeight, dWidth, CV_16UC1);
	cv::Mat binaryMat(dHeight, dWidth, CV_8UC1);
	cv::Mat depthMat(dHeight, dWidth, CV_8UC1);

	//-------------------------------------------------------------------------

	cv::namedWindow("Color");
	cv::namedWindow("Depth");

	IColorFrame* pColorFrame = nullptr;
	IDepthFrame* pDepthFrame = nullptr;

	UINT16* backBuffer = nullptr;
	cv::Mat backFrame(dHeight, dWidth, CV_16UC1);

	//-------------------------------[Get Background Loop]-----------------------------------
	while (1) {
		// Frame
		pColorFrame = nullptr;
		pDepthFrame = nullptr;

		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult))
		{
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult))
			{
				cv::resize(colorBufferMat, colorMat, cv::Size(), 0.5, 0.5);
			}
		}
		
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult))
		{
			hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));
			if (SUCCEEDED(hResult))
			{
				pDepthFrame->AccessUnderlyingBuffer(&bufferSize, &backBuffer);
				for (int i = 0; i < dHeight; i++)
				{
					for (int j = 0; j < dWidth; j++)
					{	
						backFrame.at<UINT16>(i, j) = backBuffer[i * dWidth + j];
					}
				}
				bufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
			}
		}
		
		SafeRelease(pColorFrame);
		SafeRelease(pDepthFrame);

		cv::imshow("Color", colorMat);
		cv::imshow("Depth", depthMat);

		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	cv::destroyAllWindows();

	//-------------------------------[Main Loop]-----------------------------------

	cv::namedWindow("Color");
	cv::namedWindow("Depth");
	cv::namedWindow("Binary");

	while (1) {
		// Frame
		pColorFrame = nullptr;
		pDepthFrame = nullptr;

		IFrameDescription* depthFrameDescription = nullptr;

		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult))
		{
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult))
			{
				cv::resize(colorBufferMat, colorMat, cv::Size(), 0.5, 0.5);
			}
		}

		cv::Mat img(dHeight, dWidth, CV_8UC1);

		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult)) {

			UINT16* buffer = nullptr;
			hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&buffer));
			if (SUCCEEDED(hResult))
			{
				for (int i = 0; i < dHeight; i++)
				{
					for (int j = 0; j < dWidth; j++)
					{	
						int curDepthVal = buffer[i * dWidth + j];
						int backDepthVal = backFrame.at<UINT16>(i, j);
						if (std::abs(backDepthVal - curDepthVal) < 20)
							img.at<uchar>(i, j) = 255;
						else
							img.at<uchar>(i, j) = 0;
					}
				}

				pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));
				bufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
			}
			
		}

		SafeRelease(pColorFrame);
		SafeRelease(pDepthFrame);

		cv::imshow("Color", colorMat);
		cv::imshow("Depth", depthMat);

//		BackgroundSubtration(backgroundMat, bufferMat, binaryMat);

//		cv::imshow("Binary", binaryMat);
		cv::imshow("Binary", img);

		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	SafeRelease(pColorSource);
	SafeRelease(pColorReader);
	SafeRelease(pColorDescription);

	SafeRelease(pDepthSource);
	SafeRelease(pDepthReader);
	SafeRelease(pDescription);
	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();

	return 0;
}
