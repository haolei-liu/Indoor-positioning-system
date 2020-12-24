// OpenCV.cpp : 义控制台应用程序的入口点。
//
//BIG5 TRANS ALLOWED
#include "windows.h"
#include <opencv2/opencv.hpp>
#include<opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include <process.h>
#include"CameraApi.h"
#pragma comment(lib, "./MVCAMSDK_X64.lib")

using namespace std;
using namespace cv;

UINT            m_threadID;		//图像抓取线程的ID
HANDLE          m_hDispThread;	//图像抓取线程的句柄
BOOL            m_bExit = FALSE;		//用来通知图像抓取线程结束
CameraHandle    m_hCamera;		//相机句柄，多个相机同时使用时，可以用数组代替	
BYTE* m_pFrameBuffer; //用于将原始图像数据转换为RGB的缓冲区
tSdkFrameHead   m_sFrInfo;		//用于保存当前图像帧的帧头信息

int	            m_iDispFrameNum;	//用于记录当前已经显示的图像帧的数量
float           m_fDispFps;			//显示帧率
float           m_fCapFps;			//捕获帧率
tSdkFrameStatistic  m_sFrameCount;
tSdkFrameStatistic  m_sFrameLast;
int					m_iTimeLast;
char		    g_CameraName[64];

//鼠标抓取
bool flag = false;
int xvalue = 0;
int yvalue = 0;
Mat matImage;
void mousecallback(int event, int x, int y, int flags, void* userdata);

void img_loc(double H, float u_, float v_, float& X, float& Y)
{
	// float internal_reference[], float Rotation[], int translation[], 
	float f_x = 1745.2;//internal_reference[0]//
	float f_y = 1745.8;//internal_reference[4]//
	float c_x = 1614.4;//internal_reference[2]//954.1
	float c_y = 966.9441;//internal_reference[5]//537.1
	float R[3][3] = {0.9117,-0.1365,0.3876,
					-0.4084, -0.4056, 0.8177 ,
					0.0456, -0.9038, -0.4256 };
	/*----------------------------------------------------*/
	float a_1 = R[0][0];//1.0;//Rotation[0]
	float a_2 = R[0][1];//0.0;//Rotation[1]
	float a_3 = R[0][2];//0.0;//Rotation[2]
	float a_4 = R[1][0];//0.0;//Rotation[3]
	float a_5 = R[1][1];//-0.5736;//Rotation[4]
	float a_6 = R[1][2];//0.8192;//Rotation[5]
	float a_7 = R[2][0];//0.0175;//Rotation[6]
	float a_8 = R[2][1];//-0.8192;//Rotation[7]
	float a_9 = R[2][2];//-0.5736;//Rotation[8]
	float delta_x = 2.7546;//translation[0]
	float delta_y = -5.1480;//translation[1]
	float delta_z = 247.1551;//translation[2] //75.8+105

	float k1 = -0.1156;
	float k2 = 0.1183;
	float k3 = -0.0019;
	float p1 = -0.0023;
	float p2 = 0;

	//先把畸变的u，v变成真实u,v
	float x_ = (u_ - c_x) / f_x;
	float y_ = (v_-c_y) / f_y;
	float r = sqrt(x_ * x_ + y_ * y_);
	float x_corr = (1+k1*pow(r,2)+k2*pow(r,4)+k3*pow(r,6))*x_+2*p1*x_*y_+p2*(pow(r,2)+2*pow(x_,2));
	float y_corr = (1 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6)) * y_ + p1 * (pow(r, 2) + 2 * pow(y_, 2)) + 2 * p2 * x_ * y_;
	float u = f_x * x_corr + c_x;
	float v = f_y * y_corr + c_y;

	float X_Z = (u_ - c_x) / f_x;
	float Y_Z = (v_ - c_y) / f_y;
	float Z = (H  - delta_z) / (a_7 * X_Z + a_8 * Y_Z + a_9);

	if (H > 0)
	{
		X = (a_1 * X_Z + a_2 * Y_Z + a_3) * Z + delta_x;
		Y = (a_4 * X_Z + a_5 * Y_Z + a_6) * Z + delta_y;
	}
}

/*
USE_CALLBACK_GRAB_IMAGE
如果需要使用回调函数的方式获得图像数据，则反注释宏定义USE_CALLBACK_GRAB_IMAGE.
我们的SDK同时支持回调函数和主动调用接口抓取图像的方式。两种方式都采用了"零拷贝"机制，以最大的程度的降低系统负荷，提高程序执行效率。
但是主动抓取方式比回调函数的方式更加灵活，可以设置超时等待时间等，我们建议您使用 uiDisplayThread 中的方式
*/
//#define USE_CALLBACK_GRAB_IMAGE 
#ifdef USE_CALLBACK_GRAB_IMAGE
/*图像抓取回调函数*/
IplImage* g_iplImage = NULL;

void _stdcall GrabImageCallback(CameraHandle hCamera, BYTE* pFrameBuffer, tSdkFrameHead* pFrameHead, PVOID pContext)
{

	CameraSdkStatus status;


	//将获得的原始数据转换成RGB格式的数据，同时经过ISP模块，对图像进行降噪，边沿提升，颜色校正等处理。
	//我公司大部分型号的相机，原始数据都是Bayer格式的
	status = CameraImageProcess(hCamera, pFrameBuffer, m_pFrameBuffer, pFrameHead);

	//分辨率改变了，则刷新背景
	if (m_sFrInfo.iWidth != pFrameHead->iWidth || m_sFrInfo.iHeight != pFrameHead->iHeight)
	{
		m_sFrInfo.iWidth = pFrameHead->iWidth;
		m_sFrInfo.iHeight = pFrameHead->iHeight;
	}

	if (status == CAMERA_STATUS_SUCCESS)
	{
		//调用SDK封装好的显示接口来显示图像,您也可以将m_pFrameBuffer中的RGB数据通过其他方式显示，比如directX,OpengGL,等方式。
		CameraImageOverlay(hCamera, m_pFrameBuffer, pFrameHead);
		if (g_iplImage)
		{
			cvReleaseImageHeader(&g_iplImage);
		}
		g_iplImage = cvCreateImageHeader(cvSize(pFrameHead->iWidth, pFrameHead->iHeight), IPL_DEPTH_8U, sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? 1 : 3);
		cvSetData(g_iplImage, m_pFrameBuffer, pFrameHead->iWidth * (sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? 1 : 3));
		cvShowImage(g_CameraName, g_iplImage);
		m_iDispFrameNum++;
		waitKey(30);
	}

	memcpy(&m_sFrInfo, pFrameHead, sizeof(tSdkFrameHead));

}

#else 
/*图像抓取线程，主动调用SDK接口函数获取图像*/
UINT WINAPI uiDisplayThread(LPVOID lpParam)
{
	tSdkFrameHead 	sFrameInfo;
	CameraHandle    hCamera = (CameraHandle)lpParam;
	BYTE* pbyBuffer;
	CameraSdkStatus status;
	IplImage* iplImage = NULL;
	while (!m_bExit)
	{

		if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
		{
			//将获得的原始数据转换成RGB格式的数据，同时经过ISP模块，对图像进行降噪，边沿提升，颜色校正等处理。
			//我公司大部分型号的相机，原始数据都是Bayer格式的
			status = CameraImageProcess(hCamera, pbyBuffer, m_pFrameBuffer, &sFrameInfo);//连续模式

			//分辨率改变了，则刷新背景
			if (m_sFrInfo.iWidth != sFrameInfo.iWidth || m_sFrInfo.iHeight != sFrameInfo.iHeight)
			{
				m_sFrInfo.iWidth = sFrameInfo.iWidth;
				m_sFrInfo.iHeight = sFrameInfo.iHeight;
				//图像大小改变，通知重绘
			}

			if (status == CAMERA_STATUS_SUCCESS)
			{
				//调用SDK封装好的显示接口来显示图像,您也可以将m_pFrameBuffer中的RGB数据通过其他方式显示，比如directX,OpengGL,等方式。
				CameraImageOverlay(hCamera, m_pFrameBuffer, &sFrameInfo);

				// 由于SDK输出的数据默认是从底到顶的，转换为Opencv图片需要做一下垂直镜像
				CameraFlipFrameBuffer(m_pFrameBuffer, &sFrameInfo, 1);

#if 0
				if (iplImage)
				{
					cvReleaseImageHeader(&iplImage);
				}
				iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? 1 : 3);
				cvSetData(iplImage, m_pFrameBuffer, sFrameInfo.iWidth * (sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? 1 : 3));
				cvShowImage(g_CameraName, iplImage);
#else
				matImage = Mat(
					cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
					sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
					m_pFrameBuffer
				);
				//识别红色小球
				int count = 0;
				float center_row = 0, center_col = 0;
				for (int row = 0; row < matImage.rows; row++) {
					uchar* line = matImage.ptr<uchar>(row);
					for (int col = 0; col < matImage.cols; col++) {
						int b = line[col*3];
						int g = line[col * 3+1];
						int r = line[col * 3+2];
						if (b < 70 && g < 70 && r>80&&r>2.5*b&&r>2.5*g) {
							count++;
							center_row += row;
							center_col += col;
						}
					}
				}

				if (count != 0) {
					center_row /= count;
					center_col /= count;
					circle(matImage, Point(center_col, center_row),50, Scalar(255, 0, 0),10);
					putText(matImage, "center_row:" + to_string(center_row) + "  center_col:" + to_string(center_col), Point(100, 100), FONT_HERSHEY_SIMPLEX,1,Scalar(255, 0, 0),3);
				}
				else putText(matImage, "No Red Point", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0),3);
				
				////u,v转化为x,y;
				//float x, y;
				//double h=20;
				//cout << "center_row" <<center_row<< "   center_col:" << center_col << endl;
				//img_loc(h,center_col,center_row,x,y);
				//cout << "世界坐标：x:" << x << "   y:" << y << endl;
				imshow("my_img", matImage);
#endif
				m_iDispFrameNum++;
			}

			//在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
			//否则再次调用CameraGetImageBuffer时，程序将被挂起，知道其他线程中调用CameraReleaseImageBuffer来释放了buffer
			CameraReleaseImageBuffer(hCamera, pbyBuffer);

			memcpy(&m_sFrInfo, &sFrameInfo, sizeof(tSdkFrameHead));
		}

		int c = waitKey(10);

		if (c == 'q' || c == 'Q' || (c & 255) == 27)
		{
			m_bExit = TRUE;
			break;
		}
	}

	if (iplImage)
	{
		cvReleaseImageHeader(&iplImage);
	}

	_endthreadex(0);
	return 0;
}
#endif


int main(int argc, char* argv[])
{
	cout << "主函数" << endl;
	namedWindow("my_img",0);
	cv::setMouseCallback("my_img", mousecallback, 0);
	
	tSdkCameraDevInfo sCameraList[10];
	INT iCameraNums;
	CameraSdkStatus status;
	tSdkCameraCapbility sCameraInfo;

	//枚举设备，获得设备列表
	iCameraNums = 10;//调用CameraEnumerateDevice前，先设置iCameraNums = 10，表示最多只读取10个设备，如果需要枚举更多的设备，请更改sCameraList数组的大小和iCameraNums的值
	if (CameraEnumerateDevice(sCameraList, &iCameraNums) != CAMERA_STATUS_SUCCESS || iCameraNums == 0)
	{
		printf("No camera was found!");
		return FALSE;
	}

	//该示例中，我们只假设连接了一个相机。因此，只初始化第一个相机。(-1,-1)表示加载上次退出前保存的参数，如果是第一次使用该相机，则加载默认参数.
	//In this demo ,we just init the first camera.
	if ((status = CameraInit(&sCameraList[0], -1, -1, &m_hCamera)) != CAMERA_STATUS_SUCCESS)
	{
		char msg[128];
		sprintf_s(msg, "Failed to init the camera! Error code is %d", status);
		printf(msg);
		printf(CameraGetErrorString(status));
		return FALSE;
	}


	//Get properties description for this camera.
	CameraGetCapability(m_hCamera, &sCameraInfo);//"获得该相机的特性描述"

	m_pFrameBuffer = (BYTE*)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax * sCameraInfo.sResolutionRange.iWidthMax * 3, 16);

	if (sCameraInfo.sIspCapacity.bMonoSensor)
	{
		CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_MONO8);
	}

	strcpy_s(g_CameraName, sCameraList[0].acFriendlyName);

	CameraCreateSettingPage(m_hCamera, NULL,
		g_CameraName, NULL, NULL, 0);//"通知SDK内部建该相机的属性页面";

#ifdef USE_CALLBACK_GRAB_IMAGE //如果要使用回调函数方式，定义USE_CALLBACK_GRAB_IMAGE这个宏
	//Set the callback for image capture
	CameraSetCallbackFunction(m_hCamera, GrabImageCallback, 0, NULL);//"设置图像抓取的回调函数";
#else
	m_hDispThread = (HANDLE)_beginthreadex(NULL, 0, &uiDisplayThread, (PVOID)m_hCamera, 0, &m_threadID);
#endif

	CameraPlay(m_hCamera);

	CameraShowSettingPage(m_hCamera, TRUE);//TRUE显示相机配置界面。FALSE则隐藏。

	while (m_bExit != TRUE)
	{
		waitKey(10);
	}

	CameraUnInit(m_hCamera);

	CameraAlignFree(m_pFrameBuffer);

	destroyWindow(g_CameraName);

#ifdef USE_CALLBACK_GRAB_IMAGE
	if (g_iplImage)
	{
		cvReleaseImageHeader(&g_iplImage);
	}
#endif
	return 0;
}

void mousecallback(int event, int x, int y, int flags, void* userdata)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
	{
		flag = true;
	}
	break;
	case EVENT_LBUTTONUP:
	{
		if (flag)
		{
			xvalue = x;
			yvalue = y;
			flag = 0;
			int b = matImage.at<Vec3b>(yvalue, xvalue)[0];
			int g = matImage.at<Vec3b>(yvalue, xvalue)[1];
			int r = matImage.at<Vec3b>(yvalue, xvalue)[2];
			cout << " row: " << yvalue << "  col: " << xvalue << " B:" << b << ' ' << " G:" << g << ' ' << " R:" << r << endl;
		}
	}
	break;
	}
}
