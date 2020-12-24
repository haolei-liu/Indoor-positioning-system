// OpenCV.cpp : �����̨Ӧ�ó������ڵ㡣
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

UINT            m_threadID;		//ͼ��ץȡ�̵߳�ID
HANDLE          m_hDispThread;	//ͼ��ץȡ�̵߳ľ��
BOOL            m_bExit = FALSE;		//����֪ͨͼ��ץȡ�߳̽���
CameraHandle    m_hCamera;		//��������������ͬʱʹ��ʱ���������������	
BYTE* m_pFrameBuffer; //���ڽ�ԭʼͼ������ת��ΪRGB�Ļ�����
tSdkFrameHead   m_sFrInfo;		//���ڱ��浱ǰͼ��֡��֡ͷ��Ϣ

int	            m_iDispFrameNum;	//���ڼ�¼��ǰ�Ѿ���ʾ��ͼ��֡������
float           m_fDispFps;			//��ʾ֡��
float           m_fCapFps;			//����֡��
tSdkFrameStatistic  m_sFrameCount;
tSdkFrameStatistic  m_sFrameLast;
int					m_iTimeLast;
char		    g_CameraName[64];

//���ץȡ
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

	//�Ȱѻ����u��v�����ʵu,v
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
�����Ҫʹ�ûص������ķ�ʽ���ͼ�����ݣ���ע�ͺ궨��USE_CALLBACK_GRAB_IMAGE.
���ǵ�SDKͬʱ֧�ֻص��������������ýӿ�ץȡͼ��ķ�ʽ�����ַ�ʽ��������"�㿽��"���ƣ������ĳ̶ȵĽ���ϵͳ���ɣ���߳���ִ��Ч�ʡ�
��������ץȡ��ʽ�Ȼص������ķ�ʽ�������������ó�ʱ�ȴ�ʱ��ȣ����ǽ�����ʹ�� uiDisplayThread �еķ�ʽ
*/
//#define USE_CALLBACK_GRAB_IMAGE 
#ifdef USE_CALLBACK_GRAB_IMAGE
/*ͼ��ץȡ�ص�����*/
IplImage* g_iplImage = NULL;

void _stdcall GrabImageCallback(CameraHandle hCamera, BYTE* pFrameBuffer, tSdkFrameHead* pFrameHead, PVOID pContext)
{

	CameraSdkStatus status;


	//����õ�ԭʼ����ת����RGB��ʽ�����ݣ�ͬʱ����ISPģ�飬��ͼ����н��룬������������ɫУ���ȴ���
	//�ҹ�˾�󲿷��ͺŵ������ԭʼ���ݶ���Bayer��ʽ��
	status = CameraImageProcess(hCamera, pFrameBuffer, m_pFrameBuffer, pFrameHead);

	//�ֱ��ʸı��ˣ���ˢ�±���
	if (m_sFrInfo.iWidth != pFrameHead->iWidth || m_sFrInfo.iHeight != pFrameHead->iHeight)
	{
		m_sFrInfo.iWidth = pFrameHead->iWidth;
		m_sFrInfo.iHeight = pFrameHead->iHeight;
	}

	if (status == CAMERA_STATUS_SUCCESS)
	{
		//����SDK��װ�õ���ʾ�ӿ�����ʾͼ��,��Ҳ���Խ�m_pFrameBuffer�е�RGB����ͨ��������ʽ��ʾ������directX,OpengGL,�ȷ�ʽ��
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
/*ͼ��ץȡ�̣߳���������SDK�ӿں�����ȡͼ��*/
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
			//����õ�ԭʼ����ת����RGB��ʽ�����ݣ�ͬʱ����ISPģ�飬��ͼ����н��룬������������ɫУ���ȴ���
			//�ҹ�˾�󲿷��ͺŵ������ԭʼ���ݶ���Bayer��ʽ��
			status = CameraImageProcess(hCamera, pbyBuffer, m_pFrameBuffer, &sFrameInfo);//����ģʽ

			//�ֱ��ʸı��ˣ���ˢ�±���
			if (m_sFrInfo.iWidth != sFrameInfo.iWidth || m_sFrInfo.iHeight != sFrameInfo.iHeight)
			{
				m_sFrInfo.iWidth = sFrameInfo.iWidth;
				m_sFrInfo.iHeight = sFrameInfo.iHeight;
				//ͼ���С�ı䣬֪ͨ�ػ�
			}

			if (status == CAMERA_STATUS_SUCCESS)
			{
				//����SDK��װ�õ���ʾ�ӿ�����ʾͼ��,��Ҳ���Խ�m_pFrameBuffer�е�RGB����ͨ��������ʽ��ʾ������directX,OpengGL,�ȷ�ʽ��
				CameraImageOverlay(hCamera, m_pFrameBuffer, &sFrameInfo);

				// ����SDK���������Ĭ���Ǵӵ׵����ģ�ת��ΪOpencvͼƬ��Ҫ��һ�´�ֱ����
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
				//ʶ���ɫС��
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
				
				////u,vת��Ϊx,y;
				//float x, y;
				//double h=20;
				//cout << "center_row" <<center_row<< "   center_col:" << center_col << endl;
				//img_loc(h,center_col,center_row,x,y);
				//cout << "�������꣺x:" << x << "   y:" << y << endl;
				imshow("my_img", matImage);
#endif
				m_iDispFrameNum++;
			}

			//�ڳɹ�����CameraGetImageBuffer�󣬱������CameraReleaseImageBuffer���ͷŻ�õ�buffer��
			//�����ٴε���CameraGetImageBufferʱ�����򽫱�����֪�������߳��е���CameraReleaseImageBuffer���ͷ���buffer
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
	cout << "������" << endl;
	namedWindow("my_img",0);
	cv::setMouseCallback("my_img", mousecallback, 0);
	
	tSdkCameraDevInfo sCameraList[10];
	INT iCameraNums;
	CameraSdkStatus status;
	tSdkCameraCapbility sCameraInfo;

	//ö���豸������豸�б�
	iCameraNums = 10;//����CameraEnumerateDeviceǰ��������iCameraNums = 10����ʾ���ֻ��ȡ10���豸�������Ҫö�ٸ�����豸�������sCameraList����Ĵ�С��iCameraNums��ֵ
	if (CameraEnumerateDevice(sCameraList, &iCameraNums) != CAMERA_STATUS_SUCCESS || iCameraNums == 0)
	{
		printf("No camera was found!");
		return FALSE;
	}

	//��ʾ���У�����ֻ����������һ���������ˣ�ֻ��ʼ����һ�������(-1,-1)��ʾ�����ϴ��˳�ǰ����Ĳ���������ǵ�һ��ʹ�ø�����������Ĭ�ϲ���.
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
	CameraGetCapability(m_hCamera, &sCameraInfo);//"��ø��������������"

	m_pFrameBuffer = (BYTE*)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax * sCameraInfo.sResolutionRange.iWidthMax * 3, 16);

	if (sCameraInfo.sIspCapacity.bMonoSensor)
	{
		CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_MONO8);
	}

	strcpy_s(g_CameraName, sCameraList[0].acFriendlyName);

	CameraCreateSettingPage(m_hCamera, NULL,
		g_CameraName, NULL, NULL, 0);//"֪ͨSDK�ڲ��������������ҳ��";

#ifdef USE_CALLBACK_GRAB_IMAGE //���Ҫʹ�ûص�������ʽ������USE_CALLBACK_GRAB_IMAGE�����
	//Set the callback for image capture
	CameraSetCallbackFunction(m_hCamera, GrabImageCallback, 0, NULL);//"����ͼ��ץȡ�Ļص�����";
#else
	m_hDispThread = (HANDLE)_beginthreadex(NULL, 0, &uiDisplayThread, (PVOID)m_hCamera, 0, &m_threadID);
#endif

	CameraPlay(m_hCamera);

	CameraShowSettingPage(m_hCamera, TRUE);//TRUE��ʾ������ý��档FALSE�����ء�

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
