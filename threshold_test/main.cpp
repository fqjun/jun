#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include "CameraApi.h"
//---工业相机中使用到opencv2.0的 IplImage 需要包含此头文件 ---*/
//#include "opencv2/imgproc/imgproc_c.h"
///*---工业相机中使用到opencv2.0的 cvReleaseImageHeader 需要包含此头文件 ---

#define FPS
#define Little_Black
#define camera
/*
算法暂且发现受以下三方面影响较大：曝光，积分区域像素的个数（s），依赖于其是否比其前s个像素的平均值的百分之t的暗（t）
s取图像宽度的8分之一，t取15是暂时位置比较稳的，曝光数据要看摄像头；
*/

using namespace std;
using namespace cv;
unsigned char  * g_pRgbBuffer;     //处理后数据缓存区

#ifdef FPS
double t1, t2, fps;
double t_max=0.00;
int t=0;
#endif

void adaptiveThreshold( const cv::Mat& inImg, cv::Mat& outImg, int step)
{
    if (outImg.empty())
    {
        outImg.create(inImg.size(), inImg.depth());
    }
    int S = inImg.cols/8 ;//可修改
    int T = 15;//可修改
    int count=0;
//    int index;
    int row, col, x1, y1, x2, y2;
    long double brs, bls, trs, tls, value;
    int s2 = S/2;
    Mat sum(inImg.rows+1, inImg.cols+1, CV_32FC1);
    integral(inImg, sum);//创建积分图像
    for (row = 0; row < inImg.rows; row++)
    {
        const uchar* ptr = inImg.ptr<uchar>(row);
        uchar* ptr_out = outImg.ptr<uchar>(row);
        for (col = 0; col < inImg.cols; col++)
        {
            // set the SxS region
            x1 = col-s2; x2 = col+s2;
            y1 = row-s2; y2 = row+s2;
            // check the border
            if (x1 < 0) x1 = 0;
            if (x2 >= inImg.cols) x2 = inImg.cols-1;
            if (y1 < 0) y1 = 0;
            if (y2 >= inImg.rows) y2 = inImg.rows-1;
            count = (x2-x1)*(y2-y1);
            brs = sum.at<int>(y2, x2);
            bls = sum.at<int>(y2, x1);
            trs = sum.at<int>(y1, x2);
            tls = sum.at<int>(y1, x1);
            value = brs+tls-trs-bls;
            if ((ptr[col]*count) < (long)(value*(100-T)/100))
            {
                ptr_out[col] = 255;
            }
            else
            {
                ptr_out[col] = 0;
            }
        }
    }
}

int main()
{
#ifdef camera
    /*--- 相机初始化 ---*/
    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    IplImage *iplImage = NULL;
    int                     channel=3;
    BOOL                    AEstate=FALSE;

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    //初始化失败
    printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    cout<<CameraGetAeState(hCamera,&AEstate);
    cout<<CameraSetAeState(hCamera,FALSE);
    CameraSetExposureTime(hCamera,500);//曝光时间的修改以改动相机图像的亮暗  暗1650左右
    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
//    CameraReleaseImageBuffer(hCamera,pbyBuffer);
    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */

    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

    /*--- 相机初始化 ---*/
#endif

   Mat src_img,bin_img,gray_img,copy_img;

#ifdef Little_Black
   VideoCapture capture(1);
#endif

   while(true)
   {

#ifdef camera
       //读取原图
       if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
       {
           CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
           if (iplImage)
           {
               cvReleaseImageHeader(&iplImage);
           }
           iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
           cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
           src_img = cvarrToMat(iplImage,true);//这里只是进行指针转换，将IplImage转换成Mat类型
       }

     CameraReleaseImageBuffer(hCamera,pbyBuffer);
#endif

#ifdef FPS
     t1 = static_cast<double>(getTickCount());
#endif

#ifdef Little_Black
       capture>>src_img;
#endif
       if (!src_img.data)
       {
           cout<<"quit"<<endl;
       }

       imshow("src_img",src_img);

       copy_img=src_img.clone();

       cvtColor(copy_img,gray_img,CV_RGB2GRAY);
       imshow("gray_img",gray_img);

//       adaptiveThreshold(gray_img,bin_img,src_img.cols,src_img.rows);
       adaptiveThreshold(gray_img,bin_img,3);
//       Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
//       morphologyEx(bin_img,bin_img,MORPH_GRADIENT,element);
//       morphologyEx(bin_img,bin_img,MORPH_GRADIENT,element);
       imshow("bin",bin_img);

       CameraReleaseImageBuffer(hCamera,pbyBuffer);
#ifdef FPS
       t2 = ((double)getTickCount() - t1) / getTickFrequency();//计时结束
       fps = 1.0 / t2;//计算帧率
       if(t>10){
       t_max= t_max > t2 ? t_max:t2 ;
       }
       t+=1;
       cout << "运行时间：" << t_max << endl << "帧率：" << fps <<endl;
#endif
       int a = waitKey(1);
       if ((char)a == 27)
           break;

   }

   //CameraReleaseImageBuffer(hCamera,pbyBuffer);
   CameraUnInit(hCamera);
   //注意，现反初始化后再free
   free(g_pRgbBuffer);

    return 0;
}



