#include "CameraApi.h"
/*---工业相机中使用到opencv2.0的 IplImage 需要包含此头文件 ---*/
#include "opencv2/imgproc/imgproc_c.h"
/*---工业相机中使用到opencv2.0的 cvReleaseImageHeader 需要包含此头文件 ---*/
#include <opencv2/opencv.hpp>
#include <iostream>
#define DRAW_RECT
#include "solve_pnp.h"

#define DRAW_RECT
#define PUT_TEXT
#define COUT_FPS
#define SHOW_IMG

#define RED_COLOR 1
#define BLUE_COLOR 2

#define LOSE_CNT_MAX 4

using namespace std;
using namespace cv;

unsigned char           * g_pRgbBuffer;     //处理后数据缓存区

struct ArmorROI
{
    RotatedRect last_armor_rect = RotatedRect();      // 上一次检测到装甲的旋转矩形
    Point tl = Point(0,0);                         // 截图区域的左上角的点
    int width = 0;                        // 截图区域的宽度
    int height = 0;                       // 截图区域的高度
    Mat ROI_img;                      // ROI区域

    //获得参数的函数
    void getParam(RotatedRect rect,Point point,int W,int H,Mat src){
        src.copyTo(ROI_img);
        last_armor_rect = rect;
        tl = point;
        width = W;
        height = H;
    }

    //重置结构体中的参数
    void resetParam(){
        last_armor_rect = RotatedRect();      // 上一次检测到装甲的旋转矩形
        tl = Point(0,0);                         // 截图区域的左上角的点
        width = 0;                        // 截图区域的宽度
        height = 0;
    }
};

ArmorROI setImg(Mat src,bool is_Lost_target,short int cnt,RotatedRect last_roi_rect);
RotatedRect fit_Rrect(RotatedRect &rect_left,RotatedRect &rect_right,ArmorROI &roirect);
float centerDistance(Point p1, Point p2);

int main()
{
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
    CameraSetExposureTime(hCamera,500);
    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
    CameraReleaseImageBuffer(hCamera,pbyBuffer);
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

    /*--- 角度结算初始化 ---*/
    RM_SolveAngle angle_solve;
#ifdef PUT_TEXT
    Point put_point;
#endif
    Mat src_img;    //读入图像
    Mat dst_img;    //输出图像
    Mat bin_img_gray;   //灰度二值图
    Mat bin_img_hsv;    //颜色二值图
    Mat gray_img;   //灰度图
    Mat hsv_img;    //颜色图
    Mat roi_img;    //ROI区域

    //上一帧是否有数据
    bool is_last_data_catch = false;
    //储存上一帧检测到的装甲的旋转矩形
    RotatedRect last_armor = RotatedRect();

    //丢失目标时的帧计数
    short int lose_target_cnt = 0;
    bool is_Lost_target = true;
    //VideoCapture capture(0);
    while (1) {
#ifdef COUT_FPS
        double t = (double)getTickCount();
#endif
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
        //cout<<" w:"<< src_img.cols<<"   h:"<<src_img.rows <<endl;
        resize(src_img,src_img,Size(640,512),INTER_NEAREST);
//        ArmorROI roi = setImg(frame,is_last_data_catch,0,last_armor);
//        roi_img = roi.ROI_img;
        ArmorROI roi;
        if(is_last_data_catch == true && is_Lost_target == false){
            roi = setImg(src_img,is_Lost_target,lose_target_cnt,last_armor);
            roi_img = roi.ROI_img;
        }else if(is_Lost_target == false && is_last_data_catch == false && lose_target_cnt <= LOSE_CNT_MAX){
            roi = setImg(src_img,is_Lost_target,lose_target_cnt,last_armor);
            roi_img = roi.ROI_img;
            lose_target_cnt += 1;
        }else {
            lose_target_cnt = 0;
            is_Lost_target = true;
            roi = setImg(src_img,is_Lost_target,lose_target_cnt,last_armor);
            roi_img = roi.ROI_img;
        }
        is_last_data_catch = false;

        src_img.copyTo(dst_img);
        cvtColor(roi_img,gray_img,COLOR_BGR2GRAY);
        cvtColor(roi_img,hsv_img,COLOR_BGR2HSV);
//        cvtColor(roi_img,hsv_img,COLOR_BGR2Lab);
        threshold(gray_img,bin_img_gray,20,150,THRESH_BINARY); //20 140
        Mat element1 = getStructuringElement(MORPH_RECT,Size(1,3));
        Mat element2 = getStructuringElement(MORPH_RECT,Size(1,3));
//        dilate(bin_img_gray,bin_img_gray,element1);


        int color = 1;
        if (color == RED_COLOR) { //己方颜色
            Mat mix_img_1;
            Mat mix_img_2;
            inRange(hsv_img, Scalar(165, 160, 100 ), Scalar(185, 255, 255 ), mix_img_1); //red
            inRange(hsv_img, Scalar(0  , 200, 80 ), Scalar(15  , 255, 255 ), mix_img_2);//解除地面反光将V值下限提高
            bin_img_hsv = mix_img_1 + mix_img_2;
//            inRange(hsv_img, Scalar( 40, 150, 130), Scalar(170, 220, 190), bin_img_hsv); //red LAB值
               // 20 150 130       170 220 190
        } else if (color == BLUE_COLOR) {
            //inRange(hsv_img, Scalar(80, 210, 130), Scalar(124, 255, 255), bin_img_1); //blue
            inRange(hsv_img, Scalar(60, 20, 120), Scalar(140, 250, 255), bin_img_hsv); //blue    rm_use
        }                           //80 160  125 255 255
//        dilate(bin_img_hsv,bin_img_hsv,element2);
//        medianBlur(bin_img_hsv,bin_img_hsv,3);

        vector <vector < RotatedRect> > candidate_armor;
        vector <RotatedRect> candidate_light;

        vector < vector < Point > > contours_gray;
        vector < Vec4i > hierarchy_gray;
        findContours(bin_img_gray, contours_gray, hierarchy_gray, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
        vector < vector < Point > > contours_hsv;
        vector < Vec4i > hierarchy_hsv;
        findContours(bin_img_hsv, contours_hsv, hierarchy_hsv, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
        for (size_t i = 0; i < contours_gray.size();++i){
            for (size_t j = 0; j < contours_hsv.size();++j){
                if (contours_hsv[j].size() < 6){continue;}
                RotatedRect R_rect = fitEllipse(contours_hsv[j]);
                if(pointPolygonTest(contours_gray[i],R_rect.center,true) >= 0){
                    float width = MIN(R_rect.size.width,R_rect.size.height);
                    float height =MAX(R_rect.size.width,R_rect.size.height);
                    //float rect_area = width * height;
                    float w_h_ratio = width / height;
                    if ((w_h_ratio < 0.50) /*高宽比,角度筛选形状符合要求的轮廓*/
                            && ((0<= R_rect.angle && R_rect.angle<=45)||(135<=R_rect.angle && R_rect.angle<=180))){
                        candidate_light.push_back(R_rect);
#ifdef DRAW_RECT
                        Point2f vtx[4];
                        R_rect.points(vtx);
                        for (int j = 0; j < 4; j++){line(dst_img, vtx[j] + (Point2f)roi.tl, vtx[(j + 1) % 4] + (Point2f)roi.tl, Scalar(200, 0, 0),2,8,0);}
#endif
#ifndef PUT_TEXT
                        putText(dst_img, to_string(R_rect.angle),R_rect.center,FONT_HERSHEY_PLAIN,1,Scalar(255, 255, 255),1,8,false);
#endif
                    }
                }
            }
        }
        float light_addH_max = 0;
        for (size_t i = 0;i<candidate_light.size();++i){
            for (size_t j = i+1;j<candidate_light.size();++j){
                float angle_left;
                float angle_right;
                float w1,h1,w2,h2;

                //区别出左右灯条
                if(candidate_light[i].center.x < candidate_light[j].center.x){
                    //灯条的角度
                    angle_left = candidate_light[i].angle;
                    angle_right = candidate_light[j].angle;
                    //灯条的宽高
                    w1 = MIN(candidate_light[i].size.height,candidate_light[i].size.width);
                    h1 = MAX(candidate_light[i].size.height,candidate_light[i].size.width);
                    w2 = MIN(candidate_light[j].size.height,candidate_light[j].size.width);
                    h2 = MAX(candidate_light[j].size.height,candidate_light[j].size.width);
                } else {
                    //灯条的角度
                    angle_right = candidate_light[i].angle;
                    angle_left = candidate_light[j].angle;
                    //灯条的宽高
                    w1 = MIN(candidate_light[j].size.height,candidate_light[j].size.width);
                    h1 = MAX(candidate_light[j].size.height,candidate_light[j].size.width);
                    w2 = MIN(candidate_light[i].size.height,candidate_light[i].size.width);
                    h2 = MAX(candidate_light[i].size.height,candidate_light[i].size.width);
                }
                //灯条高度差
                float light_y_diff = fabs(candidate_light[i].center.y - candidate_light[j].center.y);
                bool is_height_diff_catch = (light_y_diff < (h1+h2)*0.5);

                //灯条间的距离
                float light_distance = centerDistance(candidate_light[i].center,candidate_light[j].center);
                bool light_distance_catch = (light_distance < MAX(h1,h2) * 5);
                //float dis_height_ratio = light_distance / MAX(h1,h2);

//                cout<<"angle_left=="<<angle_left<<endl<<"angle_right=="<<angle_right<<endl<<endl<<endl;

                //灯条之间角度的关系
                bool is_light_angle_catch;
                if((angle_left <90 && angle_right < 90) || (angle_left > 90 && angle_right > 90)){
                    //同侧
                    is_light_angle_catch = (fabs(angle_left-angle_right) <= 10);
                } else if(angle_left < 90 && angle_right > 90){
                    //--- \ / --- 内八
                    is_light_angle_catch = (170 <= fabs(angle_left-angle_right) && fabs(angle_left-angle_right) <= 180);
                } else if(angle_left > 90 && angle_right < 90){
                    //--- / \ --- 外八
                    is_light_angle_catch = (170 <= fabs(angle_left-angle_right) && fabs(angle_left-angle_right) <= 180);
                } else if(angle_left ==0 && angle_right != 0){
                    //左边竖直
                    is_light_angle_catch = (170 <= fabs(angle_left-angle_right) || fabs(angle_left-angle_right) <= 10);
                } else if(angle_left !=0 && angle_right == 0){
                    //右边竖直
                    is_light_angle_catch = (170 <= fabs(angle_left-angle_right) || fabs(angle_left-angle_right) <= 10);
                }

#ifdef PUT_TEXT
//                //--- 打印灯条距离与最大高的比
//                putText(dst_img,to_string(dis_height_ratio),(candidate_light[i].center+candidate_light[j].center)*0.5,FONT_HERSHEY_PLAIN,2,Scalar(255, 255, 255),1,8,false);
#endif
                light_distance_catch = true;


                bool is_little_armor;
                if( light_distance/(h1+h2) > 0.60 && light_distance/(h1+h2) < 4.30 && light_distance <= 600.0)  //小 280   大600
                {
                    is_little_armor = 1 ;
//                    cout<< "light_distance/(h1+h2)=="<<light_distance<<endl;
                }
//                else if( light_distance/(h1+h2) > 0.60 && light_distance/(h1+h2) < 4.30 && light_distance <= 25.0 && angle_solve.dist >=5000.0)
//                {
//                    is_little_armor = 1 ;
////                    cout<< "light_distance/(h1+h2)=="<<light_distance/(h1+h2)<<endl;
//                }
                else
                {
                    is_little_armor = 0 ;
//                    cout<< "light_distance/(h1+h2)=="<<light_distance/(h1+h2)<<endl;
                }


                if(is_height_diff_catch
                        && is_light_angle_catch
                        && light_distance_catch
                        && is_little_armor
//                        &&(light_distance/(h1+h2) >= 0.60 && light_distance/(h1+h2) <=4.30)
                        ){

//                    cout << "light_distance==" << light_distance << endl ;
//                    cout << "h1+h2==" << h1+h2 << endl << endl ;
//                    if((y-(h1+h2)) <= 10.0){cout<<"小小小小小小"<<endl;}
                    double y = 0.9089 * light_distance  - 2.8811;
                    if ((y-(h1+h2) <=15) )  {cout << "小小小小小小" << endl ;}
                    else if((y-(h1+h2)) >= 30.0){cout<<"大大大大大大"<<endl;}  //镜头离小装甲最近距离时得
                    else if( (y-(h1+h2) > 15.0) && (y-(h1+h2) < 30.0) &&light_distance > 100)   //小装甲 > 110
                    {cout << "小小小小小小" << endl ;}
                    else
                    {cout << "大大大大大大" << endl ;}

                    cout << light_distance << endl ;
                    vector < RotatedRect> light_pair(2);
                    light_pair[0] = candidate_light[i];
                    light_pair[1] = candidate_light[j];
                    candidate_armor.push_back(light_pair);
                    is_last_data_catch = true;//检测到装甲板，则下一帧会标识上一帧有数据

                    if(h1 + h2 >= light_addH_max){
                        light_addH_max = h1 + h2;

//                        cout << "light_distance==" << angle_solve.dist /light_distance << endl;
//                        cout << "light_addH_max==" << light_addH_max << endl << endl ;
                    } //获取所有配对装甲中灯条加起来最大值

#ifdef PUT_TEXT
                    put_point = (Point)(candidate_light[i].center+candidate_light[j].center)*0.5 + roi.tl;
//                    putText(dst_img,to_string(fabs(angle_left-angle_right)),put_point,FONT_HERSHEY_PLAIN,2,Scalar(255, 255, 255),1,8,false);
                    putText(dst_img,to_string((y-(h1+h2))),put_point,FONT_HERSHEY_PLAIN,5,Scalar(255, 255, 255),1,8,false);

                    /*--- 打印两灯条角度差 --- */
#endif
                }
            }
        }//循环结束

        //筛选最优目标
        for(size_t i = 0;i<candidate_armor.size();++i){
            if(candidate_armor[i][0].size.height + candidate_armor[i][1].size.height >= light_addH_max  ){
                RotatedRect armor_rect = fit_Rrect(candidate_armor[i][0],candidate_armor[i][1],roi);
                last_armor = armor_rect;
                is_Lost_target = false;
                lose_target_cnt = 0;
#ifdef DRAW_RECT
                Point2f vtx[4];
                armor_rect.points(vtx);

                Point2f real_center = armor_rect.center + (Point2f)roi.tl;
                Point2f real_vtx[4];
                for (int j = 0; j < 4; j++){
                    line(dst_img, vtx[j], vtx[(j + 1) % 4], Scalar(150, 200, 0),2,8,0);
                    real_vtx[j] = vtx[j] + (Point2f)roi.tl;
                }
                int armor_size = 1;
                angle_solve.run_SolvePnp(src_img,armor_rect,armor_size);
                circle(src_img,armor_rect.center,2,Scalar(255,255,255),2,8,0);

#endif
#ifdef PUT_TEXT
                put_point = Point(armor_rect.center.x,armor_rect.center.y+20);
                /*--- 打印装甲的旋转矩形旋转角 --- */
                //putText(dst_img, to_string(armor_rect.angle),armor_rect.center,FONT_HERSHEY_PLAIN,2,Scalar(255, 255, 255),1,8,false);
#endif
            }
        }//循环结束
#ifdef PUT_TEXT
        putText(dst_img, to_string(light_addH_max),Point(10,10),FONT_HERSHEY_PLAIN,1,Scalar(255, 255, 255),1,8,false);
        putText(dst_img, to_string(angle_solve.dist),Point(40,50),FONT_HERSHEY_PLAIN,2,Scalar(255, 255, 255),2,8,false);
        /*--- 打印装甲的灯条长度之和 --- */
#endif
#ifdef SHOW_IMG
        imshow("bin_img_gray",bin_img_gray);
        imshow("bin_img_hsv",bin_img_hsv);
//        imshow("src_img",src_img);
        imshow("dst_img",dst_img);
//        imshow("roi_img",roi_img);

#endif
#ifdef COUT_FPS
        t = ((double)getTickCount() - t) / getTickFrequency();
        int fps = int(1.0 / t);
//        cout << "FPS: " << fps<<endl;
#endif
        CameraReleaseImageBuffer(hCamera,pbyBuffer);
        int key = waitKey(1);
        if(char(key) == 27)break;
    }

    //CameraReleaseImageBuffer(hCamera,pbyBuffer);
    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);
    return 0;
}

ArmorROI setImg(Mat src,bool is_Lost_target,short int cnt,RotatedRect last_roi_rect){
    ArmorROI Rrect;
    Mat set_img;
    Point tl;
    int W;
    int H;
    if(is_Lost_target == false){
        if(cnt <= LOSE_CNT_MAX*0.5){
            W = last_roi_rect.boundingRect().width * 2;
            H = last_roi_rect.boundingRect().height * 2.5;
        } else if (LOSE_CNT_MAX*0.5 < cnt &&cnt <= LOSE_CNT_MAX){
            W = last_roi_rect.boundingRect().width * 3;
            H = last_roi_rect.boundingRect().height * 3.5;
        } else{
            W = last_roi_rect.boundingRect().width;
            H = last_roi_rect.boundingRect().height;
        }

        /*--- ROI区域安全处理 ---*/
        tl = Point(last_roi_rect.center.x - W*0.5,last_roi_rect.center.y - H*0.5);
        if (tl.x < 0){
            tl.x = 0;//如果左上角点x超出尺寸，x =  0
        }
        if (tl.y < 0){
            tl.y = 0;//如果左上角点y超出尺寸，y = 0
        }
        if (last_roi_rect.center.x - W*0.5 < 0){
            W = W + (last_roi_rect.center.x - W*0.5 );
            // 如果宽度超出了左边界，设置为到边界的值
        }
        if (tl.x + W > src.cols){
            W = W - (tl.x + W - src.cols);
            // 如果宽度超出了右边界，设置为到边界的值
        }
        if (last_roi_rect.center.y - H*0.5 < 0){
            H = H + (last_roi_rect.center.y - H*0.5 );
            // 如果高度超出了上边界，设置为到边界的值
        }
        if (tl.y + H > src.rows){
            H = H - (tl.y + H - src.rows );
            // 如果高度超出了下边界，设置为到边界的值
        }
        /*--- ROI区域安全处理 ---*/

        Rect roi = Rect(tl.x,tl.y,W,H);
        src(roi).copyTo(set_img);
        Rrect.getParam(last_roi_rect,tl,W,H,set_img);
    }else{
        src.copyTo(set_img);
        Rrect.getParam(last_roi_rect,tl,0,0,set_img);
        Rrect.resetParam();
    }
    return Rrect;
}

float centerDistance(Point p1, Point p2) {
    float D = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    return D;
}

RotatedRect fit_Rrect(RotatedRect &rect_left,RotatedRect &rect_right,ArmorROI &roirect){
    Point2f center = (rect_left.center+rect_right.center)*0.5;
    center.x = center.x + roirect.tl.x;
    center.y = center.y + roirect.tl.y;
    float center_slope = (rect_left.center.y-rect_right.center.y)/(rect_left.center.x-rect_right.center.x);
    float distance = centerDistance(rect_left.center,rect_right.center);
    float width_left =  MIN(rect_left.size.width,rect_left.size.height);
    float height_left = MAX(rect_left.size.width,rect_left.size.height);
    float width_right = MIN(rect_right.size.width,rect_right.size.height);
    float height_right =MAX(rect_right.size.width,rect_right.size.height);

    float W = distance-(width_left + width_right);
    float H = MAX(height_left,height_right);//*2.27;
    float angle = atan(center_slope);
    RotatedRect Rrect = RotatedRect(center,Size2f(W,H),angle*180/CV_PI);
    return Rrect;
}
