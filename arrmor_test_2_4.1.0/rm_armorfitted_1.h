#ifndef RM_ARMORFITTED_1_H
#define RM_ARMORFITTED_1_H

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include <fstream>

using namespace std;
using namespace cv;

int  min_light_height= 10;// 板灯最小高度值
int min_light_delta_w=12;// 左右灯柱在水平位置上的最大差值，像素单位
int max_light_delta_w=450;// 左右灯柱在水平位置上的最小差值，像素单位
int max_light_delta_v=50;// 左右灯柱在垂直位置上的最大差值，像素单位
float max_lr_rate=2;// 左右灯柱的比例值 1.5

// 匹配灯条的结构体
struct matched_rect
{
    cv::RotatedRect rect;
    float lr_rate;
    float angle_abs;

};

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))


#endif // RM_ARMORFITTED_H
