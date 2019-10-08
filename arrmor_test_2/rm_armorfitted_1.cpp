#include "rm_armorfitted_1.h"

#define DEBUG
#define DEBUG_1
#define BLUE
//#define RED

#define CAMERA
//#define VEDIO

//#define LR_rate
//#define Len

RotatedRect boundingRRect( cv::RotatedRect  left,  cv::RotatedRect  right);//装甲板拟合矩形



int main()
{
    Mat src_img;

#ifdef WRITE
    ofstream write;
    write.open("text.txt");//输出数据
#endif

#ifdef VEDIO
    VideoCapture capture("/home/fqjun/WorkSpace/录像/armor_4.avi");//armor_2  大小装甲-红  步兵自旋-蓝  基地步兵-蓝
#endif

#ifdef CAMERA
    VideoCapture capture(1);
#endif

    Mat copy_img,lab_img,/*dst_img,*/mask;

//    vector<matched_rect>match_rects;

    int a=0;
    float box=0;
    float average=0;
    float lr_rate_max=0;

    //delta between left_angle and center_angle
    float delta_lc_angle=0;
    //delta between right_angle and center_angle
    float delta_rc_angle=0;




    while(true)
    {
//       a++;
//       cout<<a<<endl;
       capture>>src_img;
       imshow("src_img",src_img);

        if (!src_img.data)
        {
            cout<<"quit"<<endl;

        }

        copy_img=src_img.clone();
//        imshow("src_img",src_img);

    //----------------------------------预处理-----------------------------------------
        cvtColor(copy_img,lab_img,CV_RGB2Lab);


#ifdef BLUE//26                       //120
        inRange(lab_img,Scalar (200,100,150),Scalar(255,150,222),mask);//蓝 (a下限提高，白减少) (100,80,110) (200,190,210)    (170,100,180),Scalar(255,150,225)    (200,100,180),Scalar(255,150,230)   (200,100,150),Scalar(255,150,222)
#endif

#ifdef RED
        inRange(lab_img,Scalar(0,141,20),Scalar(130,205,101),mask);//红 (0,141,20) (130,205,101)
#endif

//        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
//        GaussianBlur(mask,mask,Size(3,3),0,0);
//        erode(mask,mask,element);
//        dilate(mask,mask,element);//膨胀
//        GaussianBlur(mask,mask,Size(3,3),0,0);
//        dilate(mask,mask,element);//膨胀


//        morphologyEx(mask,mask,MORPH_CLOSE,element);//闭运算
        imshow("mask",mask);



        vector < vector < Point > > contours;
//        vector < vector < Point2i > > contours;
//        vector < Rect > boundRect;//最小外接矩形
        vector < RotatedRect > rotateRect;//最小外接旋转
        vector < vector < RotatedRect > > armor_light_pair;//储存一对灯条
    //--------------------------------查找灯条-------------------------------------------------
        findContours(mask, contours,  RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());
        for (int i = 0; i < (int) contours.size(); ++i)
        {

            if (contours.size() <= 1)
            {
                cout<<"boom"<<endl;
                break;
            }

//            Rect B_rect_i = boundingRect(contours[i]);//最小外接矩形
//            if(contours[i].size()<6){continue;}
            RotatedRect R_rect_i= minAreaRect(contours[i]);//最小外接旋转矩形

            double max_rrect_len = MAX(R_rect_i.size.width, R_rect_i.size.height);
            double min_rrect_len = MIN(R_rect_i.size.width, R_rect_i.size.height);

//            bool if1 = ((fabs(R_rect_i.angle) >= 135.0 )&&(fabs(R_rect_i.angle) <= 180.0 ) && max_rrect_len > min_rrect_len); // 往左倾斜限制 待改进
//            bool if2 = ((fabs(R_rect_i.angle) >= 0.0 )&&(fabs(R_rect_i.angle) <= 45.0 ) && max_rrect_len > min_rrect_len);// 往右倾斜限制 待改进
//            bool if4 = ((min_rrect_len / max_rrect_len <0.50)&&(min_rrect_len / max_rrect_len>0.10))  ; // 灯条的长宽比

            bool if1 = (fabs(R_rect_i.angle) <=45 && R_rect_i.size.height>R_rect_i.size.width);// 往左倾斜限制 待改进
            bool if2 = (fabs(R_rect_i.angle) >45 && R_rect_i.size.width>R_rect_i.size.height);// 往右倾斜限制 待改进
            bool if3 = max_rrect_len > min_light_height; // 灯条的最小高度
            bool if4 = (max_rrect_len/min_rrect_len >= 2.1)  ; // 灯条的长宽比

//            if(min_rrect_len!=0 )
//            cout<<"比值："<<max_rrect_len / min_rrect_len<<endl;

            bool if5=(max_rrect_len / min_rrect_len < 15);

            if ((if1 || if2) && if3 && if4&&if5)//判断是否为灯条的形状
            {
//                 cout<<R_rect_i.angle<<endl;
//                boundRect.push_back(B_rect_i);
                rotateRect.push_back(R_rect_i);

#ifdef DEBUG
                Point2f light_point[4];

                R_rect_i.points(light_point);
                for(size_t i=0;i<4;i++)
                {
                   line(copy_img,light_point[i],light_point[(i+1)%4],Scalar(0,255,255),1,4,0);//黄色
                }
#endif
            }
        }

#ifdef DEBUG_1
    //--------------------------------------------双循环两两匹配灯条---------------------------------------

        Point2f _pt[4],lu_i,ru_i,lu_j,ru_j;

        for(size_t i=0; i<rotateRect.size(); ++i)
        {

            RotatedRect rect_i = rotateRect[i];

            Point center_i = rect_i.center;
            float xi =center_i.x;
            float yi =center_i.y;
            float leni = MAX(rect_i.size.width, rect_i.size.height);
            float width_i=MIN(rect_i.size.width,rect_i.size.height);
//            float anglei = fabs(rect_i.angle);
            float anglei;
            float area_i=rect_i.size.area();

            rect_i.points(_pt);
            if(anglei>45.0)
            {
                lu_i = _pt[2];
                ru_i = _pt[3];
            }
            else
            {
                lu_i = _pt[1];
                ru_i = _pt[2];
            }

            anglei=(atan((lu_i.y-ru_i.y)/(lu_i.x-ru_i.x))) * 180 / CV_PI;

            for (size_t j = i + 1; j < rotateRect.size(); j++)
            {

                RotatedRect rect_j = rotateRect[j];
                Point center_j = rect_j.center;
                float xj = center_j.x;
                float yj = center_j.y;
                float lenj = MAX(rect_j.size.width, rect_j.size.height);
                float width_j=MIN(rect_j.size.width,rect_j.size.height);

                float area_j=rect_j.size.area();
//                float anglej=  fabs(rect_j.angle);
                float anglej;
                rect_j.points(_pt);
                if(anglej > 45.0)
                {
                    lu_j = _pt[2];
                    ru_j = _pt[3];
                }
                else
                {
                    lu_j = _pt[1];
                    ru_j = _pt[2];
                }


                anglej=(atan((lu_j.y-ru_j.y)/(lu_j.x-ru_j.x))) * 180 / CV_PI;

                if(anglei<-45)
                {
                    anglei=anglei+90;
                }

                if(anglej<-45)
                {
                    anglej=anglej+90;
                }

                float delta_w = fabs( xj - xi );
                float delta_h = fabs( yj - yi );
//                float len_max =MAX(lenj,leni);
                float lr_rate = leni > lenj ? leni / lenj : lenj / leni;
                float area_max=MAX(area_i,area_j);
                float area_min=MIN(area_i,area_j);
                float area_ratio=area_max/area_min;

                float angle_left,angle_right;
                float width_left,width_right;
                float height_left,height_right;
                float light_distance = POINT_DIST(center_i,center_j);
                float delta_center_y=fabs(rect_j.center.y-rect_i.center.y);

                //左右灯条斜率
                if(rect_i.center.x>rect_j.center.x)
                {
                    angle_left=anglej;
                    angle_right=anglei;
                    width_left=width_j;
                    width_right=width_i;
                    height_left=lenj;
                    height_right=leni;
                }
                else
                {
                    angle_left=anglei;
                    angle_right=anglej;
                    width_left=width_i;
                    width_right=width_j;
                    height_left=leni;
                    height_right=lenj;
                }


//                cout<<"left:"<<angle_left<<"   "<<"right:"<<angle_right<<endl;
#ifdef LR_rate
                if(lr_rate<2&&leni>0&&lenj>0)
                {
                    a++;
                    lr_rate_max=MAX(lr_rate_max,lr_rate);
                    box+=lr_rate;
                    average=box/a;

                }

                cout<<"左棱长比值："<<lr_rate<<endl;
                cout<<"最大值："<<lr_rate_max<<endl;
                write<<average<<endl;

#endif
#ifdef Len
           if(1)
           {
               a++;
               cout<<"leni:"<<leni<<"   "<<"lenj:"<<lenj<<endl;
               cout<<"abs:"<<abs(yi - yj) <<endl;
               box+=abs(yi - yj);
               average=box/a;
               write<<average<<endl;
           }
#endif

                bool condition1 = delta_w > min_light_delta_w && delta_w < max_light_delta_w;//状况一：判断两灯条的左右距离差差
                bool  condition2= (delta_center_y < (rect_j.center.y+rect_i.center.y)*0.50 );//灯条的高度差 is_height_diff_catch
//                    cout<<"delta_center_y:"<<delta_center_y<<endl;
//                    cout<<"对比："<<(rect_j.center.y+rect_i.center.y)*0.5<<endl;
                bool condition3=(light_distance < MAX (leni,lenj) * 5);//灯条间的距离 (待改进)  light_distance_catch
//                    cout<<"light_distance:"<<light_distance<<endl;
//                    cout<<"对比："<<MAX (leni,lenj) * 5<<endl;
//                    cout<<"i的高："<<leni<<"   j的高："<<lenj<<endl;

                //判断目标左右灯柱的比例值(可选)
                bool condition4= delta_h < (rect_j.center.y+rect_i.center.y)*0.5;
                bool condition5=area_ratio<2;//灯条面积比值 (待改进)

                if(/*condition1*/ /*&&*/ condition2 && condition3 && condition4 && condition5)
                {
                    RotatedRect obj_rect = boundingRRect(rect_i, rect_j);//两灯条矩形的拟合矩形


#ifdef DEBUG
                    // for debug use

//                    float delta_lc_angle_max=0,delta_rc_angle_max=0;


                    float obj_rect_angle=obj_rect.angle;//拟合矩形的角度


                    delta_lc_angle=fabs(obj_rect_angle-angle_left);
                    delta_rc_angle=fabs(obj_rect_angle-angle_right);


                    bool delta_light_center_angle_max = delta_lc_angle < 9 && delta_rc_angle < 9;//左、右灯条与拟合矩形的角度之差
//                    cout<<"左差值："<<delta_lc_angle<<"   "<<"右差值："<<delta_rc_angle<<endl;

                    bool center_rect_angle = fabs(obj_rect_angle)<40;//拟合矩形的合适斜率
//                    cout<<"obj_rect_angle:"<<obj_rect_angle<<endl;





                    bool is_light_angle_catch;//右正左负
                    if(angle_left /angle_right > 0.0){
                        //同侧
//                        cout<<fabs(angle_left-angle_right)<<endl;
                        is_light_angle_catch = (fabs(angle_left-angle_right) <= 10);
                    } else if(angle_left < 0.0 && angle_right > 0.0){
                        //--- \ / --- 内八
//                        cout<<"内八："<<fabs(angle_left-angle_right)<<endl;
                        is_light_angle_catch = (0 <= fabs(angle_left-angle_right) && fabs(angle_left-angle_right) <= 10);
                    } else if(angle_left > 0.0 && angle_right < 0.0){
                        //--- / \ --- 外八
//                        cout<<"外八："<<fabs(angle_left-angle_right)<<endl;
                        is_light_angle_catch = (0 <= fabs(angle_left-angle_right) && fabs(angle_left-angle_right) <= 10);
                    } else if((angle_left ==0.0 /*|| angle_left == -0 */)&& angle_right != 0){
                        //左边竖直
//                        cout<<"左边竖直："<<fabs(angle_left-angle_right)<<endl;
                        is_light_angle_catch = /*false*/(/*170 <= fabs(angle_left-angle_right) || */fabs(angle_left-angle_right) <= 10);
                    } else if(angle_left !=0 && (angle_right == 0.0 /*|| angle_right == -0*/)){
                        //右边竖直
                        is_light_angle_catch = /*false*/(/*170 <= fabs(angle_left-angle_right) || */fabs(angle_left-angle_right) <= 10);
                    }



//                    cout<<obj_rect_angle<<endl;
                    float height_max=MAX(height_left,height_right);
                    cout<<"ratio_距离:"<<light_distance/height_max<<endl;
                    if( delta_light_center_angle_max
                            && center_rect_angle
                            && is_light_angle_catch)
                    {

                        cout<<"ratio:"<<delta_w/delta_h<<endl;
//                        cout<<"ratio_距离:"<<light_distance/height_max<<endl;

                        cout<<"angle:"<<obj_rect_angle<<endl;
                                vector < RotatedRect > armor_light(2);//储存一对灯条中的两个
                                armor_light[0]=rotateRect[i];
                                armor_light[1]=rotateRect[j];
                                armor_light_pair.push_back(armor_light);

//                                cout<<"left:"<<angle_left<<"   "<<"right:"<<angle_right<<endl;
//                                 cout<<"左宽："<<width_left<<"    右宽："<<width_right<<endl;
//                                 cout<<"宽比值："<<MAX(width_left,width_right)/MIN(width_left,width_right)<<endl;
                                Point2f vertice[4];
                                obj_rect.points(vertice);
                                for (int i = 0; i < 4; i++)
                                    line(src_img, vertice[i], vertice[(i + 1) % 4], Scalar(255, 255, 255), 2);                        
                    }

#endif




                    cout<<"------------------------------------"<<endl;
//                    cout<<"左差值："<<delta_lc_angle_max<<"   "<<"右差值："<<delta_rc_angle_max<<endl;

                }
            }
        }
#endif

//        for(size_t i=0;i<armor_light_pair.size();i++)
//        {

//        }


//        imshow("dst_img",dst_img);
#ifdef DEBUG_1
        imshow("src_img",src_img);
#endif
        imshow("copy_img",copy_img);


#ifdef VEDIO
        int a = waitKey(0);
        if ((char)a == 32)
            continue;

        if ((char)a == 27)
            break;
#endif

#ifdef CAMERA
        int a = waitKey(1);
        if ((char)a == 27)
            break;

#endif
    }

#ifdef WRITE
    write.close();
#endif
    cout<<"lou"<<endl;

    return 0;
}

RotatedRect boundingRRect( cv::RotatedRect  left,  cv::RotatedRect  right)
{

    Point2f  pl = left.center, pr = right.center;
    Point2f center = (pl + pr) / 2.0;

//    cv::Size2f wh_l = left.size;
//    cv::Size2f wh_r = right.size;
    // 这里的目标矩形的height是之前灯柱的width
    double width_l = MIN(left.size.width, left.size.height);
    double width_r = MIN(right.size.width, right.size.height);
    double height_l = MAX(left.size.width, left.size.height);
    double height_r = MAX(right.size.width, right.size.height);

    float width = POINT_DIST(pl, pr) - (width_l + width_r) / 2.0;
    float height = max(height_l, height_r);
    //float height = (wh_l.height + wh_r.height) / 2.0;
//    float angle = atan2(right.center.y - left.center.y, right.center.x - left.center.x);//jiao du fanwei zai 0~180
    float angle = atan((right.center.y-left.center.y)/(right.center.x-left.center.x));//-90~0~90

//    cout<<(atan((right.center.y-left.center.y)/(right.center.x-left.center.x)))* 180 / CV_PI<<endl;

    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

