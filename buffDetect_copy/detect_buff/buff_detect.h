#ifndef BUFF_DETECT_H
#define BUFF_DETECT_H

#include"/home/jun/workplace/RM/buffDetect_copy/base.h"
#include"solve_buff/solve_pnp.h"

#define DEFAULT 0
#define FIRE 3
#define RESET 4

typedef enum{UNKOWN,INACTION,ACTION}ObjectType;
class Object
{
public:
    Object(){}
    ~Object(){}
    void smallUpdate_Order(); // 更新能量机关装甲板的绝对位置
    void bigUpdate_Order(); //更新能量机关叶片的绝对位置
    void knowYour_Self(Mat &img);    //判断能量机关扇叶的状态（激活　未激活）

    RotatedRect small_rect_;    // 能量机关扇叶内轮廓
    RotatedRect big_rect_;  // 能量机关扇叶外轮廓
    vector<Point2f> points_2d_; //内轮廓装点集
    vector<Point2f> big_points_2d_; //叶片点集
    float angle_ = 0; //待激活装甲板 0~360度
    float diff_angle = 0; //待激活装甲板和叶片的相对角度（判断是否垂直）
    int type_ = UNKOWN; //是否激活种类初始化
};


class FireTask{
public:
    FireTask(){}
    // 开火条件是当输出的角度趋近于０一段时间开火
    int run(Point2f current_angle, bool is_change_target)
    {
        // 获得发射机会的条件
        if(wait_action_flag == true ){// 重复激活
            if(is_change_target){
                wait_action_flag = false;
//                cout<<"改变完目标了"<<endl;
            }else{
                double t2 = getTickCount();
                double t = (t2 - change_time)*1000 / getTickFrequency();
                if(t > repeat_time){
//                  cout<<"重复激活"<<endl;
                    change_time = getTickCount();
#ifndef NO_REPEAT_FIRE
                    if(repeat_fire_flag == 1){
                        shoot_chance_ = true;
//                        cout<<"repeat"<<endl;
                    }else{
                        shoot_chance_ = false;
//                        cout<<"repeat-default"<<endl;
                    }
#endif
                }
            }
        }

        if(is_change_target && wait_action_flag== false){
            shoot_chance_ = true;
            change_time = getTickCount();
            wait_action_flag = true;
        }
        // 控制发射条件
        int command = DEFAULT;
//        cout<<"current_angle.x:"<<current_angle.x <<endl;
//        cout<<"current_angle.y:"<<current_angle.y <<endl;
        //每帧之间的最大偏差量，可以修改
        if(fabs(current_angle.x) < limit_angle_x_
                && fabs(current_angle.y) < limit_anlge_y_)
        {
            // 满足小于一段时间计数
            cnt_ ++;
            time ++;
            cout<<"第"<<time<<"帧"<<endl;
//            cout<<"时间量："<<cnt_<<endl;
        }else {
            // 不满足条件加速减时间，可以修改
            cnt_ -= 3;
            if(cnt_<0) cnt_ = 0;
        }

        last_angle_ = current_angle;

        if(cnt_ > max_cnt_)
        {
            cnt_ = 0;
            if(shoot_chance_ == true)
            {
#ifndef NO_FIRE
                if(fire_flag == 1){
                    command = FIRE;
//                    cout<<"fire"<<endl;
                }else{
                     command = DEFAULT;
//                     cout<<"fire-default"<<endl;
                }

#else
                command = DEFAULT;
#endif
                fire_cnt++;
//                cout<<"开火："<<fire_cnt<<endl;
                shoot_chance_ = false;
            }else {
                command = DEFAULT;
            }
        }else{
            command = DEFAULT;
        }
        return command;
    }

    void set_fire_chance(){
        shoot_chance_ = true;
        change_time = getTickCount();
        wait_action_flag = true;
    }
public:
    // 获得发射机会的参数
    Point2f last_angle_ = Point2f(0,0);
    bool wait_action_flag = true;
    double change_time;
    bool shoot_chance_ = true;
    int repeat_time = REPEAT_FIRE_TIME;
    // debug
    int fire_flag = 1;//手动控制，1为可以开火，0为不能开火
    int repeat_fire_flag = 1;

    // 控制开火的参数
    int cnt_ = 0;
    int time = 0;
    int fire_cnt = 0;
    int max_cnt_ = 50;             // 满足条件次数,根据帧率进行调整
    float limit_angle_x_ = 15.0f;    // 条件角度阈值 2.0f
    float limit_anlge_y_ = 15.0f;
};

/**
 * @brief 能量机关复位任务
 */
class ResetTask{
public:
    ResetTask(){}
    // 复位归中条件是，丢失目标超过几帧
    int run(bool find_flag)
    {
        int command = DEFAULT;
        if(find_flag == false){
            cnt++;
        }else{
            cnt = 0;
        }
        // 判断归中条件
        if(cnt > max_cnt_)
        {
            command = RESET;

        }else{
            command = DEFAULT;
        }
        return command;
    }
public:
    int cnt = 0;
    int max_cnt_ = 30;   // 最大丢失目标次数，根据帧率进行调整
};

/**
 * @brief 能量机关自动控制
 */
class AutoControl
{
public:
    AutoControl(){}
    int run(float current_yaw, float &current_pit,int find_flag, float diff_buff_angle){
        int command_tmp = DEFAULT;
        // 开火任务启动
        bool is_change_target = false;
        //角度范围可以修改
        if(fabs(diff_buff_angle) > 40  && fabs(diff_buff_angle) < 350){
            filter_flag = true;
        }else{
            if(filter_flag == true){
//                printf("change\r\n");
                is_change_target = true;
                filter_flag = false;
            }
        }
//        cout<<"目标是否改变完了："<<is_change_target<<endl;
        command_tmp = fire_task.run(Point2f(current_yaw, current_pit), is_change_target);
        if(command_tmp != DEFAULT){
            set_fire(command_);
            fire_cnt++;
//            cout<<"fire_cnt = "<<fire_cnt<<endl;
//            cout<<"current command_:开火 = "<<command_<<endl;
            return command_;
        }
        // 复位任务启动
        command_tmp = reset_task.run(find_flag);
        if(command_tmp != DEFAULT){
            // 复位绝对角度
            lose_cnt++;
            set_reset(command_);
//            current_pit = RESET_ANGLE;//该角度可能还要加上正值，因为不像上的云台角度那么高,根据今年情况还能加上一些不同范围调整相应的角度
            search_target(current_pit,current_yaw);//泛用性更广的复位操作，代替上式
            fire_task.set_fire_chance();// 复位获得一次开火机会
//            cout<<"current command_:复位 = "<<command_<<endl;
            return command_;
        }
        // 通过上面开火任务及复位任务还是得到默认指令，则判断跟随还是不跟随
        if(command_tmp == DEFAULT)
        {
            if(find_flag == 1)
            {
                set_follow(command_);
//                cout<<"current command_:跟随 = "<<command_<<endl;
            }else
            {
                set_no_follow(command_);
//                cout<<"current command_:不跟随 = "<<command_<<endl;
            }
        }

        return command_;
    }

private:
    // 设置命令相关函数
    // 置1用 |    清零用&   跟随位0x01 开火位0x02 复位位0x04
    void set_follow(int &command){
        command &= ~0x04;
        command |= 0x01;

    }
    void set_no_follow(int &command){
        command &= ~0x04;
        command &= ~0x01;

    }
    void set_fire(int &command){
        command |= 0x01;
        command ^= 0x02;
    }

    void set_reset(int &command){
        command |= 0x04;
    }

    //暂时只是上下摆动，后续如果需要就再加上左右的搜索
    void search_target(float &angle_pit,float &angle_yaw){
        if(lose_cnt<50){
            angle_pit = RESET_ANGLE;
            angle_yaw = angle_yaw;
        }
        else if(lose_cnt=50){
            angle_pit += (-50)*RESET_ANGLE;
            angle_yaw = angle_yaw;
        }
        else if(lose_cnt>50 && lose_cnt<100)
        {
            angle_pit = -RESET_ANGLE;
            angle_yaw = angle_yaw;
        }
    }

public:
    FireTask fire_task;     // 开火任务
    ResetTask reset_task;   // 复位任务

    bool filter_flag = false;

    int command_ = 0;
    int fire_cnt = 0;
    int lose_cnt = 0;
};

class BuffDetector
{
public:
    BuffDetector() {solve_buff = Solve_Buff();} //solve.cpp的构造函数
    ~BuffDetector(){}
    int buffDetect_Task(Mat & frame); //主逻辑函数
    float angle_x, angle_y, dist; //输出数据(角度，距离)

    //自动控制 类申明




private://类的声明
    Solve_Buff solve_buff;
    Object object;
    Object final_target;
    Object object_tmp;
    AutoControl auto_control;

private:
    void imageProcess(Mat & frame); //预处理
    bool findTarget(Mat & frame);   //寻找叶片以及待激活装甲板
    bool findCenter_R(Mat & bin_img, Mat &frame); //寻找R
    int getState(); //能量机关顺逆时针滤波函数

private://Object object_tmp新类，用于装清洗出来的新数据 和一些需要公用的数据
     vector<Point2f> points_2d;
     //vector<Point2f> big_points_2d;
     Point2f target_center;//小轮廓圆心
     Point2f roi_center;//假定圆心
     Point2f circle_center;//中心R
     Point2f pre_center;
     Mat bin_Img;
     RotatedRect solve_rect;

     Point2f roi_power_center;//test

private://能量机关顺逆时针判断
    float buff_angle_ = 0;
    float diff_angle_ = 0;
    float last_angle = 0;
    float d_angle_ = 0;
    int find_cnt_ = 0;
    int direction_tmp_ = 0;

private://大神符加速函数
    float diff_angle_large = 0;
    float last_angle_large = 0;
    double timing_point_1 = 0;
    double timing_point_2 = 0;
    double current_speed = 0;

    double first_frame_time = 0;
    double second_frame_time = 0;
};

#endif // BUFF_DETECT_H
double pointDistance(Point2f & p1, Point2f & p2);//计算两点之间距离
int getRect_Intensity(const Mat &frame, Rect rect);//创建两小roi，判断叶片是否为未激活状态
