#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "rplidar.h"
#include "cmath"
#include "rplidar_ros/safety_ori.h"//定义msg调用需要在头文件声明,虽然这个.h文件没找到
//								     大胆预测这个头文件定义在了ros空间内,随着catkin_make的使用生成
//									 使用VSCode内置Ctrl键可看到该头文件由gencpp生成,预测正确
#include "rplidar_ros/min_dist.h"//include the msg that could boardcast the min dist
#include "rplidar_ros/ave_dist.h"

#define RAD2DEG(x) ((x)*180./M_PI)//宏定义，弧度转角度//硕师兄NB
#define DEG2RAD(x) ((x)*pi/180)
#define pi 3.1415926

using namespace rp::standalone::rplidar;

rplidar_ros::safety_ori safe_ori;
rplidar_ros::min_dist min_dist;
rplidar_ros::ave_dist ave_dist_ori;

sensor_msgs::LaserScan laser;
std_msgs::Bool safe_bol;
// std_msgs::Float32 ave_dist;
float global_laserData[360];//激光雷达扫描一个周期生成点云距离信息
int cnt = 0;
float warning_distence = 1.8;
float safety_distence = 0.8;
int report_count;

ros::Publisher safe_ori_pub;
ros::Publisher safe_bol_pub;
// ros::Publisher ave_dist_pub;
ros::Publisher min_dist_pub;
ros::Publisher ave_dist_ori_pub;

void judge();

void safety_rep(const sensor_msgs::LaserScan::ConstPtr& LS){
    memset(global_laserData,0,sizeof(global_laserData));
    laser = *LS;
    cnt = laser.scan_time/laser.time_increment;
    float ave;
    int i;
    for(i=0;i<cnt;i++){
        // global_laserData[i][0] = RAD2DEG(laser.angle_min + laser.angle_increment * i);
        
        if(laser.ranges[i] != INFINITY){
            global_laserData[i] = laser.ranges[i];
        }
    }
    judge();
    min_dist_pub.publish(min_dist);
    safe_bol_pub.publish(safe_bol);
    safe_ori_pub.publish(safe_ori);
    ave_dist_ori_pub.publish(ave_dist_ori);
    
    // ROS_INFO("SafeOri_F%d",safe_ori.Forward);
    // ROS_INFO("SafeOri_L%d",safe_ori.Leftward);
    // ROS_INFO("SafeOri_B%d",safe_ori.Backward);
    // ROS_INFO("SafeOri_R%d",safe_ori.Rightward);
    // ROS_INFO("FAveDist%f",ave_dist_ori.F_ave_dist);
    
    ros::Duration(0.12).sleep();
}

int main(int argc,char** argv){
    ros::init(argc,argv,"srge3");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Subscriber scan_data = nh.subscribe<sensor_msgs::LaserScan>("/scan",1000,&safety_rep);
    safe_ori_pub = nh.advertise<rplidar_ros::safety_ori>("safety_report/ori",20);
    safe_bol_pub = nh.advertise<std_msgs::Bool>("safety_report/bool",10);
    // ave_dist_pub = nh.advertise<std_msgs::Float32>("safety_report/adist",20);
    min_dist_pub = nh.advertise<rplidar_ros::min_dist>("safety_report/min_dist",20);
    ave_dist_ori_pub = nh.advertise<rplidar_ros::ave_dist>("/safety_report/ave_dist_ori",20);
    report_count = 0;
    ros::spin();
    return 0;
}

void judge(){
    int i,j;
    double temprecorder;
    safe_bol.data = 1;
    safe_ori.Forward = 2;
    safe_ori.Rightward = 2;
    safe_ori.Leftward = 2;
    safe_ori.Backward = 2;
    min_dist.F_min_dist = INFINITY;
    min_dist.R_min_dist = INFINITY;
    min_dist.L_min_dist = INFINITY;
    min_dist.B_min_dist = INFINITY;
    ave_dist_ori.F_count = 0;
    ave_dist_ori.R_count = 0;
    ave_dist_ori.L_count = 0;
    ave_dist_ori.B_count = 0;
    ave_dist_ori.F_ave_dist = 0;
    ave_dist_ori.R_ave_dist = 0;
    ave_dist_ori.L_ave_dist = 0;
    ave_dist_ori.B_ave_dist = 0;
    
    temprecorder = 0;
    for(i=140;i<220;i++){
        if(global_laserData[i] != 0){
            if(-global_laserData[i]*cos(i*pi/180)<min_dist.F_min_dist){
                min_dist.F_min_dist = global_laserData[i];
            }
            ave_dist_ori.F_count++;
            temprecorder-=global_laserData[i]*cos(DEG2RAD(i));
        }
    }
    if(ave_dist_ori.F_count!=0)
        ave_dist_ori.F_ave_dist = temprecorder/ave_dist_ori.F_count;
    // std::cout<<"adoF"<<ave_dist_ori.F_ave_dist<<std::endl;
    
    temprecorder = 0;
    for(i=50;i<130;i++){
        if(global_laserData[i] > 0.05){
            if(global_laserData[i]*sin(i*pi/180)<min_dist.R_min_dist){
                min_dist.R_min_dist = global_laserData[i];
            }
            ave_dist_ori.R_count++;
            temprecorder+=global_laserData[i]*sin(DEG2RAD(i));
        }
    }
    if(ave_dist_ori.R_count!=0)
        ave_dist_ori.R_ave_dist = temprecorder/ave_dist_ori.R_count;
    // std::cout<<"adoR"<<ave_dist_ori.R_ave_dist<<std::endl;
    
    temprecorder = 0;
    for(i=0;i<40;i++){
        if(global_laserData[i] > 0.05){
            if(global_laserData[i]*abs(cos(DEG2RAD(i)))<min_dist.B_min_dist){
                min_dist.B_min_dist = global_laserData[i];
            }
            ave_dist_ori.B_count++;
            temprecorder += global_laserData[i]*cos(DEG2RAD(i));
        }
    }
    for(i=320;i<cnt;i++){
        if(global_laserData[i] > 0.05){
            if(global_laserData[i]*abs(cos(DEG2RAD(i)))<min_dist.B_min_dist){
                min_dist.B_min_dist = global_laserData[i];
            }
            ave_dist_ori.B_count++;
            temprecorder += global_laserData[i]*cos(DEG2RAD(i));
        }
    }

    // std::cout<<"test_tr_B"<<temprecorder<<std::endl;
    if(ave_dist_ori.B_count!=0)
        ave_dist_ori.B_ave_dist = temprecorder/ave_dist_ori.B_count;
    // std::cout<<"adoB"<<ave_dist_ori.B_ave_dist<<std::endl;
    
    temprecorder = 0;
    for(i=230;i<310;i++){

        if(global_laserData[i] > 0.05){
            if(-global_laserData[i]*sin(i*pi/180)<min_dist.L_min_dist){
                min_dist.L_min_dist = global_laserData[i];
            }
            ave_dist_ori.L_count++;
            temprecorder-=global_laserData[i]*sin(i*pi/180);
        }
    }
    // std::cout<<"test_tr_L"<<temprecorder<<std::endl;
    if(ave_dist_ori.L_count!=0)
        ave_dist_ori.L_ave_dist = temprecorder/ave_dist_ori.L_count;
    // std::cout<<"adoL"<<ave_dist_ori.L_ave_dist<<std::endl;
    
    if(min_dist.F_min_dist<warning_distence){
        safe_ori.Forward = 1;
        safe_bol.data = 0;
    }
    if(min_dist.F_min_dist<safety_distence){
        safe_ori.Forward = 0;
    }
    if(min_dist.L_min_dist<warning_distence){
        safe_ori.Leftward = 1;
    }
    if(min_dist.L_min_dist<safety_distence){
        safe_ori.Leftward = 0;
    }
    if(min_dist.B_min_dist<warning_distence){
        safe_ori.Backward = 1;
    }
    if(min_dist.B_min_dist<safety_distence){
        safe_ori.Backward = 0;
    }
    if(min_dist.R_min_dist<warning_distence){
        safe_ori.Rightward = 1;
    }
    if(min_dist.R_min_dist<safety_distence){
        safe_ori.Rightward = 0;
    }
}