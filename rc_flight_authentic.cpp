// 遥控器
// 起飞
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include "std_msgs/Bool.h"

sensor_msgs::Joy rc;
bool status_of_permission=0;
bool rc_flight_permission;
int rc_flight_permission_counter = 0;
int counter4counter = 0;
int SiCiChengGong = 0;
int JiLuRC = 0;
int loop_account=0;

void show_what_listener_listenning(const sensor_msgs::Joy::ConstPtr& msg){//这里时接收遥控器gear信息的标准调用函数
	rc = *msg;
	if(status_of_permission == 1){
		loop_account++;
		return;
	}
	if(rc_flight_permission_counter - counter4counter < 8){
		if(JiLuRC !=  rc.axes[5]){
			SiCiChengGong++;
			ROS_INFO("ChengGongCiShu:%d",SiCiChengGong);
			//ROS_INFO("Counter:%d",counter4counter);
			JiLuRC = rc.axes[5];
			counter4counter = rc_flight_permission_counter;
		}
	}
	//ROS_INFO("Counter^&%d",counter4counter);
	//ROS_INFO("Counter##%d",rc_flight_permission_counter);
	if(rc_flight_permission_counter - counter4counter > 10){
		SiCiChengGong = 0;
		counter4counter = rc_flight_permission_counter;
		ROS_INFO("ChengGongCiShu_%d",SiCiChengGong);
		//ROS_INFO("Counter_%d",counter4counter);
	}
	if(SiCiChengGong == 4){
		status_of_permission = 1;
		ROS_INFO("FLIGHT ALLOWED GOOD_LUCK!");
	}
}


int main(int argc, char** argv){
	ros::init(argc,argv,"flight_permission");
	ros::NodeHandle nh;
	ros::Subscriber lisn = nh.subscribe<sensor_msgs::Joy>("dji_sdk/rc", 100, show_what_listener_listenning); //接收gear信息，进而执行起降操作
	ros::Publisher could_fly = nh.advertise<std_msgs::Bool>("could_fly",10);
	ros::Rate loop_rate(10);
	std_msgs::Bool FeiXingXuKe;
	
	while(ros::ok()){
		rc_flight_permission_counter++;
		//ROS_INFO("rfpc:%d",rc_flight_permission_counter);
		FeiXingXuKe.data = status_of_permission;
		could_fly.publish(FeiXingXuKe);
		loop_rate.sleep();
		ros::spinOnce();
	}
  return 0;

}
