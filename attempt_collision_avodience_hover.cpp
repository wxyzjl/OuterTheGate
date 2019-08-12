// 悬浮
// 靠近
// 躲避

#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include "rplidar_ros/safety_ori.h"
#include "geometry_msgs/Vector3.h"

sensor_msgs::Joy rc;
bool could_fly;
bool mission_trigger;
int task_trigger[3];

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
rplidar_ros::safety_ori safety_detection_result;

Mission square_mission;

void detect_resu(const rplidar_ros::safety_ori::ConstPtr& result){
  safety_detection_result = *result;
}

std_msgs::Bool listener;

void list_auth_func(const std_msgs::Bool::ConstPtr& al){
    listener = *al;
    could_fly = listener.data;
}

int main(int argc, char** argv){
	  ros::init(argc,argv,"acah");
	  ros::NodeHandle nh;
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    ros::Rate loop_rate(10);
	
    // Subscribe to messages from dji_sdk_node
    ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
    ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
    ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
    ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
    ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

    // Subscribe the authentic information by switch the "gear" pannel for 4 times
    ros::Subscriber CAlistener  = nh.subscribe<rplidar_ros::safety_ori>("/safety_report/ori",100,&detect_resu);
    
    // Subscribe to the "gear" 
    ros::Subscriber list_auth = nh.subscribe("could_fly", 10, &list_auth_func);

    
    // Publish the control signal
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    
    // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
    // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
    // properly in function Mission::step()
    ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
    
    // Basic services
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

    bool obtain_control_result = obtain_control();
    int collision_counter=0;
    int avoid_back;
    bool takeoff_result=0;
    bool if_land = 0;
    int safety_counter=0;
    bool could_land = 0;
    bool bol4to=1;
    bool inloop_=1;
    mission_trigger=0;
    task_trigger[0]= 0;
    task_trigger[1]= 0;
    task_trigger[2]= 0;
    int time_trigger = 10;//设计一个触发时间，如果连续触发同一任务，延长触发事件

    while(ros::ok()){
      if(could_fly&&is_M100()&&bol4to)
        {
          ROS_INFO("M100 taking off!");
          takeoff_result = M100monitoredTakeoff();
          could_land = 1;
          bol4to = 0;
        }
        ros::spinOnce();
        if(takeoff_result&&inloop_)
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(5, 0, 0, 0);
          square_mission.state = 0;
          //ROS_INFO("initial setting_won't change the oritation");
          inloop_ = 0;
        }
        if(takeoff_result){
          std::cout<<"起飞结束"<<std::endl;
          break;
          }    
    }
    while(ros::ok()){
      
      if(safety_detection_result.Forward == 2){
        collision_counter = 0;
        safety_counter++;
        if(safety_counter>300){
          ROS_INFO("ENOUGH FREE SPACE FOR HOVERING");
          safety_counter = 0;
        }
        task_trigger[0]= 0;
        task_trigger[1]= 0;
        task_trigger[2]= 0;
      }
      if(safety_detection_result.Forward != 2){
        safety_counter = 0;
        collision_counter++;
        if(safety_detection_result.Backward == 0){

          task_trigger[0]++;
          task_trigger[1]= 0;
          task_trigger[2]= 0;

          if(task_trigger[0] == time_trigger){
            mission_trigger = 1;
            square_mission.state = 1;
            std::cout<<"后方无空间，需要上升"<<std::endl;
          }
        }
        if(safety_detection_result.Backward == 1){

          task_trigger[0]= 0;
          task_trigger[1]++;
          task_trigger[2]= 0;

          if(task_trigger[1] == time_trigger){
            mission_trigger = 1;
            square_mission.state = 2;
            std::cout<<"后方空间较小，后退较小距离"<<std::endl;
          }
        }
        if(safety_detection_result.Backward == 2){

          task_trigger[0]= 0;
          task_trigger[1]= 0;
          task_trigger[2]++;

          if(task_trigger[2] == time_trigger){
            mission_trigger = 1;
            square_mission.state = 3;
            std::cout<<"后方空间充足，适量后退"<<std::endl;
          }
        }
      }
      // if(collision_counter!=0)
        // ROS_INFO("collision_counter_shows%d",collision_counter);
      if(collision_counter>50000&&could_land){
        ROS_INFO("LONG TIME NO SPACE!!");
        ROS_INFO("LiuLe_LiuLe..FeiBuLiaoLe..GaoCi");
        if_land = M100monitoredLand();
      }
      loop_rate.sleep();
      if(if_land) return 0;//校验降落即终止程序
      ros::spinOnce();
    }

}

void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    // ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    // ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_local_position.z + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 50)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.5 &&
      std::abs(yOffsetRemaining) < 0.5 &&
      std::abs(zOffsetRemaining) < 0.5 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 50)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }

}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
//   Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02)&&square_mission.state!=6)
  {
    start_time = ros::Time::now();
    switch(square_mission.state)
    {
      case 0:
        // if(!square_mission.finished&&lock_ori)
        // {
        //   square_mission.step();
        // }
        // else
        // {
        //   square_mission.reset();
        //   square_mission.start_gps_location = current_gps;
        //   square_mission.start_local_position = current_local_pos;
        //   square_mission.setTarget(0, 0, 0, 90);
        //   mission_trigger = 0;
        //   ROS_INFO("backaBit");
        // }
        break;

      case 1:
        if(!square_mission.finished&&!mission_trigger)
        {
          square_mission.step();
          // std::cout<<"仍在执行任务1"<<std::endl;
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(0, 0, 2.2, 0);
          mission_trigger = 0;
          ROS_INFO("getting higher");
        }
        break;

      case 2:
        if(!square_mission.finished&&!mission_trigger)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(-0.3, 0, 1.4, 0);
          mission_trigger = 0;
          ROS_INFO("backaBit");
        }
        break;
      case 3:
        if(!square_mission.finished&&!mission_trigger)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(-1.2, 0, 1.4, 0);
          mission_trigger = 0;
          ROS_INFO("back more");
        }
        break;
      // case 4:
      //   if(!square_mission.finished)
      //   {
      //     square_mission.step();
      //   }
      //   else
      //   {
      //     square_mission.reset();
      //     square_mission.start_gps_location = current_gps;
      //     square_mission.start_local_position = current_local_pos;
      //     square_mission.setTarget(0, 5, 0, 0);
      //     square_mission.state = 5;
      //     ROS_INFO("##### Start route %d ....", square_mission.state);
      //   }
      //   break;
      // case 5:
      //   if(!square_mission.finished)
      //   {
      //     square_mission.step();
      //   }
      //   else
      //   {
      //     ROS_INFO("##### Mission %d Finished ....", square_mission.state);
      //     square_mission.state = 0;
      //     land_request = 1;
      //     ROS_INFO("land_request is: %d",land_request);
      //     square_mission.state = 6;
      //     break;
      //   }
      //   break;
    }
    // if(land_request){
		// 	ROS_INFO("M100 landing!");
		// 	land_result = M100monitoredLand();
		// }
  }
}

bool M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool
M100monitoredLand()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND))
  {
    return false;
  }
  
  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude > 0.1)
  {
    ROS_ERROR("Land failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful land!");
    ros::spinOnce();
  }

  return true;
}

// bool
// M100monitoredGoHome()
// {
//   ros::Time start_time = ros::Time::now();

//   float home_altitude = current_gps.altitude;//在起飞时写一个函数把current_gps输出到外部，进行调用
//   if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_GOHOME))
//   {
//     return false;
//   }

//   ros::Duration(0.01).sleep();
//   ros::spinOnce();

//   if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
//       current_gps.altitude - home_altitude < 1.0)//没有找到M100FlightStatus定义的文件，无法明确这个东西的使用方法__更正 找到了 但是没记录文件位置
//   {
//     ROS_ERROR("Takeoff failed.");
//     return false;
//   }
//   else
//   {
//     start_time = ros::Time::now();
//     ROS_INFO("Successful gohome!");
//     ros::spinOnce();
//   }

//   return true;
// }

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
