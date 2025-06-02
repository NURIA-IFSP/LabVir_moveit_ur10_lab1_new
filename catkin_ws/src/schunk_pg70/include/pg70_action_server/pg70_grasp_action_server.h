#ifndef PG70_GRASP_ACTION_SERVER_H
#define PG70_GRASP_ACTION_SERVER_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <time.h>
#include <list>
#include <string.h>

#include <actionlib/server/simple_action_server.h>


#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <schunk_pg70/GraspAction.h>
#include <schunk_pg70/GraspGoal.h>
#include <schunk_pg70/GraspResult.h>


#include <schunk_pg70/set_pvac.h>
#include <schunk_pg70/stop.h>


namespace pg70_action_server
{


class pg70GraspActionServer
{
public:

   pg70GraspActionServer(std::string name);
   ~pg70GraspActionServer();

private:

   ros::NodeHandle nh_;

   int gripper_id_;
   std::string port_name_;
   int baudrate_;
   serial::Serial *com_port_;
   
   float act_position_;
   float act_velocity_;
   float act_current_;
   uint8_t pg70_error_;


   actionlib::SimpleActionServer<schunk_pg70::GraspAction> pg70_as_;
   std::string action_name;

   schunk_pg70::GraspAction grasp_action;
   schunk_pg70::GraspGoal grasp_goal;
   schunk_pg70::GraspResultConstPtr grasp_result;
   schunk_pg70::GraspResult pg70_as_result_;
   sensor_msgs::JointState joint_state_;

   serial::Serial *port;


   void execute_cb(const schunk_pg70::GraspGoalConstPtr& goal);
   void jointPos_cb(const sensor_msgs::JointState& msg);

   // void float_to_IEEE_754(float position, unsigned int *output_array);
   // uint16_t CRC16(uint16_t crc, uint16_t data);  

   ros::Subscriber joint_sub_;
   ros::ServiceClient stop_client_;

   schunk_pg70::set_pvac new_command_;
   schunk_pg70::stop stop_;


};

} // pg70_action_server
#endif