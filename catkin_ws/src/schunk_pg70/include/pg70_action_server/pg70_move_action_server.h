#ifndef PG70_MOVE_ACTION_SERVER_H
#define PG70_MOVE_ACTION_SERVER_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <time.h>
#include <list>
#include <string.h>

#include <actionlib/server/simple_action_server.h>


#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <schunk_pg70/MoveAction.h>
#include <schunk_pg70/MoveGoal.h>
#include <schunk_pg70/MoveResult.h>
#include <schunk_pg70/stop.h>


namespace pg70_action_server
{

class pg70MoveActionServer
{
public:

   pg70MoveActionServer(std::string name);
   ~pg70MoveActionServer();

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


   actionlib::SimpleActionServer<schunk_pg70::MoveAction> pg70_as_;
   std::string action_name;

   schunk_pg70::MoveAction grasp_action;
   schunk_pg70::MoveGoal grasp_goal;
   schunk_pg70::MoveResultConstPtr grasp_result;
   schunk_pg70::MoveResult pg70_as_result_;
   sensor_msgs::JointState joint_state_;

   serial::Serial *port;


   void execute_cb(const schunk_pg70::MoveGoalConstPtr& goal);
   void jointPos_cb(const sensor_msgs::JointState& msg);



   ros::Subscriber joint_sub_;
   ros::ServiceClient stop_client_;

   schunk_pg70::stop stop_;


};

} // pg70_action_server
#endif