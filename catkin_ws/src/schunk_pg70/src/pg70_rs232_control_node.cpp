

#include <pg70_rs232_control.h>
#include <pg70_action_server/pg70_grasp_action_server.h>
#include <pg70_action_server/pg70_move_action_server.h>

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "pg70_control");
   ros::NodeHandle nh;

   //Create gripper object instance
   
   schunk_pg70::PG70_serial gripper(&nh);
      
   //Initialize user interface
   ros::ServiceServer reference_service         = nh.advertiseService("schunk_pg70/reference", &schunk_pg70::PG70_serial::referenceCallback, &gripper);
   ros::ServiceServer set_position_service      = nh.advertiseService("schunk_pg70/set_position", &schunk_pg70::PG70_serial::setPositionCallback, &gripper);
   ros::ServiceServer set_pvac_service          = nh.advertiseService("schunk_pg70/set_pvac", &schunk_pg70::PG70_serial::setPVACCallback, &gripper);
   ros::ServiceServer get_error_service         = nh.advertiseService("schunk_pg70/get_error", &schunk_pg70::PG70_serial::getErrorCallback, &gripper);
   ros::ServiceServer get_position_service      = nh.advertiseService("schunk_pg70/get_position", &schunk_pg70::PG70_serial::getPositionCallback, &gripper);
   ros::ServiceServer move_time_service	      = nh.advertiseService("schunk_pg70/move_time", &schunk_pg70::PG70_serial::movePosTimeCallback, &gripper);
   ros::ServiceServer move_time_loop_service    = nh.advertiseService("schunk_pg70/move_time_loop", &schunk_pg70::PG70_serial::movePosTimeLoopCallback, &gripper);
   ros::ServiceServer get_current_service       = nh.advertiseService("schunk_pg70/get_current", &schunk_pg70::PG70_serial::getCurrentCallback, &gripper);
   ros::ServiceServer acknowledge_error_service = nh.advertiseService("schunk_pg70/acknowledge_error", &schunk_pg70::PG70_serial::acknowledgeErrorCallback, &gripper);
   ros::ServiceServer stop_service              = nh.advertiseService("schunk_pg70/stop", &schunk_pg70::PG70_serial::stopCallback, &gripper);
   
   ros::Timer timer = nh.createTimer(ros::Duration(schunk_pg70::TF_UPDATE_PERIOD), &schunk_pg70::PG70_serial::timerCallback, &gripper);
   gripper.joint_pub = nh.advertise<sensor_msgs::JointState>("schunk_pg70/joint_states", 1); 
   
	pg70_action_server::pg70GraspActionServer grasp_server("schunk_pg70/Grasp");
   pg70_action_server::pg70MoveActionServer move_server("schunk_pg70/Move");
	
   ros::spin();
   
   return 0;
}
