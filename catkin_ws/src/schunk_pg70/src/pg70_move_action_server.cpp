/* 
* Action server interface to the grasp ction of the schunk pg70
*/


#include <pg70_action_server/pg70_move_action_server.h>
#include <pg70_rs232_control.h>

namespace pg70_action_server
{
pg70MoveActionServer::pg70MoveActionServer(std::string name) :
   pg70_as_(nh_, name, boost::bind(&pg70MoveActionServer::execute_cb, this, _1), false),
   action_name(name)
{

   ROS_INFO("PG70 Move Action Client: Init");

   nh_.getParam("/schunk_pg70/Grasp/portname", port_name_);
   nh_.getParam("/schunk_pg70/Grasp/baudrate", baudrate_);
   nh_.getParam("/schunk_pg70/Grasp/gripper_id", gripper_id_);

   joint_sub_ = nh_.subscribe("/schunk_pg70/joint_states", 1, &pg70MoveActionServer::jointPos_cb, this);

   ROS_INFO("Stop service Connection: Waiting");  
   stop_client_ = nh_.serviceClient<schunk_pg70::stop>("/schunk_pg70/stop");
   stop_client_.waitForExistence();
   ROS_INFO("Stop service Connection: OK");

   pg70_as_.start();
   ROS_INFO("PG70 Move Action Client: OK");

}

pg70MoveActionServer::~pg70MoveActionServer()
{
  com_port_->close();      //Close port
  delete com_port_;        //delete object
}


void pg70MoveActionServer::execute_cb(const schunk_pg70::MoveGoalConstPtr& goal)
{
  

   /*
   act_position_ = goal->position;
   */

   float target_pos = goal->position*1000;
   float target_vel = goal->velocity*1000;
   float target_acc = goal->acceleration*1000;
	float current = schunk_pg70::DEFAULT_CURRENT;

	ROS_INFO_STREAM("PG70: Moving from: " << act_position_ << " [mm] to " << target_pos << " [mm]");
	ROS_INFO_STREAM("PG70: Data used: " << target_vel << " [mm/s], " << target_acc << "[mm/s2], " << current <<" [A]");
	

   // aggiungere Controllo correttezza della richiesta

   if(schunk_pg70::checkRequestLimit(target_pos, target_vel, target_acc))
   {      
      ROS_WARN("Goal not accepted");
      pg70_as_result_.success = false;
      pg70_as_.setSucceeded(pg70_as_result_);
      return;
   }

	std::vector<uint8_t> output;
	output.push_back(0x05);                //message from master to module
	output.push_back(gripper_id_);          //module id
	output.push_back(0x15);                //15-Len
	output.push_back(0xB1);                //Command "mov pos"

	//Position <0-69>mm
	unsigned int IEEE754_bytes[4];
	schunk_pg70::float_to_IEEE_754(target_pos,IEEE754_bytes);

	output.push_back(IEEE754_bytes[0]);    //Position first byte
	output.push_back(IEEE754_bytes[1]);    //Position second byte
	output.push_back(IEEE754_bytes[2]);    //Position third byte
	output.push_back(IEEE754_bytes[3]);    //Position fourth byte

	//Velocity<0-82>mm/s
	schunk_pg70::float_to_IEEE_754(target_vel, IEEE754_bytes);
	output.push_back(IEEE754_bytes[0]);    //Velocity first byte
	output.push_back(IEEE754_bytes[1]);    //Velocity second byte
	output.push_back(IEEE754_bytes[2]);    //Velocity third byte
	output.push_back(IEEE754_bytes[3]);    //Velocity fourth byte

	//Acceleration<0-320>mm/s2
	schunk_pg70::float_to_IEEE_754(target_acc, IEEE754_bytes);
	output.push_back(IEEE754_bytes[0]);    //Acceleration first byte
	output.push_back(IEEE754_bytes[1]);    //Acceleration second byte
	output.push_back(IEEE754_bytes[2]);    //Acceleration third byte
	output.push_back(IEEE754_bytes[3]);    //Acceleration fourth byte

	//Current<0-2999>mA
	//current = current / 1000;
	schunk_pg70::float_to_IEEE_754(current, IEEE754_bytes);
	output.push_back(IEEE754_bytes[0]);    //Current first byte
	output.push_back(IEEE754_bytes[1]);    //Current second byte
	output.push_back(IEEE754_bytes[2]);    //Current third byte
	output.push_back(IEEE754_bytes[3]);    //Current fourth byte

	//Time
	schunk_pg70::float_to_IEEE_754(current, IEEE754_bytes);
	output.push_back(IEEE754_bytes[0]);    //Time first byte
	output.push_back(IEEE754_bytes[1]);    //Time second byte
	output.push_back(IEEE754_bytes[2]);    //Time third byte
	output.push_back(IEEE754_bytes[3]);    //Time fourth byte
   
   //Checksum calculation
	uint16_t crc = 0;

	for(size_t i = 0; i < output.size(); i++)
		crc = schunk_pg70::CRC16(crc,output[i]);

	// Add checksum to the output buffer
	output.push_back(crc & 0x00ff);
	output.push_back((crc & 0xff00) >> 8);

	// Send message to the module
	port->write(output);
	

   std::cout <<"max time " << goal->max_time << "\n";
   float start_time = ros::Time::now().toSec();

  // ros::Duration counter = start_time;
   ros::Rate r(100);
   while(ros::ok())
   {
      if (ros::Time::now().toSec() - start_time > goal->max_time)
      {
         ROS_WARN("Timer expired");
         pg70_as_result_.success = false;
         break;
      }   
      ros::spinOnce();
      if( ((act_position_ - target_pos) < 1) && ((act_position_ - target_pos) > -1 ) )
      {
         ROS_INFO("Goal reached succesfully");
         pg70_as_result_.success = true;
         break;
      }
      r.sleep();
   }

   stop_client_.call(stop_);
   pg70_as_.setSucceeded(pg70_as_result_);

   return ;
}


void pg70MoveActionServer::jointPos_cb(const sensor_msgs::JointState& msg)
{
   act_position_ = msg.position[0] * 2 * schunk_pg70::URDF_SCALE_FACTOR;
}


} // pg70_action_server



