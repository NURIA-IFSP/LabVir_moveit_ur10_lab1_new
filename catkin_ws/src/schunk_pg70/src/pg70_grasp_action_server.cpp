/* 
* Action server interface to the grasp ction of the schunk pg70
*/


#include <pg70_action_server/pg70_grasp_action_server.h>
#include <pg70_rs232_control.h>

namespace pg70_action_server
{
pg70GraspActionServer::pg70GraspActionServer(std::string name) :
   pg70_as_(nh_, name, boost::bind(&pg70GraspActionServer::execute_cb, this, _1), false),
   action_name(name)
{

   ROS_INFO("gripper Action client initialization");

   nh_.getParam("/schunk_pg70/Grasp/portname", port_name_);
   nh_.getParam("/schunk_pg70/Grasp/baudrate", baudrate_);
   nh_.getParam("/schunk_pg70/Grasp/gripper_id", gripper_id_);

   joint_sub_ = nh_.subscribe("/schunk_pg70/joint_states", 1, &pg70GraspActionServer::jointPos_cb, this);

   /*
   ROS_INFO("Set pvac service Connection: Waiting");  
   set_pvac_ = nh_.serviceClient<schunk_pg70::set_pvac>("/schunk_pg70/set_pvac");
   set_pvac_.waitForExistence();
   ROS_INFO("Set pvac service Connection: OK");
   */

   ROS_INFO("Stop service Connection: Waiting");  
   stop_client_ = nh_.serviceClient<schunk_pg70::stop>("/schunk_pg70/stop");
   stop_client_.waitForExistence();
   ROS_INFO("Stop service Connection: OK");

   pg70_as_.start();
   ROS_INFO("Gripper Action Client: OK");

}

pg70GraspActionServer::~pg70GraspActionServer()
{
  com_port_->close();      //Close port
  delete com_port_;        //delete object
}


void pg70GraspActionServer::execute_cb(const schunk_pg70::GraspGoalConstPtr& goal)
{
  

   /*
   act_position_ = goal->position;
   */

   float target_pos = goal->position*1000;
   float target_vel = goal->velocity*1000;
   float target_acc = goal->acceleration*1000;
   float target_current = goal->current;

	ROS_INFO_STREAM("PG70: Moving from: " << act_position_ << " [mm] to " << goal->position << " [mm]");
	ROS_INFO("PG70: Data used: %f [mm/s], %f [mm/s2], %f [A]", goal->velocity, goal->acceleration, goal->current);
	

   // aggiungere Controllo correttezza della richiesta

   if(schunk_pg70::checkRequestLimit(target_pos, target_vel, target_acc, target_current))
   {      
      ROS_WARN("Goal not accepted");
      pg70_as_result_.success = false;
      pg70_as_.setSucceeded(pg70_as_result_);
      return;
   }


	std::vector<uint8_t> output;
	output.push_back(0x05);                //message from master to module
	output.push_back(gripper_id_);         //module id
	output.push_back(0x11);                //11-Len
	output.push_back(0xB0);                //Command "mov pos"

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

   float current_f = (float)target_current / 1000.0;
   schunk_pg70::float_to_IEEE_754(current_f, IEEE754_bytes);
	output.push_back(IEEE754_bytes[0]);    //Current first byte
	output.push_back(IEEE754_bytes[1]);    //Current second byte
	output.push_back(IEEE754_bytes[2]);    //Current third byte
	output.push_back(IEEE754_bytes[3]);    //Current fourth byte
   
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

/*
void pg70GraspActionServer::schunk_pg70::float_to_IEEE_754(float position, unsigned int *output_array)
{
	unsigned char *p_byte = (unsigned char*)(&position);

	for(size_t i = 0; i < sizeof(float); i++)
		output_array[i] = (static_cast<unsigned int>(p_byte[i]));
}

uint16_t pg70GraspActionServer::CRC16(uint16_t crc, uint16_t data)
{
	const uint16_t tbl[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
	};

	return(((crc & 0xFF00) >> 8)  ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)]);
}
*/

void pg70GraspActionServer::jointPos_cb(const sensor_msgs::JointState& msg)
{
   act_position_ = msg.position[0] * 2 * schunk_pg70::URDF_SCALE_FACTOR;
}


} // pg70_action_server



