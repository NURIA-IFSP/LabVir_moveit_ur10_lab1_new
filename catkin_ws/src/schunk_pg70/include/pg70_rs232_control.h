

#ifndef PG70_RS232_CONTROL_H
#define PG70_RS232_CONTROL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

//Service headers
#include <schunk_pg70/reference.h>
#include <schunk_pg70/set_position.h>
#include <schunk_pg70/set_pvac.h>
#include <schunk_pg70/move_time.h>
#include <schunk_pg70/move_time_loop.h>
#include <schunk_pg70/get_error.h>
#include <schunk_pg70/get_position.h>
#include <schunk_pg70/get_current.h>
#include <schunk_pg70/acknowledge_error.h>
#include <schunk_pg70/stop.h>

#include <pg70_common.h>

/** \brief Control of Schunk PG70 gripper */
namespace schunk_pg70
{

/** \brief Client class for PG70 serial control */  
class PG70_serial
{
public:
  
  /** \brief Construct a client for PG70 serial control*/    
  PG70_serial(ros::NodeHandle *nh);
  
  ~PG70_serial();
  
  /** \brief Reference service callback */
  bool referenceCallback(schunk_pg70::reference::Request &req,
                         schunk_pg70::reference::Response &res);
  
  /** \brief GetError service callback */
  bool getErrorCallback(schunk_pg70::get_error::Request &req,
                        schunk_pg70::get_error::Response &res);
  
  /** \brief GetPosition service callback */
  bool getPositionCallback(schunk_pg70::get_position::Request &req,
                           schunk_pg70::get_position::Response &res);

  /** \brief GetStatus service callback */
  bool getCurrentCallback(schunk_pg70::get_current::Request &req,
                           schunk_pg70::get_current::Response &res);

  /** \brief SetPosition service callback */
  bool setPositionCallback(schunk_pg70::set_position::Request &req,
                           schunk_pg70::set_position::Response &res);

  /** \brief SetPVAC service callback */
  bool setPVACCallback(schunk_pg70::set_pvac::Request &req,
                           schunk_pg70::set_pvac::Response &res);

    /** \brief movePosTime service callback */
  bool movePosTimeCallback(schunk_pg70::move_time::Request &req,
                           schunk_pg70::move_time::Response &res);

    /** \brief movePosTimeLoop service callback */
  bool movePosTimeLoopCallback(schunk_pg70::move_time_loop::Request &req,
                           schunk_pg70::move_time_loop::Response &res);
  
   /** \brief AcknowledgeError service callback */
  bool acknowledgeErrorCallback(schunk_pg70::acknowledge_error::Request &req,
                                schunk_pg70::acknowledge_error::Response &res);
  
  /** \brief Stop service callback */
  bool stopCallback(schunk_pg70::stop::Request &req,
                    schunk_pg70::stop::Response &res);
  
   /** \brief Timer callback to read serial input buffer periodically */
  void timerCallback(const ros::TimerEvent &event);  
  
  /** \brief Gripper joint state publisher */
  ros::Publisher joint_pub;
  
 
   
private:

  /** \brief Send CMD REFERENCE(0x92) command to the gripper */
  void reference(serial::Serial *port);
   
  /** \brief Read actual error by GET STATE(0x95) command */
  uint8_t getError(serial::Serial *port);
    
  /** \brief Read actual position by GET_STATE(0x95) command */
  float getPosition(serial::Serial *port);

  /** \brief Read actual position by GET_STATE(0x95) command */
  float getCurrent(serial::Serial *port);
  
  /** \brief Send MOV_POS(0x80) command to the gripper */
  void setPosition(serial::Serial *port, int goal_position, int velocity, int acceleration);

  /** \brief Send MOV_POS(0xB0) command to the gripper */
  void setPVAC(serial::Serial *port, int goal_position, int velocity, int acceleration, int current);

   /** \brief Send MOV_POS_TIME(0xB1) command to the gripper */
  void movePosTime(serial::Serial *port, int goal_position, int velocity, int acceleration, int current, int time);

   /** \brief Send MOV_POS_TIME_LOOP(0xBB) command to the gripper */
  void movePosTimeLoop(serial::Serial *port, int goal_position, int velocity, int acceleration, int current, int time);
   
  /** \brief Send CMD_ACK(0x8b) command to the gripper */
  void acknowledgeError(serial::Serial *port);
    
  /** \brief Send CMD_STOP(0x91) to stop moving gripper */
  void stop(serial::Serial *port);
    
  /** \brief Set periodic position reading by GET_STATE(0x95) command */
  void getPeriodicPositionUpdate(serial::Serial *port, float update_frequency);
  
  
  // Move to pg70_common.h

  /** \brief Function to determine checksum*/
  //uint16_t CRC16(uint16_t crc, uint16_t data);   
     
  /** \brief Conversion from 4 bytes to float*/
  //float IEEE_754_to_float(uint8_t *raw);
  
  /** \brief Conversion from float to 4 bytes*/
  //void float_to_IEEE_754(float position, unsigned int *output_array);

  //Launch params
  int gripper_id_;
  std::string port_name_;
  int baudrate_;  

  //Gripper state variables
  float act_position_;
  float act_velocity_;
  float act_current_;
  uint8_t pg70_error_;
  sensor_msgs::JointState pg70_joint_state_; 

  //Serial variables
  serial::Serial *com_port_;
  
  //Consts
    
};  //PG70_serial
}   //schunk_pg70

#endif //PG70_RS232_CONTROL_H
