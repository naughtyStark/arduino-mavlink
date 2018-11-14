#include <mavlink.h>        // Mavlink interface

// Message #0  HEARTHBEAT 
uint8_t    ap_type = MAV_TYPE_GENERIC;
uint8_t    ap_autopilot = MAV_AUTOPILOT_GENERIC;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 1;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

//<Message #76 COMMAND_LONG - using the mavlink_msg_command_long_pack() function
uint8_t target_system=1;// - not really sure why this has to be at a valu of 1
uint8_t target_component=0;
uint16_t CMD_LONG_command=0;
uint8_t  CMD_LONG_confirmation=0;
float CMD_LONG_param1=0;
float CMD_LONG_param2=0;
float CMD_LONG_param3=0;
float CMD_LONG_param4=0;
float CMD_LONG_param5=0;
float CMD_LONG_param6=0;
float CMD_LONG_param7=0;

//Mavlink COmms
unsigned long hb_timer;
uint8_t MavLink_Connection_Status;//no communication initialy

//Standard requireemnts for packets to have
uint8_t system_id = 255; 
uint8_t component_id = 200;

void setup() 
{
  
  Serial1.begin(57600);
  
  pinMode(CAP_TOUCH_PIN,INPUT);
  //Mavlink COMMS
  MavLink_Connection_Status = 0;
  hb_timer=millis();
  Send_Heart_Beat();
  Command_long_ARM(); //disarm bhi kar dena bklodo. warna tata bye bye ho jayega
}
//basic loop:
void loop() 
{ 
  int CAP_TOUCH_STATE = digitalRead(CAP_TOUCH_PIN);
  send_Heart_Beat();
  delay(1000);
  comm_receive();
}

void comm_receive() { 
  mavlink_message_t msg; 
  mavlink_status_t status;
  
  //receive data over serial 
  while(Serial1.available() > 0) { 
    uint8_t c = Serial1.read();
    
    //try to get a new message 
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    { 
      // Handle message
      switch(msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT://do nothing
      break;
      case MAVLINK_MSG_ID_SET_MODE: 
      {
      // set mode
      }
      break;
      /*
      case MAVLINK_MSG_ID_ACTION:
      // EXECUTE ACTION
      break;
      */
      default:
      //Do nothing
      break;
      }
    } 
    // And get the next one
  }
}

void Command_position(long lat,long lon, long alt,int16_t yaw)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_global_position_setpoint_int_pack(system_id, component_id, &msg, coordinate_frame, lat, lon, alt, yaw);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void Send_Heart_Beat(){
// Define the system type (see mavlink_types.h for list of possible types) 
  int system_type = MAV_TYPE_QUADROTOR;
  int autopilot_type = MAV_AUTOPILOT_GENERIC;
  
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, system_type, autopilot_type,ap_base_mode,ap_custom_mode,ap_system_status);
  
  // Copy the message to send buffer 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes) 
  Serial1.write(buf, len);
}

void Command_long_ARM()
{

// Define the system type (see mavlink_types.h for list of possible types) 
  
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  CMD_LONG_param1 = 1;// this variable already exists.
  CMD_LONG_command=MAV_CMD_COMPONENT_ARM_DISARM;//this is the type of command i.e. disamr/arm
  // Pack the message
  //uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
  mavlink_msg_command_long_pack(system_id,component_id,&msg,target_system,target_component,CMD_LONG_command,CMD_LONG_confirmation,CMD_LONG_param1,CMD_LONG_param2,CMD_LONG_param3,CMD_LONG_param4,CMD_LONG_param5,CMD_LONG_param6,CMD_LONG_param7);
  // Copy the message to send buffer 

      /*
      mavlink_command_long_t arm_command_msg;
      mavlink_message_t arm_msg;

      arm_command_msg.command = 40;
      arm_command_msg.target_system = 1;
      arm_command_msg.target_component = 0;
      arm_command_msg.confirmation = 0;
      arm_command_msg.param1 = 1;
      arm_command_msg.param2 = 0;
      arm_command_msg.param3 = 0;
      arm_command_msg.param4 = 0;
      arm_command_msg.param5 = 0;
      arm_command_msg.param6 = 0;
      arm_command_msg.param7 = 0;

    mavlink_msg_command_long_encode(system_id, component_id, &arm_msg, &arm_command_msg);
    */   
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  Serial1.write(buf, len);
}
void Command_long_Disarm()
{
// Define the system type (see mavlink_types.h for list of possible types)  
  // Initialize the required buffers 
  mavlink_message_t msg; 
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  CMD_LONG_param1 = 0;// to arm the damn thing
  CMD_LONG_command=MAV_CMD_COMPONENT_ARM_DISARM;//this is the type of command i.e. disamr/arm
  // Pack the message
  //uint16_t mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
  mavlink_msg_command_long_pack(system_id,component_id,&msg,target_system,target_component,CMD_LONG_command,CMD_LONG_confirmation,CMD_LONG_param1,CMD_LONG_param2,CMD_LONG_param3,CMD_LONG_param4,CMD_LONG_param5,CMD_LONG_param6,CMD_LONG_param7);
  // Copy the message to send buffer 

        //int fd = serial_fd;
        //char buf[300];
  /*
      mavlink_command_long_t arm_command_msg;
      mavlink_message_t arm_msg;

       arm_command_msg.command = 40;
      arm_command_msg.target_system = 1;
      arm_command_msg.target_component = 0;
      arm_command_msg.confirmation = 0;
      arm_command_msg.param1 = 0;
      arm_command_msg.param2 = 0;
      arm_command_msg.param3 = 0;
      arm_command_msg.param4 = 0;
      arm_command_msg.param5 = 0;
      arm_command_msg.param6 = 0;
      arm_command_msg.param7 = 0;

    mavlink_msg_command_long_encode(system_id, component_id, &arm_msg, &arm_command_msg);
    */   
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes) 
  Serial1.write(buf, len);   
}



