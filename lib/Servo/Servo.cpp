#include <Arduino.h>
#include "Servo.h"
#include "CANbus.h"

uint16_t StatusWord = 0;
uint16_t ErrorCode = 0;
uint32_t MotorSpeed = 0;
uint16_t MotorCurrent = 0;
uint32_t MotorRounds = 0;
uint16_t MotorAngleRounds = 0;
uint16_t MotorAngleAngle = 0;
uint32_t SDO_StatusWord = 0;
uint32_t current_position = 0;

uint32_t Servoposition;
uint32_t targetposition;
// uint32_t servospeed = 0x00001388;// speed is 500r/min
uint32_t servospeed = 0x000061A8;// speed is 2500r/min
// uint32_t servospeed = 0x00007530;// speed is 3000r/min
uint32_t current_servo_position = 0x00000000;

void can_heartbeat_publish(HBSTATE hbState) {
  static uint8_t can_heart_beat_msg[1] = {(uint8_t)hbState};

  int ret = CANsend(HB_FUNC + ThisNodeId, can_heart_beat_msg,
                    sizeof(can_heart_beat_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("send heartbeat ");
  Serial.print((uint8_t)hbState, HEX);
  Serial.print(" return with ");
  Serial.println(ret, HEX);
#endif
}

void can_nmt_start_publish() {
  static uint8_t can_nmt_start_msg[2] = {0x01, 0x00};
  can_nmt_start_msg[1] = ServoNodeID;
  int ret = CANsend(0x0, can_nmt_start_msg,
                    sizeof(can_nmt_start_msg) / sizeof(uint8_t));
// #if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
//     SERIAL_GENERAL
  Serial.print("send nmt start [");
  Serial.print(can_nmt_start_msg[0], HEX);
  Serial.print(", ");
  Serial.print(can_nmt_start_msg[1], HEX);
  Serial.print("] return with 0x");
  Serial.println(ret, HEX);
// #endif 
}

void can_set_pp_mode() {
  static uint8_t can_pp_mode_msg[1] = {0x01};
  int ret = CANsend(RPDO2 + ServoNodeID, can_pp_mode_msg,
                    sizeof(can_pp_mode_msg) / sizeof(uint8_t));
// #if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
//     SERIAL_GENERAL
  Serial.print("send pp mode message return with ");
  Serial.println(ret, HEX);
// #endif
}

void initServo() {
  can_nmt_start_publish();
  can_set_pp_mode();
}

void setServoTarget(uint32_t position, uint32_t speed) {
  static uint8_t can_target_msg[8];
  int index = 0;
  can_target_msg[index] = position & 0xFF;
  can_target_msg[++index] = (position >> 8) & 0xFF;
  can_target_msg[++index] = (position >> 16) & 0xFF;
  can_target_msg[++index] = (position >> 24) & 0xFF;

  can_target_msg[++index] = speed & 0xFF;
  can_target_msg[++index] = (speed >> 8) & 0xFF;
  can_target_msg[++index] = (speed >> 16) & 0xFF;
  can_target_msg[++index] = (speed >> 24) & 0xFF;
  int ret = CANsend(RPDO4 + ServoNodeID, can_target_msg,
                    sizeof(can_target_msg) / sizeof(uint8_t));
  // Keep track of the commanded height in firmware
  current_servo_position = position;

#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("can send target position/speed msg return with ");
  Serial.println(ret, HEX);
#endif

  static uint8_t servo_action[2] = {0x1F, 0x00};

  ret = CANsend(RPDO1 + ServoNodeID, servo_action,
                  sizeof(servo_action) / sizeof(uint8_t));

#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    Serial.print("can send servo action msg with ");
    Serial.println(ret, HEX);
#endif
}

// void HAXServo::can_request_status_word() {
//   static uint8_t can_status_word_request_msg[2] = {0x00, 0x00};
//   int ret = CANsend(TPDO1 + ServoNodeID, can_status_word_request_msg,
//                     sizeof(can_status_word_request_msg) / sizeof(uint8_t));
// #if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
//     SERIAL_GENERAL
//   Serial.print("request status word return with ");
//   Serial.println(ret, HEX);
// #endif
// }

void runPPcommand(PPCOMMAND cmd) {
  static uint8_t can_pp_command_msg[2] = {0x00, 0x00};
  can_pp_command_msg[0] = cmd;
  int ret = ESP_OK;
  ret = CANsend(RPDO1 + ServoNodeID, can_pp_command_msg,
                sizeof(can_pp_command_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("can execute pp command return with ");
  Serial.println(ret, HEX);
#endif
}

void startPP() { runPPcommand(PPSTART); }
void pausePP() { runPPcommand(PPPAUSE); }
void resumePP() { runPPcommand(PPRESUME); }
void stopPP() { runPPcommand(PPSTOP); }


void can_change_servo_mode() {
  int ret = ESP_OK;
  if (StatusWord & 0x0040) {  // Switch on disable
    static uint8_t can_ready_to_switch_on_msg[2] = {0x06, 0x00};

    ret = CANsend(RPDO1 + ServoNodeID, can_ready_to_switch_on_msg,
                  sizeof(can_ready_to_switch_on_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    Serial.print("can send switch on disable msg return with ");
    Serial.println(ret, HEX);
#endif
  }

  if ((StatusWord & 0x006F) == 0x0021) {  // Ready to switch on
    static uint8_t can_switch_on_msg[2] = {0x07, 0x00};
    ret = CANsend(RPDO1 + ServoNodeID, can_switch_on_msg,
                  sizeof(can_switch_on_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    Serial.print("can send ready to switch on msg return with ");
    Serial.println(ret, HEX);
#endif
  }
  if ((StatusWord & 0x006F) == 0x0023) {  // Switched on
    static uint8_t can_operation_enable_msg[2] = {0x0F, 0x00};
    ret = CANsend(RPDO1 + ServoNodeID, can_operation_enable_msg,
                  sizeof(can_operation_enable_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    Serial.print("can send switch on msg return with ");
    Serial.println(ret, HEX);
#endif
  }
}

void CAN_TPDO_CALLBACK( twai_message_t can_receive_msg) {
  int16_t cmd_val;

  // int ret = twai_receive(&can_receive_msg, 0);
  // if (ret == ESP_OK) {
    switch (can_receive_msg.identifier) {
      
      case TPDO1 + ServoNodeID:
        StatusWord = ((uint16_t)(can_receive_msg.data[1]) << 8) +
                     can_receive_msg.data[0];
        // Serial.print("StatusWord:"); 
        // Serial.println(StatusWord,HEX);
        break;
      case TPDO2 + ServoNodeID:
        ErrorCode = ((uint16_t)(can_receive_msg.data[1]) << 8) +
                    can_receive_msg.data[0];
        break;
      case TPDO3 + ServoNodeID:
        MotorSpeed = ((uint32_t)(can_receive_msg.data[3]) << 24) +
                     ((uint32_t)(can_receive_msg.data[2]) << 16) +
                     ((uint32_t)(can_receive_msg.data[1]) << 8) +
                     can_receive_msg.data[0];
        MotorCurrent = ((uint32_t)(can_receive_msg.data[5]) << 8) +
                       can_receive_msg.data[4];
        break;
      case TPDO4 + ServoNodeID:
        MotorRounds = ((uint32_t)(can_receive_msg.data[3]) << 24) +
                      ((uint32_t)(can_receive_msg.data[2]) << 16) +
                      ((uint32_t)(can_receive_msg.data[1]) << 8) +
                      can_receive_msg.data[0];
        MotorAngleAngle = ((uint32_t)(can_receive_msg.data[5]) << 8) +
                          can_receive_msg.data[4];
        MotorAngleRounds = ((uint32_t)(can_receive_msg.data[7]) << 8) +
                           can_receive_msg.data[6];
        current_position = MotorAngleRounds * 0x10000 + MotorAngleAngle ;
        // Serial.print("current_position:"); 
        // Serial.println(current_position,HEX);
        break;
      case SDOADDRESS + ServoNodeID:
        SDO_StatusWord = ((uint32_t)(can_receive_msg.data[2]) << 16) + 
                         ((uint32_t)(can_receive_msg.data[1]) << 8) + 
                         can_receive_msg.data[0];
        // Serial.println(SDOADDRESS + ServoNodeID , HEX);
        Serial.print("SDO_StatusWord:");
        Serial.println(SDO_StatusWord, HEX);
        // Serial.print(", [");
        // Serial.print("0x");
        // Serial.println(MotorRounds, HEX);
        // Serial.print(" ");
        // Serial.println("]");
        break;
      default:
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL

#endif
        break;
   }
  // } else {
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    Serial.print("twai_receive return 0x");
    Serial.println(ret, HEX);
#endif
 // }
}

void setServoposition(int cmd) {
  Serial.print("cmd:");
  Serial.println(cmd);
  Servoposition = current_position;
  // Serial.print("Servoposition:"); 
  // Serial.println(Servoposition, HEX);
  if (cmd > 50)
  {
      if (Servoposition >= SERVO_POSITIVE_MIN && Servoposition < SERVO_POSITIVE_MAX)
      {
        // Servoposition = Servoposition + int(SERVO_SCALING_FACTOR  * 0x10000);
        // Servoposition = max(SERVO_POSITIVE_MIN , min(Servoposition, SERVO_POSITIVE_MAX));
        setServoTarget(SERVO_POSITIVE_MIN,servospeed); 
        // Serial.print("Servoposition:");
        // Serial.println(Servoposition, HEX);
      }
      else if ((Servoposition < SERVO_NEGTIVE_MAX) && (Servoposition >= SERVO_NEGTIVE_MIN)){
        Servoposition = Servoposition + int(SERVO_SCALING_FACTOR  * 0x10000);
        Servoposition = max(SERVO_NEGTIVE_MIN, min(Servoposition, SERVO_NEGTIVE_MAX));
        setServoTarget(Servoposition,servospeed);
        // Serial.print("Servoposition:");
        // Serial.println(Servoposition, HEX);
      }
      else if (Servoposition == SERVO_NEGTIVE_MAX ) {
        Servoposition = SERVO_POSITIVE_MIN;
        setServoTarget(Servoposition,servospeed);
        // Serial.print("Servoposition:"); 
        // Serial.println(Servoposition, HEX);
      }
  }

  if (cmd < -50) {
      if ((Servoposition > SERVO_POSITIVE_MIN) && (Servoposition <= SERVO_POSITIVE_MAX))
      {
        Servoposition = Servoposition - int(SERVO_SCALING_FACTOR  * 0x00010000);
        Servoposition = max(SERVO_POSITIVE_MIN, min(Servoposition, SERVO_POSITIVE_MAX));
        setServoTarget(Servoposition,servospeed);
        Serial.print("Servoposition:");
        Serial.println(Servoposition, HEX);
      }
      else if ( (Servoposition > SERVO_NEGTIVE_MIN) && (Servoposition <= SERVO_NEGTIVE_MAX) ){
        Servoposition = Servoposition - int(SERVO_SCALING_FACTOR  * 0x00010000);
        Servoposition = max(SERVO_NEGTIVE_MIN, min(Servoposition, SERVO_NEGTIVE_MAX));
        setServoTarget(Servoposition,servospeed);
        // Serial.print("Servoposition:");
        // Serial.println(Servoposition, HEX);
      }
      else if (Servoposition == 0x00000000 ) {
        Servoposition = SERVO_NEGTIVE_MAX;
        setServoTarget(Servoposition,servospeed);
        // Serial.print("Servoposition:"); 
        // Serial.println(Servoposition, HEX);
      }
  } 
} 

void setServooriginal(){
    //move the servo to zero position
    setServoTarget(0x000000000,servospeed);
    // Servoposition = getServoPosition();
}


void zero_return_mode(){
  int ret = ESP_OK;
  if ( (StatusWord & 0x8637) || (StatusWord & 0x9637) ) {
  // SDO: set the mode 2
  static uint8_t zeroreturn_mode_msg[8];
  int index = 0;
  zeroreturn_mode_msg[index] = 0x2F;
  zeroreturn_mode_msg[++index] = 0x98;
  zeroreturn_mode_msg[++index] = 0x60;
  zeroreturn_mode_msg[++index] = 0x00;
  zeroreturn_mode_msg[++index] = 0x02;
  zeroreturn_mode_msg[++index] = 0x00;
  zeroreturn_mode_msg[++index] = 0x00;
  zeroreturn_mode_msg[++index] = 0x00;
  ret = CANsend(0x0600+ ServoNodeID, zeroreturn_mode_msg,
                    sizeof(zeroreturn_mode_msg) / sizeof(uint8_t));
  Serial.println("set mode ready");
  

  switch (SDO_StatusWord)
    {
    case 0x609860:
    static uint8_t zeroreturn_speed_msg[8];
    index = 0;
    zeroreturn_speed_msg[index] = 0x23;
    zeroreturn_speed_msg[++index] = 0x99;
    zeroreturn_speed_msg[++index] = 0x60;
    zeroreturn_speed_msg[++index] = 0x00;
    zeroreturn_speed_msg[++index] = 0xA8;
    zeroreturn_speed_msg[++index] = 0x61;
    zeroreturn_speed_msg[++index] = 0x00;
    zeroreturn_speed_msg[++index] = 0x00;
    ret = CANsend(0x0600 + ServoNodeID, zeroreturn_speed_msg,
                    sizeof(zeroreturn_speed_msg) / sizeof(uint8_t));
    Serial.println("set speed ready");    
    break;
    
    case 0x609960:
    static uint8_t zeroreturn_start_msg[8];
    index = 0;
    zeroreturn_start_msg[index] = 0x2F;
    zeroreturn_start_msg[++index] = 0xFB;
    zeroreturn_start_msg[++index] = 0x60;
    zeroreturn_start_msg[++index] = 0x04;
    zeroreturn_start_msg[++index] = 0x01;
    zeroreturn_start_msg[++index] = 0x00;
    zeroreturn_start_msg[++index] = 0x00;
    zeroreturn_start_msg[++index] = 0x00;
    ret = CANsend(0x0600 + ServoNodeID, zeroreturn_start_msg,
                    sizeof(zeroreturn_start_msg) / sizeof(uint8_t));
    Serial.println("sart zero return");

    default:
    break;
    }
  } 
  // Servoposition = getServoPosition();
  // Serial.println(Servoposition,HEX);
}

uint32_t getCurrentPosition() {
  return Servoposition;
}

uint32_t getServoPosition() {
  return current_position;
}

