#include <Arduino.h>  // Essential

#include "CANbus.h"
#include "cylinder.h"
#include "fourTwenty.h"
#include "gripper.h"
#include "hardwarePins.h"
#include "joystick.cpp"
#include "microSD.h"
#include "peripherals.h"  // Essential
#include "relay.h"
#include "Servo.h"
#include "HardwareSerial.h"

HardwareSerial MySerial(1); // define a Serial for UART1
const int MySerialRX = 15;
const int MySerialTX = 16;


/*----------------DO NOT REMOVE----------------------------------------*/
// used by fourTwenty and microSD
SPIClass hspi = SPIClass(HSPI);
SPIClass fspi = SPIClass(FSPI);

void initSPI();
/*----------------DO NOT REMOVE----------------------------------------*/

/*---------------------PREPROCESSOR CONFIGURATION---------------------*/

// NOTE: SEE platformio.ini for PLATFORM CONFIGURATION

#define SERIAL_DEBUG_SEND_FLG false
#define SERIAL_DEBUG_AUTO_FLG false
#define SERIAL_DEBUG_MANU_FLG false
#define SERIAL_GENERAL true

enum Platform { GRAPE = 0, CELERY = 1, DANFOSS = 2 };

void setupSerial();

/*---------------------PREPROCESSOR CONFIGURATION---------------------*/

/*--------------------------GRAPE ASSIGNMENTS--------------------------*/
// PINS
#define LEFT_VER_CYL 1
#define RIGHT_VER_CYL 3
#define LEFT_HOR_CYL 2
#define RIGHT_HOR_CYL 4

// CAN ADDRESSES
#define CAN_MANUAL_ID 0x002

#define CAN_JOYSTICK_DLC 8

// OBJECT INSTANCES
CylinderHydraulic vertical_left_cyl(LEFT_VER_CYL);
CylinderHydraulic vertical_right_cyl(RIGHT_VER_CYL);

Joystick joystick(&vertical_left_cyl, &vertical_right_cyl);

// VARIABLES
uint8_t can_joystick_msg[CAN_JOYSTICK_DLC];

/*--------------------------GRAPE ASSIGNMENTS--------------------------*/

/*--------------------------CELREY ASSIGNMENTS--------------------------*/
// PINS
// #define GRIPPER_POS 1   // gripper pos
// #define GRIPPER_NEG 3   // gripper neg
// #define PIN_ROTATION 2  // rotational valve

// CAN ADDRESSES
#define CAN_CELERY_ID 0x020
#define CAN_CELERY_GRIPPER_ID 0x021
#define CAN_CELERY_STAT_ID 0x05
#define CAN_CELERY_STAT_DLC 8
#define CAN_CELERY_STAT_MS 100

// OBJECT INSTANCES
CylinderPneumatic vertical_left_cyl_cel(LEFT_VER_CYL);
CylinderPneumatic vertical_right_cyl_cel(RIGHT_VER_CYL);
CylinderPneumatic horizontal_right_cyl(RIGHT_HOR_CYL);

// Gripper right_gripper(GRIPPER_NEG, GRIPPER_POS, PIN_ROTATION,
//                       &horizontal_right_cyl, &vertical_right_cyl_cel);

// VARIABLES
int flag_celery_trigger = 0;
uint8_t can_celery_stat_msg[CAN_CELERY_STAT_DLC];

/*--------------------------CELERY ASSIGNMENTS--------------------------*/

/*------------------------------ENUMERATORS----------------------------*/
enum CAN_Gripper_frame {
  GRIPPER_TRIGGER_IDX = 0,
  GRIPPER_STATE_IDX = 1,
  GRIPPER_TIMESTAMP_IDX = 2,
  DEADMAN_COUNTER_IDX = 3
};
enum CAN_Auto_frame {
  CONF_IDX = 0,
  CMD_IDX_UPPER = 1,
  CMD_IDX_LOWER = 2,
  FRM_CNT_IDX = 3,
  CELERY_TRIG_IDX = 4,
};

enum CAN_Manual_frame {
  RIGHT_ENCODER_IDX = 0,
  LEFT_ENCODER_IDX = 1,
  BINARY_COMPONENTS_IDX = 2
};

enum CAN_Manual_component {
  RIGHT_BTT_IDX = 0,
  LEFT_BTT_IDX = 1,
  RIGHT_SW_IDX = 2,
  LEFT_SW_IDX = 3
};
/*------------------------------ENUMERATORS----------------------------*/

/*----------------------------CUSTOM DATA TYPES--------------------------*/
struct as_bytes_t {
  uint8_t lower;
  uint8_t upper;
};

union position_t {
  as_bytes_t as_byte;
  uint16_t as_word;
};
/*----------------------------CUSTOM DATA TYPES--------------------------*/

/*----------------------------COMMON--------------------------*/

// CAN ADDRESSES
#define CAN_LEFT_CAM_ID 0x014
#define CAN_RIGHT_CAM_ID 0x015

#define CAN_LOGGING_ID 0x003
#define CAN_LOGGING_DLC 6
#define CAN_LOGGING_MS 200

#define CAN_POSITION_ID 0X013
#define CAN_POSITION_DLC 8
#define CAN_POSITION_MS 50

#define CAN_MODE_ID 0x002
#define CAN_MODE_DLC 3
#define CAN_MODE_MS 50

// CAN STRUCTURE

struct CanMessage {
  void (*publishFunc)();        // Function to publish the CAN message
  unsigned long interval;       // Time interval in milliseconds
  unsigned long lastTimestamp;  // Timestamp of the last publication
};

// VARIABLES
bool left_btt_state, right_btt_state;
bool left_sw_state, right_sw_state;

static uint8_t left_frame_count, right_frame_count;

uint8_t can_log_msg[CAN_LOGGING_DLC];
uint8_t can_position_msg[CAN_POSITION_DLC];
uint8_t can_mode_msg[CAN_MODE_DLC];

inline bool get_bit(uint8_t n, uint8_t k) { return (n >> k) & 1; }
/*----------------------------COMMON--------------------------*/

void publishIfTimeElapsed(CanMessage &canMessage) {
  if ((millis() - canMessage.lastTimestamp) > canMessage.interval) {
    canMessage.publishFunc();
    canMessage.lastTimestamp = millis();
  }
}

void can_log_publish() {
  int index = 0;
  can_log_msg[index] = vertical_left_cyl.getLastConf();
  can_log_msg[++index] = vertical_left_cyl.getLastCmd();
  can_log_msg[++index] = vertical_left_cyl.getLastPosition();
  can_log_msg[++index] = vertical_right_cyl.getLastConf();
  can_log_msg[++index] = vertical_right_cyl.getLastCmd();
  can_log_msg[++index] = vertical_right_cyl.getLastPosition();

  CANsend(CAN_LOGGING_ID, can_log_msg, sizeof(can_log_msg) / sizeof(uint8_t));

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display(&can_log_msg);
#endif
}

void can_position_publish() {
  union position_t left_position;
  union position_t right_position;
  union position_t left_cmd;
  union position_t right_cmd;
  left_cmd.as_word = uint16_t(vertical_left_cyl.getLastCmd());
  right_cmd.as_word = uint16_t(vertical_right_cyl.getLastCmd());
  left_position.as_word = uint16_t(0);
  right_position.as_word = uint16_t(0);

  int index = 0;
  can_position_msg[index] = getCurrentPosition() & 0xFF;
  can_position_msg[++index] = (getCurrentPosition() >> 8) & 0xFF;
  can_position_msg[++index] = (getCurrentPosition() >> 16) & 0xFF;
  can_position_msg[++index] = (getCurrentPosition() >> 24) & 0xFF;
  // can_position_msg[++index] = left_cmd.as_byte.upper;
  // can_position_msg[++index] = left_cmd.as_byte.lower;
  // can_position_msg[++index] = right_position.as_byte.upper;
  // can_position_msg[++index] = right_position.as_byte.lower;
  // can_position_msg[++index] = right_cmd.as_byte.upper;
  // can_position_msg[++index] = right_cmd.as_byte.lower;

  CANsend(CAN_POSITION_ID, can_position_msg, sizeof(can_position_msg) / sizeof(uint8_t));

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display(&can_position_msg);
#endif
}

void can_celery_stat_publish() {
  // int gripper_state = right_gripper.getGripperState();
  // unsigned long gripper_timestamp = right_gripper.getSeqTimestamp();
  // int deadman_counter = joystick.getDeadmanClickCount();

  // gripper_state = right_gripper.getGripperState();
  // gripper_timestamp = right_gripper.getSeqTimestamp();

  // int index = 0;
  // can_celery_stat_msg[index] = vertical_right_cyl.getLastPosition();
  // can_celery_stat_msg[++index] = gripper_state;
  // can_celery_stat_msg[++index] = (gripper_timestamp >> 24) & 0xFF;
  // can_celery_stat_msg[++index] = (gripper_timestamp >> 16) & 0xFF;
  // can_celery_stat_msg[++index] = (gripper_timestamp >> 8) & 0xFF;
  // can_celery_stat_msg[++index] = gripper_timestamp & 0xFF;
  // can_celery_stat_msg[++index] = deadman_counter;
  can_celery_stat_msg[GRIPPER_STATE_IDX] = gripper_motor.seqRet;
  can_celery_stat_msg[GRIPPER_STATE_IDX + 1] = gripper_motor.getGripperState();
  CANsend(CAN_CELERY_GRIPPER_ID, can_celery_stat_msg,sizeof(can_celery_stat_msg) / sizeof(uint8_t));

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display(&can_celery_stat_msg);
#endif
}

void can_mode_publish() {
  bool left_auto_state = joystick.isLeftAutomaticMode();
  bool right_auto_state = joystick.isRightAutomaticMode();

  can_mode_msg[RIGHT_ENCODER_IDX] = 0x00;
  can_mode_msg[LEFT_ENCODER_IDX] = 0x00;
  can_mode_msg[BINARY_COMPONENTS_IDX] = 0x00 |
                                        (right_auto_state << RIGHT_SW_IDX) |
                                        (left_auto_state << LEFT_SW_IDX);

  CANsend(CAN_MODE_ID, can_mode_msg,sizeof(can_mode_msg) / sizeof(uint8_t));

#if SERIAL_DEBUG_SEND_FLG
  serial_publish_display();
#endif
}

void can_request_status_word() {
  static uint8_t can_status_word_request_msg[2] = {0x00, 0x00};
  int ret = CANsend(TPDO1 + ServoNodeID, can_status_word_request_msg,
                    sizeof(can_status_word_request_msg) / sizeof(uint8_t));
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.print("request status word return with ");
  Serial.println(ret, HEX);
#endif
}

#if SERIAL_DEBUG_MANU_FLG
void serial_manual_display(mbed::CANMessage *msg) {
  Serial.print("left_vert_cyl_pos: ");
  Serial.print(vertical_left_cyl.getLastPosition());
  Serial.print(", right_vert_cyl_pos: ");
  Serial.print(vertical_right_cyl.getLastPosition());
  Serial.print(", left_switch: ");
  Serial.print(left_sw_state);
  Serial.print(", right_switch: ");
  Serial.print(right_sw_state);
  Serial.print(", left_btt: ");
  Serial.print(left_btt_state);
  Serial.print(", right_btt: ");
  Serial.print(right_btt_state);
  Serial.print(", left_encoder: ");
  Serial.print(msg->data[LEFT_ENCODER_IDX]);
  Serial.print(", right_encoder: ");
  Serial.print(msg->data[RIGHT_ENCODER_IDX]);
  Serial.print(", vertical left position: ");
  Serial.print(vertical_left_position_ss.get_raw_position());
  Serial.print(", vertical right position: ");
  Serial.print(vertical_right_position_ss.get_raw_position());
  Serial.print(", bin: ");
  Serial.print(msg->data[BINARY_COMPONENTS_IDX], BIN);

  Serial.print("\n");
}
#endif

#if SERIAL_DEBUG_AUTO_FLG
void serial_auto_display(mbed::CANMessage *msg) {
  Serial.print("Auto!\t ID: ");
  Serial.print(msg->id);
  Serial.print(":");
  for (int i = 0; i < msg->len; i++) {
    Serial.print("\t");
    Serial.print(msg->data[i]);
  }
  Serial.print("\n\r");
}
#endif

#if SERIAL_DEBUG_SEND_FLG
void serial_publish_display(mbed::CANMessage *msg) {
  Serial.print("Publish! can_id: ");
  Serial.print(msg->id, HEX);  // print ID
  Serial.print("\t, length: ");
  Serial.print(msg->len, HEX);  // print DLC
  Serial.print(", \t");

  for (int i = 0; i < msg->len; i++) {  // print the data
    Serial.print(msg->data[i]);
    Serial.print("\t");
  }
  Serial.print("\r\n");
}
#endif

void CANCallback(twai_message_t can_receive_msg) {
  
  int16_t cmd_val;
  uint32_t cmd;
  // Serial.println(can_receive_msg.identifier);
  // int ret = twai_receive(&can_receive_msg, 0);
  // if (ret == ESP_OK) {
    // Serial.println(can_receive_msg.identifier);
    switch (can_receive_msg.identifier) {
      case CAN_JOYSTICK_ID_DEC:
        joystick.parseCanJoystickMessage(can_receive_msg.data);
        joystick.controlCylinders();
        // joystick.printJoystickData();

        break;
      case CAN_LEFT_CAM_ID:
        
        cmd_val = can_receive_msg.data[CMD_IDX_UPPER] << 8 |
                  can_receive_msg.data[CMD_IDX_LOWER];

        cmd = 65535 * cmd_val / 10;
       
        // Serial.println("Left cmd val raw: " + String(cmd_val));
          
        // 
        // Serial.println("Left cmd val: " + String(cmd));
        // convert cmd_val to bytes
        
        // joystick.setLeftCmdVal(left_cmd_val);
        // joystick.controlCylinders();
        // setServoposition(left_cmd_val);
        // uint32_t servospeed = 0x000061A8;// speed is 2500r/min
        setServoTarget(cmd, 0x000061A8);
        left_frame_count = can_receive_msg.data[FRM_CNT_IDX];

#if SERIAL_DEBUG_AUTO_FLG
        serial_auto_display(&can_receive_msg);
#endif
        break;
      case CAN_RIGHT_CAM_ID:
        float right_cmd_val;
        cmd_val = can_receive_msg.data[CMD_IDX_UPPER] << 8 |
                  can_receive_msg.data[CMD_IDX_LOWER];
        right_cmd_val = static_cast<float>(cmd_val);
        joystick.setRightCmdVal(right_cmd_val);
        joystick.controlCylinders();
        right_frame_count = can_receive_msg.data[FRM_CNT_IDX];
      case CAN_CELERY_GRIPPER_ID:
        
        flag_celery_trigger = can_receive_msg.data[GRIPPER_TRIGGER_IDX];
        if (flag_celery_trigger == 1) {
            //Serial.println("Gripper triggered");
            gripper_motor.seq(true);
        }
      
        break;


#if SERIAL_DEBUG_AUTO_FLG
        serial_auto_display(&can_receive_msg);
#endif
        break;
#if SERIAL_DEBUG_AUTO_FLG
        serial_auto_display(&can_receive_msg);
#endif
        break;
      default:
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
        // Serial.println(can_receive_msg.identifier);
#endif
        break;
    }
  // } else {
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
    // Serial.print("twai_receive return ");
    // Serial.println(ret, HEX);
#endif
  //}
}

// For publishing can messages

CanMessage canMessages[] = {{can_log_publish, CAN_LOGGING_MS, 0},
                            {can_position_publish, CAN_POSITION_MS, 0},
                            {can_celery_stat_publish, CAN_CELERY_STAT_MS, 0},
                            {can_mode_publish, CAN_MODE_MS, 0}};

CanMessage status_word_ = {can_request_status_word, CAN_LOGGING_MS, 0};

void setup() {
  setupSerial();
  // initialize the Serial to the pins
    MySerial.begin(115200, SERIAL_8N1, MySerialRX, MySerialTX);
  /*----------------------INITIALIZE SPI
   * PERIPHERAL------------------------------*/
  /*----------------------DO NOT
   * REMOVE------------------------------------------*/
  initSPI();

  /*----------------------DO NOT
   * REMOVE------------------------------------------*/
  /*----------------------INITIALIZE SPI
   * PERIPHERAL------------------------------*/

  initFourTwenty();
  initRelays();
  CANinit();

  left_frame_count = 0;
  right_frame_count = 0;

  left_btt_state = false;
  right_btt_state = false;

  initServo();
  // Interrupt for CAN bus

}


void loop() {
  twai_message_t can_receive_msg;
  int ret = twai_receive(&can_receive_msg, 0);
  if (ret == ESP_OK) {
    CAN_TPDO_CALLBACK(can_receive_msg);
    can_change_servo_mode();
    CANCallback(can_receive_msg);
  }
  publishIfTimeElapsed(canMessages[1]);
  publishIfTimeElapsed(canMessages[2]);

  gripper_motor.seq();
#if SERIAL_DEBUG_SEND_FLG
  Serial.print("Left cylinder spool position: ");
  Serial.print(vertical_left_cyl.getLastPosition());
  Serial.print("; Right cylinder spool position: ");
  Serial.println(vertical_right_cyl.getLastPosition());
#endif

  // MySerial.print(11);

}

void initSPI() {
  // used by fourTwenty and microSD
  hspi.begin(SPI_2_CLOCK, 17, SPI_2_MOSI);  //  CLK=18  MISO=19  MOSI=23
  fspi.begin(SPI_3_CLOCK, SPI_3_MISO,
             SPI_3_MOSI);  //  CLK=18  MISO=19  MOSI=23
}

void setupSerial() {
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL
  Serial.begin(115200);
  Serial.println("BEAGLE CONTROL BOARD v2.2.0");
  Serial.println("Date: 2024/02/15");
  Serial.println("LEFT cam ID: 0x014, RIGHT cam ID: 0x015");
  Serial.println("Debug ID: 0x003, Position sensors ID: 0x004");
#endif
}
