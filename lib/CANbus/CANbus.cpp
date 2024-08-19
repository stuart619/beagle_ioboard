#include "CANbus.h"

#define DEBUG_CAN_INIT false
void CANinit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN,
      TWAI_MODE_NORMAL);  // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or
                          // TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  int ret = twai_driver_install(&g_config, &t_config, &f_config);
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL or DEBUG_CAN_INIT
  Serial.print("twai_driver_install return ");
  Serial.println(ret, HEX);
#endif
  ret = twai_start();
#if SERIAL_DEBUG_SEND_FLG or SERIAL_DEBUG_AUTO_FLG or SERIAL_DEBUG_MANU_FLG or \
    SERIAL_GENERAL or DEBUG_CAN_INIT
  Serial.print("twai_start return ");
  Serial.println(ret, HEX);
#endif
}

int CANsend(uint32_t identifier, uint8_t dataArray[], uint8_t datalen) {
  twai_message_t message;
  message.identifier = identifier;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = datalen;

  for (int i = 0; i < datalen; i++) {
    message.data[i] = dataArray[i];
  }

  return twai_transmit(&message, 0);
}

void CANrecv(void) {
  twai_message_t message;
  if (twai_receive(&message, 0) != ESP_OK) {
    return;
  }

  Serial.println("identifier - " + String(message.identifier));
  Serial.println("data1: " + String(uint8_t(message.data[0])));
  Serial.println("data2: " + String(uint8_t(message.data[1])));
  Serial.println("data3: " + String(uint8_t(message.data[2])));
  Serial.println("data4: " + String(uint8_t(message.data[3])));
  Serial.println("data5: " + String(uint8_t(message.data[4])));
  Serial.println("data6: " + String(uint8_t(message.data[5])));
  Serial.println("data7: " + String(uint8_t(message.data[6])));
  Serial.println("data8: " + String(uint8_t(message.data[7])));
}