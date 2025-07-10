#include <Arduino.h> 
#include <driver/rmt.h>
#include <Bluepad32.h>
#include <algorithm>
#include <iostream>

/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////DSHOT600//////////////////////////////////////////

#define MOTOR_PIN_4 GPIO_NUM_4
#define MOTOR_PIN_2 GPIO_NUM_2
#define RMT_CH_4 RMT_CHANNEL_4
#define RMT_CH_2 RMT_CHANNEL_2

#define MOTOR_PIN_1 GPIO_NUM_14
#define MOTOR_PIN_3 GPIO_NUM_12
#define RMT_CH_1 RMT_CHANNEL_1
#define RMT_CH_3 RMT_CHANNEL_3

// DShot600 timing calculations
#define T0H_NS 625
#define T0L_NS 1041  
#define T1H_NS 1041
#define T1L_NS 625

// Convert to RMT ticks (12.5ns per tick at 80MHz)
#define T0H (T0H_NS / 12.5)  // 50 ticks
#define T0L (T0L_NS / 12.5)  // 83 ticks
#define T1H (T1H_NS / 12.5)  // 83 ticks  
#define T1L (T1L_NS / 12.5)  // 50 ticks

// Known working values from your testing
#define NEUTRAL_THROTTLE 0
#define MOTOR_RANGE 1500 // Motor goes up to 48 - 2048
#define CONTROLLER_RANGE 1024 // -512, 512

rmt_item32_t dshot_bit0;
rmt_item32_t dshot_bit1;
rmt_item32_t dshot_pause;
// Alternative version with more detailed output
void printPacketBinary(uint16_t packet, uint16_t throttle) {
  Serial.printf("Throttle value:  %u (0x%04X)\n", throttle, throttle);
  Serial.printf("Packet value: %u (0x%04X)\n", packet, packet);
  Serial.print("Binary: ");

  // Print with grouping every 4 bits
  for(int i = 15; i >= 0; i--) {
    Serial.print((packet >> i) & 1);
    if(i > 0 && i % 4 == 0) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

bool sendDshotReliable(uint16_t throttle) {
  uint16_t packet = (throttle << 1);
  uint16_t checksum = checksum = ((packet >> 0) ^ (packet >> 4) ^ (packet >> 8)) & 0xF;
  packet = (packet << 4) | (checksum & 0xF);

  rmt_item32_t items[17];

  for(int i = 15; i >= 0; i--) {
    if (packet & (1 << i)) {
      items[15-i] = dshot_bit1;
    } else {
      items[15-i] = dshot_bit0;
    }
  }

  items[16] = dshot_pause;
  
  esp_err_t err1 = rmt_write_items(RMT_CH_1, items, 17, false);
  esp_err_t err2 = rmt_write_items(RMT_CH_2, items, 17, false);
  esp_err_t err3 = rmt_write_items(RMT_CH_3, items, 17, false);
  esp_err_t err4 = rmt_write_items(RMT_CH_4, items, 17, false);
  
  if (err1 != ESP_OK || err2 != ESP_OK || err3 != ESP_OK || err4 != ESP_OK) {
    Serial.printf("RMT send error - CH1: %d, CH2: %d\n", err1, err2);
    return false;
  }

  // Wait for both channels to complete
  err1 = rmt_wait_tx_done(RMT_CH_1, pdMS_TO_TICKS(1));
  err2 = rmt_wait_tx_done(RMT_CH_2, pdMS_TO_TICKS(1));
  err3 = rmt_wait_tx_done(RMT_CH_3, pdMS_TO_TICKS(1));
  err4 = rmt_wait_tx_done(RMT_CH_4, pdMS_TO_TICKS(1));
  
  return (err1 == ESP_OK && err2 == ESP_OK && err3 == ESP_OK && err4 == ESP_OK);
}

/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////CONTROLLER////////////////////////////////////////
ControllerPtr controller = nullptr;
void onConnectedController(ControllerPtr ctl) {
  controller = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
  controller = nullptr;
}

void processController() {
  bool dataUpdated = BP32.update();
  
  if (dataUpdated && controller && controller->isConnected() && controller->hasData()) {
    if (controller->isGamepad()) {
      Serial.println(controller->axisX());
      Serial.println(controller->axisY());
    }
  }
}

uint16_t mapInputToThrottle(int32_t input) {
    if (input < -512) input = -512;
    if (input > 512) input = 512;
    return (uint16_t)((input + 512) * MOTOR_RANGE / CONTROLLER_RANGE);
}
//////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////MAIN/////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("DShot600 Motor Control - Specific Values Mode");
  
  //////////////////////////////////////DSHOT600/////////////////////////////////
  dshot_bit0.level0 = 1;
  dshot_bit0.duration0 = T0H;
  dshot_bit0.level1 = 0;
  dshot_bit0.duration1 = T0L;

  dshot_bit1.level0 = 1;
  dshot_bit1.duration0 = T1H;
  dshot_bit1.level1 = 0;
  dshot_bit1.duration1 = T1L;

  dshot_pause.level0 = 0;
  dshot_pause.duration0 = 2000; // 25μs pause
  dshot_pause.level1 = 0;
  dshot_pause.duration1 = 0;

  // Configure RMT
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(MOTOR_PIN_1, RMT_CH_1);
  config.clk_div = 1;  // 80MHz
  config.mem_block_num = 2;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  esp_err_t err = rmt_config(&config);
  if (err != ESP_OK) {
    Serial.printf("RMT config failed: %d\n", err);
    return;
  }

  err = rmt_driver_install(RMT_CH_1, 0, 0);
  if (err != ESP_OK) {
    Serial.printf("RMT install failed: %d\n", err);
    return;
  }

  // Configure RMT
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(MOTOR_PIN_2, RMT_CH_2);
  config.clk_div = 1;  // 80MHz
  config.mem_block_num = 2;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  esp_err_t err2 = rmt_config(&config);
  if (err2 != ESP_OK) {
    Serial.printf("RMT config failed: %d\n", err2);
    return;
  }

  err2 = rmt_driver_install(RMT_CH_2, 0, 0);
  if (err2 != ESP_OK) {
    Serial.printf("RMT install failed: %d\n", err2);
    return;
  }

  // Configure RMT
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(MOTOR_PIN_3, RMT_CH_3);
  config.clk_div = 1;  // 80MHz
  config.mem_block_num = 2;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  esp_err_t err3 = rmt_config(&config);
  if (err3 != ESP_OK) {
    Serial.printf("RMT config failed: %d\n", err3);
    return;
  }

  err3 = rmt_driver_install(RMT_CH_3, 0, 0);
  if (err3 != ESP_OK) {
    Serial.printf("RMT install failed: %d\n", err);
    return;
  }

  // Configure RMT for second channel (pin 2)
  rmt_config_t config2 = RMT_DEFAULT_CONFIG_TX(MOTOR_PIN_4, RMT_CH_4);
  config2.clk_div = 1;  // 80MHz
  config2.mem_block_num = 2;
  config2.tx_config.idle_output_en = true;
  config2.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  esp_err_t err4 = rmt_config(&config2);
  if (err4 != ESP_OK) {
    Serial.printf("RMT config failed for CH2: %d\n", err4);
    return;
  }

  err4 = rmt_driver_install(RMT_CH_4, 0, 0);
  if (err4 != ESP_OK) {
    Serial.printf("RMT install failed for CH2: %d\n", err4);
    return;
  }

  delay(3000);
  Serial.println("Arming ESC...");

  // Arming sequence
  for(int i = 0; i < 200; i++) {
    sendDshotReliable(NEUTRAL_THROTTLE);
    delayMicroseconds(1000);
  }

  Serial.println("ESC Armed!");

  //////////////////////////////////////CONTROLLER/////////////////////////////////
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  
  if (dataUpdated && controller && controller->isConnected() && controller->hasData()) {
    int32_t axisX = controller->axisX();
    int32_t axisY = controller->axisY();

    
      sendDshotReliable(mapInputToThrottle(axisY));
    
    // printf("Input: %4d -> Throttle: %4u\n", axisX, mapInputToThrottle(axisX));
    // printf("Input: %4d -> Throttle: %4u\n", axisY, mapInputToThrottle(axisY));
    // printf("X axis: %4d -> Y axis: %4d\n", axisX, axisY);
    // delay(1000);
    
  }
}
