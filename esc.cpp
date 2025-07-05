#include "esc.h"

#define MOTOR_PIN GPIO_NUM_18
#define RMT_CH RMT_CHANNEL_4

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

rmt_item32_t dshot_bit0;
rmt_item32_t dshot_bit1;
rmt_item32_t dshot_pause;

void escSetup() {
  Serial.println("DShot600 Motor Control - Specific Values Mode");

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
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(MOTOR_PIN, RMT_CH);
  config.clk_div = 1;  // 80MHz
  config.mem_block_num = 2;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  esp_err_t err = rmt_config(&config);
  if (err != ESP_OK) {
    Serial.printf("RMT config failed: %d\n", err);
    return;
  }

  err = rmt_driver_install(RMT_CH, 0, 0);
  if (err != ESP_OK) {
    Serial.printf("RMT install failed: %d\n", err);
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
}

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

  printPacketBinary(packet, throttle);

  rmt_item32_t items[17];

  for(int i = 15; i >= 0; i--) {
    if (packet & (1 << i)) {
      items[15-i] = dshot_bit1;
    } else {
      items[15-i] = dshot_bit0;
    }
  }

  items[16] = dshot_pause;

  // Send with error checking
  esp_err_t err = rmt_write_items(RMT_CH, items, 17, false);
  if (err != ESP_OK) {
    Serial.printf("RMT send error: %d\n", err);
    return false;
  }

  err = rmt_wait_tx_done(RMT_CH, pdMS_TO_TICKS(1));
  return (err == ESP_OK);
}
