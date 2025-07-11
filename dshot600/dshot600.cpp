#include "dshot600.h"

void rmtSetup(gpio_num_t motorPin, rmt_channel_t channel) {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(motorPin, channel);
    config.clk_div = 1;
    config.mem_block_num = 2;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    esp_err_t error = rmt_config(&config);
    if (error != ESP_OK) {
        Serial.printf("RMT config failed: %d\n", error);
        return;
    }

    error = rmt_driver_install(channel, 0, 0);
    if (error != ESP_OK) {
        Serial.printf("RMT install failed: %d\n", error);
        return;
    }
}

DShot600::DShot600(gpio_num_t motorPin1, gpio_num_t motorPin2, gpio_num_t motorPin3, gpio_num_t motorPin4, 
            rmt_channel_t channel1, rmt_channel_t channel2, rmt_channel_t channel3, rmt_channel_t channel4){
    Serial.pintln("Initializing DShot600 with default pins and channels...");
    this->_motorPin1 = motorPin1;
    this->_motorPin2 = motorPin2;
    this->_motorPin3 = motorPin3;
    this->_motorPin4 = motorPin4;
    
    this->_channel1 = channel1;
    this->_channel2 = channel2;
    this->_channel3 = channel3;
    this->_channel4 = channel4;

    dShotBit0.level0 = 1;
    dShotBit0.duration0 = T0H;
    dShotBit0.level1 = 0;
    dShotBit0.duration1 = T0L;

    dShotBit1.level0 = 1;
    dShotBit1.duration0 = T1H;
    dShotBit1.level1 = 0;
    dShotBit1.duration1 = T1L;

    dShotPause.level0 = 0;
    dShotPause.duration0 = 2000;
    dShotPause.level1 = 0;
    dShotPause.duration1 = 0;
    
    Serial.println("Configuring RMT channels...");

    rmtSetup(this->_motorPin1, this->_channel1);
    rmtSetup(this->_motorPin2, this->_channel2);
    rmtSetup(this->_motorPin3, this->_channel3);
    rmtSetup(this->_motorPin4, this->_channel4);

    delay(3000);
    Serial.println("Arming ESC...");

    for(int i = 0; i < 200; i++) {
        sendDshotReliable(NEUTRAL_THROTTLE);
        delayMicroseconds(1000);
    }

    Serial.println("ESC Armed!");

}

void DShot600::sendDShotPacket(uint16_t throttle) {
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
    
    esp_err_t error1 = rmt_write_items(this->_channel1, items, 17, false);
    esp_err_t error2 = rmt_write_items(this->_channel2, items, 17, false);
    esp_err_t error3 = rmt_write_items(this->_channel3, items, 17, false);
    esp_err_t error4 = rmt_write_items(this->_channel4, items, 17, false);
    
    if (error1 != ESP_OK || error2 != ESP_OK || error3 != ESP_OK || error4 != ESP_OK) {
        Serial.printf("RMT send error - CH1: %d, CH2: %d\n", error1, error2, error3, error4);
        return false;
    }

    error1 = rmt_wait_tx_done(this->_channel1, pdMS_TO_TICKS(1));
    error2 = rmt_wait_tx_done(this->_channel2, pdMS_TO_TICKS(1));
    error3 = rmt_wait_tx_done(this->_channel3, pdMS_TO_TICKS(1));
    error4 = rmt_wait_tx_done(this->_channel4, pdMS_TO_TICKS(1));
    
    return (error1 == ESP_OK && error2 == ESP_OK && error3 == ESP_OK && error4 == ESP_OK);
}

void DShot600::dumpPacketBinary(uint16_t packet, uint16_t throttle) {
    Serial.printf("Throttle value:  %u (0x%04X)\n", throttle, throttle);
    Serial.printf("Packet value: %u (0x%04X)\n", packet, packet);
    Serial.print("Binary: ");

    for(int i = 15; i >= 0; i--) {
        Serial.print((packet >> i) & 1);
        if(i > 0 && i % 4 == 0) {
        Serial.print(" ");
        }
    }
    Serial.println();
}