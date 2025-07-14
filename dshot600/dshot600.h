#pragma once
#include <Arduino.h>
#include <driver/rmt.h>
#include <algorithm>
#include "./rmt/rmt.h"


//default motor pins and RMT channels
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

#define NEUTRAL_THROTTLE 0

enum DShot {
    DSHOT600 = 0,
    DSHOT300 = 1,
}

class DShot600 {
    public:
        DShot600(DShot dshot, gpio_num_t motorPin1 = MOTOR_PIN_1, gpio_num_t motorPin2 = MOTOR_PIN_2, gpio_num_t motorPin3 = MOTOR_PIN_3, gpio_num_t motorPin4 = MOTOR_PIN_4, 
                 rmt_channel_t channel1 = RMT_CH_1, rmt_channel_t channel2 = RMT_CH_2, rmt_channel_t channel3 = RMT_CH_3, rmt_channel_t channel4 = RMT_CH_4);
        void sendDShotPacket(uint16_t throttle);
        void dumpPacketBinary(uint16_t packet, uint16_t throttle);
    private:
            rmt_item32_t _dShotBit0;
            rmt_item32_t _dShotBit1;
            rmt_item32_t _dShotPause;

            gpio_num_t _motorPin1;
            gpio_num_t _motorPin2;
            gpio_num_t _motorPin3;
            gpio_num_t _motorPin4;

            rmt_channel_t _channel1;
            rmt_channel_t _channel2;
            rmt_channel_t _channel3;
            rmt_channel_t _channel4;
}   