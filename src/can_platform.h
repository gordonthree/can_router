#pragma once

#ifdef ESP32
    // ESP32 / ESP-IDF
    #include "driver/twai.h"
    typedef twai_message_t can_msg_t;

#else
    // Generic fallback for STM32, Linux, unit tests, etc.
    typedef struct {
        uint32_t identifier;
        uint8_t  data_length_code;
        uint8_t  data[8];
    } can_msg_t;

#endif
