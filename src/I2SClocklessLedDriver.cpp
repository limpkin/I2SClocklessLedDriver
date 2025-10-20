/**
    @title     I2SClocklessLedDriver
    @file      I2SClocklessLedDriver.cpp
    @repo      https://github.com/hpwit/I2SClocklessLedDriver
    @Copyright © 2025 Yves Bazin
    @license   MIT License
    @license   For non MIT usage, commercial licenses must be purchased. Contact us for more information.
**/

#include "I2SClocklessLedDriver.h"

//IDF5.5: __NB_DMA_BUFFER and NUM_STRIPS are global variables to allow changing it at runtime
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    uint8_t __NB_DMA_BUFFER = 6;
    uint8_t NUM_STRIPS = 16;
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
    clock_speed clock_1123KHZ = {4, 20, 9};
    clock_speed clock_1111KHZ = {4, 2, 1};
    clock_speed clock_1000KHZ = {5, 1, 0};
    clock_speed clock_800KHZ = {6, 4, 1};
#endif