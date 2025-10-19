#include "I2SClocklessLedDriver.h"

//idf 5.5: __NB_DMA_BUFFER is variable to allow changing it at runtime
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    uint8_t __NB_DMA_BUFFER = 6;
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
    clock_speed clock_1123KHZ = {4, 20, 9}; //{4, 20, 9};
    clock_speed clock_1111KHZ = {4, 2, 1};
    clock_speed clock_1000KHZ = {5, 1, 0};
    clock_speed clock_800KHZ = {6, 4, 1};
#endif