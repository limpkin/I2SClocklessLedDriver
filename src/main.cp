#define SETUP_WIFI 1 
// flash without WiFi:
// RAM:   [=         ]   8.1% (used 26468 bytes from 327680 bytes)
// Flash: [==        ]  23.4% (used 307226 bytes from 1310720 bytes)
// flash with WiFi: !!!
// RAM:   [==        ]  15.4% (used 50312 bytes from 327680 bytes)
// Flash: [======    ]  64.7% (used 847998 bytes from 1310720 bytes)

#include "Arduino.h"
#include "I2SClocklessLedDriver.h"

#if SETUP_WIFI
    #include "esp_wifi.h"
    #include "esp_event.h"
    #include "nvs_flash.h"
#endif

#define NUM_LEDS_PER_STRIP 256
#define NUMSTRIPS 6


//here we have 3 colors per pixel
uint8_t leds[NUMSTRIPS*NUM_LEDS_PER_STRIP*3];

#ifdef CONFIG_IDF_TARGET_ESP32S3
    // uint8_t pins[6] = {9, 10,12,8,18,17};
    uint8_t pins[6] = {16, 10,12,8,18,17};
#else
    uint8_t pins[6] = {2, 12, 13, 25, 33, 32};
#endif

I2SClocklessLedDriver driver;

void setup() {
    Serial.begin(115200);

    #if SETUP_WIFI
        esp_netif_init();
        esp_event_loop_create_default();
        esp_netif_create_default_wifi_ap();
        
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        
        wifi_config_t wifi_config = {};
        strcpy((char*)wifi_config.ap.ssid, "ESP32_AP");
        strcpy((char*)wifi_config.ap.password, "password123");
        wifi_config.ap.ssid_len = strlen("ESP32_AP");
        wifi_config.ap.channel = 1;
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.ap.max_connection = 4;
        
        esp_wifi_set_mode(WIFI_MODE_AP);
        esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
        esp_wifi_start();
    #endif

    driver.initled(leds,pins,NUMSTRIPS,NUM_LEDS_PER_STRIP,ORDER_GRB);
    driver.setBrightness(10);
}

int off=0;
long time1,time2,time3;

void loop() {
    time1=ESP.getCycleCount();

    uint8_t effect = 2;

    switch (effect) {
        case 0:
            //rainbowy
            for(int j=0;j<NUMSTRIPS;j++)
                for(int i=0;i<NUM_LEDS_PER_STRIP;i++)
                    driver.setPixel((i+off)%NUM_LEDS_PER_STRIP+NUM_LEDS_PER_STRIP*j,(NUM_LEDS_PER_STRIP-i)*255/NUM_LEDS_PER_STRIP,i*255/NUM_LEDS_PER_STRIP,(((128-i)+255)%255)*255/NUM_LEDS_PER_STRIP);
            break;
        case 1:
            // random effect    
            for(int i=0;i<NUM_LEDS_PER_STRIP*NUMSTRIPS*3;i++)
                leds[i] *= 0; //fadetoblack 70%
            driver.setPixel(random(NUM_LEDS_PER_STRIP*NUMSTRIPS), 255, random(255), 0);
            break;
        case 2:
            // lines
            for(int i=0;i<NUM_LEDS_PER_STRIP*NUMSTRIPS*3;i++)
                leds[i] *= 0; //black
            uint16_t time = 200;//ms
            uint8_t row = (millis()/time)%16;
            for (uint8_t col=0; col<16; col++)
                driver.setPixel(row*16+((row%2==0)?col:(15-col)), 255, 0, 0);
            break;
    }
    
    time2=ESP.getCycleCount();

    driver.showPixels();

    time3=ESP.getCycleCount();
    if (off%100 == 0)
        Serial.printf("Calcul pixel fps:%.2f   showPixels fps:%.2f   Total fps:%.2f \n",(float)240000000/(time2-time1),(float)240000000/(time3-time2),(float)240000000/(time3-time1));
    off++;
}
