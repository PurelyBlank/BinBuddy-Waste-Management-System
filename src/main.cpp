#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory

// Pin definition for CAMERA_MODEL_AI_THINKER 
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
  
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define BAUD_RATE 115200
#define FLASH_LIGHT_PIN 4
#define LED_PIN 12

int pictureNumber = 0;

void setup() {
    // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    
    Serial.begin(BAUD_RATE);

    // camera_config_t config;
    // config.ledc_channel = LEDC_CHANNEL_0;
    // config.ledc_timer = LEDC_TIMER_0;
    // config.pin_d0 = Y2_GPIO_NUM;
    // config.pin_d1 = Y3_GPIO_NUM;
    // config.pin_d2 = Y4_GPIO_NUM;
    // config.pin_d3 = Y5_GPIO_NUM;
    // config.pin_d4 = Y6_GPIO_NUM;
    // config.pin_d5 = Y7_GPIO_NUM;
    // config.pin_d6 = Y8_GPIO_NUM;
    // config.pin_d7 = Y9_GPIO_NUM;
    // config.pin_xclk = XCLK_GPIO_NUM;
    // config.pin_pclk = PCLK_GPIO_NUM;
    // config.pin_vsync = VSYNC_GPIO_NUM;
    // config.pin_href = HREF_GPIO_NUM;
    // config.pin_sccb_sda = SIOD_GPIO_NUM;
    // config.pin_sccb_scl = SIOC_GPIO_NUM;
    // config.pin_pwdn = PWDN_GPIO_NUM;
    // config.pin_reset = RESET_GPIO_NUM;
    // config.xclk_freq_hz = 20000000;
    // config.pixel_format = PIXFORMAT_JPEG;

    // if (psramFound()){
    //   config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    //   config.jpeg_quality = 10;
    //   config.fb_count = 2;
    // } else {
    //   config.frame_size = FRAMESIZE_SVGA;
    //   config.jpeg_quality = 12;
    //   config.fb_count = 1;
    // }

    // // Init Camera
    // esp_err_t err = esp_camera_init(&config);
    // if (err != ESP_OK) {
    //   Serial.printf("Camera init failed with error 0x%x", err);
    //   return;
    // }

    // //***** IF we want to mount SD card *****
    // //Serial.println("Starting SD Card");
    // if(!SD_MMC.begin()){
    //   Serial.println("SD Card Mount Failed");
    //   return;
    // }
    
    // uint8_t cardType = SD_MMC.cardType();
    // if(cardType == CARD_NONE){
    //   Serial.println("No SD Card attached");
    //   return;
    // }

    // // Configure GPIO 4 for flashlight control
    pinMode(FLASH_LIGHT_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    Serial.print("ASDASDASD");
}


void loop() {
  digitalWrite(LED_PIN, HIGH);
  // Turn on the flashlight
  digitalWrite(4, HIGH);
  Serial.println("Flashlight ON");
  delay(2000); // Keep it on for 2 seconds

  // Take Picture with Camera
  // camera_fb_t * fb = esp_camera_fb_get();
  // if (!fb) {
  //   Serial.println("Camera capture failed");
  //   return;
  // }

  // // Process the image buffer (fb->buf) here
  // Serial.printf("Captured photo! Size: %d bytes\n", fb->len);

  // // Release the frame buffer
  // esp_camera_fb_return(fb);

  // Turn off the flashlight
  digitalWrite(4, LOW);
  Serial.println("Flashlight OFF");
  digitalWrite(LED_PIN, LOW);
  delay(2000); // Keep it off for 2 seconds
}