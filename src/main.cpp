#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include <SD_MMC.h>
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory
#include <WiFi.h>

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

#define BAUD_RATE 9600
#define FLASH_LIGHT_PIN 4
#define LED_PIN 12
#define ESP_32_CAM_IP_ADDR "192.168.4.1"
#define EEPROM_SIZE 1 // For SD card size

const char* apSSID = "ESP32_CAM";       // Name of the WiFi network
const char* apPassword = "camera123";   // Password to connect
IPAddress ipAddr;

int pictureNumber = 0;

IPAddress get_esp_ip_addr() {
  WiFi.softAP(apSSID, apPassword);
  return WiFi.softAPIP(); 
}

// Add this before setup()
void init_sd_card() {
    // Use 1-bit mode for SD_MMC
    if(!SD_MMC.begin("/sdcard", true)){  // true enables 1-bit mode
        Serial.println("SD Card Mount Failed");
        
        // Optional: Blink LED to indicate SD card error
        while(1) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
    }

    uint8_t cardType = SD_MMC.cardType();
    switch(cardType) {
        case CARD_NONE:
            Serial.println("No SD Card attached");
            return;
        case CARD_MMC:
            Serial.println("MMC card detected");
            break;
        case CARD_SD:
            Serial.println("SD card detected");
            break;
        case CARD_SDHC:
            Serial.println("SDHC card detected");
            break;
    }

    // Print card size for verification
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void config_camera_settings(sensor_t * s) {
    // Sharp focus settings
    s->set_brightness(s, 0);     // Neutral brightness
    s->set_contrast(s, 2);       // Slightly increased contrast
    s->set_saturation(s, 0);     // Neutral saturation
    
    // Auto white balance and exposure
    s->set_whitebal(s, 1);       // Enable white balance
    s->set_awb_gain(s, 1);       // Enable auto white balance gain
    s->set_exposure_ctrl(s, 1);  // Enable exposure control
    s->set_aec2(s, 1);           // Advanced exposure control
    s->set_ae_level(s, 0);       // Neutral exposure level
    
    // Sharpness and noise reduction
    s->set_sharpness(s, 2);      // Moderate sharpness increase
    s->set_denoise(s, 1);        // Light noise reduction
}

void save_to_sd_card(camera_fb_t * fb) {
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close(); 
}

void setup() {
    Serial.begin(BAUD_RATE);
    
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()){
      config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
      config.jpeg_quality = 5;
      config.fb_count = 2;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 8;
      config.fb_count = 1;
    }

    // Init Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }

    // If we want to use SD card
    init_sd_card();

    // Configure camera settings
    config_camera_settings(esp_camera_sensor_get());

    // Configure GPIO 4 for flashlight control
    // pinMode(FLASH_LIGHT_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  // Turn on the flashlight
  // digitalWrite(4, HIGH);
  // Serial.println("Flashlight ON");

  // Take Picture with Camera
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Save picture to sd card
  save_to_sd_card(fb);

  // Process the image buffer (fb->buf) here
  Serial.printf("Captured photo! Size: %d bytes\n", fb->len);

  // Release the frame buffer
  esp_camera_fb_return(fb);

  // delay(2000); // Keep it on for 2 seconds
  // Turn off the flashlight
  // digitalWrite(4, LOW);
  // Serial.println("Flashlight OFF");

  digitalWrite(LED_PIN, LOW);
  delay(5000); // Keep it off for 5 seconds
}