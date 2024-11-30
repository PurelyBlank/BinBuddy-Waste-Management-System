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
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

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

const char * apSSID = "ESP32_CAM";       // Name of the WiFi network
const char * apPassword = "camera123";   // Password to connect

const char * SSID = "";
const char * PASSWORD = "";

// Cloud URL
const char* serverName = "http://<IP Address>/<PATH>";

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
    s->set_brightness(s, 1);     // Neutral brightness
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

void optimize_camera_focus(sensor_t *s) {
    // Maximize sharpness parameters
    s->set_sharpness(s, 3);      // Highest sharpness
    s->set_contrast(s, 2);       // Increased contrast
    
    // Exposure optimization
    s->set_exposure_ctrl(s, 1);  // Enable advanced exposure
    s->set_aec2(s, 1);           // Advanced exposure control
    s->set_ae_level(s, 0);       // Neutral exposure
    s->set_aec_value(s, 800);    // Adjusted exposure value
    
    // White balance
    s->set_whitebal(s, 1);       // Enable white balance
    s->set_awb_gain(s, 1);       // Auto white balance gain
    s->set_wb_mode(s, 0);        // Auto white balance mode
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

void connect_to_wifi() {
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
}

String upload_to_cloud(camera_fb_t *fb) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        return "";
    }

    // Create WiFiClient and HTTPClient
    WiFiClient client;
    HTTPClient http;
    http.begin(client, serverName);

    // Prepare multipart form data manually
    String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    
    // Construct the full multipart payload
    String startPart = "--" + boundary + "\r\n";
    startPart += "Content-Disposition: form-data; name=\"file\"; filename=\"capture.jpg\"\r\n";
    startPart += "Content-Type: image/jpeg\r\n\r\n";
    String endPart = "\r\n--" + boundary + "--\r\n";

    // Calculate total content length
    size_t contentLength = startPart.length() + fb->len + endPart.length();

    // Set headers explicitly
    http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http.addHeader("Content-Length", String(contentLength));

    // Debugging info
    Serial.println("Payload start length: " + String(startPart.length()));
    Serial.println("Image length: " + String(fb->len));
    Serial.println("Payload end length: " + String(endPart.length()));
    Serial.println("Total content length: " + String(contentLength));

    // Allocate buffer for full payload
    uint8_t* fullPayload = (uint8_t*)malloc(contentLength);
    if (!fullPayload) {
        Serial.println("Memory allocation failed");
        return "";
    }

    // Copy parts into buffer
    size_t offset = 0;
    memcpy(fullPayload + offset, startPart.c_str(), startPart.length());
    offset += startPart.length();
    
    memcpy(fullPayload + offset, fb->buf, fb->len);
    offset += fb->len;
    
    memcpy(fullPayload + offset, endPart.c_str(), endPart.length());

    // Send POST request
    int httpResponseCode = http.POST(fullPayload, contentLength);

    // Free allocated memory
    free(fullPayload);

    // Handle response
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.printf("HTTP Response code: %d\n", httpResponseCode);
        Serial.println("Server Response: ");
        Serial.println(response);
        http.end();
        return response;
    } else {
        Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
        http.end();
        return "";
    }
}

// UNFINISHED, turn motor based on response
void turn_motor(String response) {
  if (response == "YES") {
    Serial.println("RECYCLABLE");
  } else if ( response == "NO") {
    Serial.println("NOT RECYCLABLE");
  } else {
    Serial.println("Response Error");
  } 
}

void classify_image() {
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

  String response = upload_to_cloud(fb);  // Upload the captured photo 
  turn_motor(response);

  // Release the frame buffer
  esp_camera_fb_return(fb);
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

    sensor_t * sensor = esp_camera_sensor_get();
    // Configure camera settings
    config_camera_settings(sensor);
    optimize_camera_focus(sensor);

    // Connect to WiFi
    connect_to_wifi();

    // Configure GPIO 4 for flashlight control
    // pinMode(FLASH_LIGHT_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  // Turn on the flashlight
  // digitalWrite(4, HIGH);
  // Serial.println("Flashlight ON");

  if (Serial.available() > 0) {
    char command = Serial.read();
    command = toupper(command);
    if (command == 'C') { // captures an image
      classify_image();      
    }
  }  

  // delay(2000); // Keep it on for 2 seconds
  // Turn off the flashlight
  // digitalWrite(4, LOW);
  // Serial.println("Flashlight OFF");

  digitalWrite(LED_PIN, LOW);
  delay(1000); // Keep it off for 2 seconds
}