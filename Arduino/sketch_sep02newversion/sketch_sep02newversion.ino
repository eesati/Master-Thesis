#include <Arduino.h>
#include <WiFi.h>
//#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
//#include "camera_pins.h"
//#include "Arduino.h"
#include "esp_http_client.h"
#define CAMERA_MODEL_ESP_EYE

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER


const char* ssid = "VEGG";
const char* password = "sss3kk2aaaa4";

String serverName = "192.168.0.16";   // REPLACE WITH Raspberry Pi IP ADDRESS

String serverPath = "/";     // 

const int serverPort = 9000;

WiFiClient client;

// CAMERA_MODEL_AI_THINKER this is commented from 21.09
//#define PWDN_GPIO_NUM     32
//#define RESET_GPIO_NUM    -1
//#define XCLK_GPIO_NUM      0
//#define SIOD_GPIO_NUM     26
//#define SIOC_GPIO_NUM     27

//#define Y9_GPIO_NUM       35
//#define Y8_GPIO_NUM       34
//#define Y7_GPIO_NUM       39
//#define Y6_GPIO_NUM       36
//#define Y5_GPIO_NUM       21
//#define Y4_GPIO_NUM       19
//#define Y3_GPIO_NUM       18
//#define Y2_GPIO_NUM        5
//#define VSYNC_GPIO_NUM    25
//#define HREF_GPIO_NUM     23
//#define PCLK_GPIO_NUM     22

// new ones here 
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23
#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

const int timerInterval = 30000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent
bool is_send = false;

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event) {
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("Disconnected from WiFi access point");
//            WiFi.begin(ssid, password);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.print("Obtained IP address: ");
            Serial.println(WiFi.localIP());
        break;
        case SYSTEM_EVENT_AP_STACONNECTED:
           Serial.println("Client connected");
        break; 
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
          Serial.println("Assigned IP address to client");
          is_send = true;
        break;
        default: break;
    }
}


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  //if(!camera_init_t()){  // check camera has started
     
  //  Serial.printf("Failed to initialize camera...");
  //  return;  
  //}
    camera_init_t();
 // WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);
  WiFi.onEvent(WiFiEvent);
  
  Serial.printf("Pased Wifi Event");

//  Serial.println();
//  Serial.print("Connecting to ");
//  Serial.println(ssid);
//  WiFi.begin(ssid, password);  
//  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print(".");
//    delay(500);
//  }
//  Serial.println();
//  Serial.print("ESP32-CAM IP Address: ");
//  Serial.println(WiFi.localIP());

// commented all lines on 21.sept
//  camera_config_t config;
//  config.ledc_channel = LEDC_CHANNEL_0;
//  config.ledc_timer = LEDC_TIMER_0;
//  config.pin_d0 = Y2_GPIO_NUM;
//  config.pin_d1 = Y3_GPIO_NUM;
//  config.pin_d2 = Y4_GPIO_NUM;
//  config.pin_d3 = Y5_GPIO_NUM;
//  config.pin_d4 = Y6_GPIO_NUM;
//  config.pin_d5 = Y7_GPIO_NUM;
//  config.pin_d6 = Y8_GPIO_NUM;
//  config.pin_d7 = Y9_GPIO_NUM;
//  config.pin_xclk = XCLK_GPIO_NUM;
//  config.pin_pclk = PCLK_GPIO_NUM;
//  config.pin_vsync = VSYNC_GPIO_NUM;
//  config.pin_href = HREF_GPIO_NUM;
//  config.pin_sscb_sda = SIOD_GPIO_NUM;
//  config.pin_sscb_scl = SIOC_GPIO_NUM;
//  config.pin_pwdn = PWDN_GPIO_NUM;
//  config.pin_reset = RESET_GPIO_NUM;
//  config.xclk_freq_hz = 20000000;
//  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
//  if(psramFound()){
//    config.frame_size = FRAMESIZE_SVGA;
//    config.jpeg_quality = 10;  //0-63 lower number means higher quality
//    config.fb_count = 2;
//  } else {
//    config.frame_size = FRAMESIZE_CIF;
//    config.jpeg_quality = 12;  //0-63 lower number means higher quality
//    config.fb_count = 1;
//  }
  
  // camera init
//  esp_err_t err = esp_camera_init(&config);
//  if (err != ESP_OK) {
//    Serial.printf("Camera init failed with error 0x%x", err);
//    delay(1000);
//    ESP.restart();
//  }
  
  camera_init_t();
  //sendPhoto(); 
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    sendPhoto();
    previousMillis = currentMillis;
  }
}

String sendPhoto(){
  Serial.println("Trying to send photo");
  if(!is_send)
    return "not connnected";
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--Image\r\nContent-Disposition: form-data; name=\"fileToUpload\"; filename=\"fileToUpload\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Image--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=Image");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
      
    esp_camera_fb_return(fb);

     client.stop();
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  return getBody;
}

void camera_init_t(){
  Serial.println("camer_init_t()");
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    Serial.print("psram Found\n");
    config.frame_size = (framesize_t)FRAMESIZE_XGA;
    config.jpeg_quality = 40;
    config.fb_count = 2;
  } else {
    Serial.print("psram NOT Found\n");
    config.frame_size = (framesize_t)FRAMESIZE_XGA;
    config.jpeg_quality = 40;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("ERROR: Camera init failed with error 0x%x", err);
    //return;
  }
}
