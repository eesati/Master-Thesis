#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "esp_camera.h"




 // In this code I have an access point with an HTTP server request
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
 
const char* ssid = "Wn-VKCGMR";
const char* password = "test1234";
 
AsyncWebServer server(80); // create a server here 
 
bool initCamera(){
   
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
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
    
  esp_err_t result = esp_camera_init(&config);
   
  if (result != ESP_OK) {
    return false;
  }
 
  return true;
} // initialize camera untill here 
 
void setup() {  // start setup function
  Serial.begin(115200);
 
  if(!initCamera()){  // check camera has started
     
    Serial.printf("Failed to initialize camera...");
    return;  
  }

  // WiFi.softAP(ssid, password);

  // IPAddress IP = WiFi.softAPIP();
  // Serial.print("AP IP address: ");
  // Serial.println(IP);
  
  WiFi.begin(ssid, password); // begin connecting to the Wifie Access Point provided
 
  while (WiFi.status() != WL_CONNECTED) { // check status of the connection it returns https://www.arduino.cc/en/Reference/WiFiBegin
  delay(1000);                          /// it will loop here until it connects, once it is connected it will proceed to the other lines
   Serial.println("Connecting to WiFi..");
  }
 
  Serial.println(WiFi.localIP()); // it comes here once it is connected therefore print the ip address it has received. 
 server.begin();
  server.on("/picture", HTTP_GET, [](AsyncWebServerRequest * request) {
 
    camera_fb_t * frame = NULL;
    frame = esp_camera_fb_get();
 
    request->send_P(200, "image/jpeg", (const uint8_t *)frame->buf, frame->len);
 
    esp_camera_fb_return(frame);
  });
 
  server.begin();
}
 
void loop(){
  
  }
