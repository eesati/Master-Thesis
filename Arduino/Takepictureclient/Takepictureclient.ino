#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
// This line is added for watch dog
// #include <avr/wdt.h>
#include "soc/rtc_wdt.h"



#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "esp_camera.h"

//String serverName = "10.10.10.10";
String serverName = "192.168.0.12";   // REPLACE WITH Raspberry Pi IP ADDRESS

String serverPath = "/";     // 

const int serverPort = 9000;
WiFiClient client;
 
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
 
//AsyncWebServer server(80);
 
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
}
 
void setup() {

  // this line is added for watch dog
 // wdt_disable();
//  WDT_Disable( WDT );
 

  
WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
   String getAll;
  String getBody;
  Serial.begin(115200);
 
  if(!initCamera()){
     
    Serial.printf("Failed to initialize camera...");
    return;  
  }
    rtc_wdt_protect_off();
  rtc_wdt_disable();
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println(WiFi.localIP());
 
  //server.on("/picture", HTTP_GET, [](AsyncWebServerRequest * request) {
    
    
 
    //request->send_P(200, "image/jpeg", (const uint8_t *)frame->buf, frame->len);

 if (client.connect(serverName.c_str(), serverPort)) {
    camera_fb_t * frame = NULL;
    frame = esp_camera_fb_get();
    Serial.println("Connection successful!");    
    String head = "--Image\r\nContent-Disposition: form-data; name=\"fileToUpload\"; filename=\"fileToUpload\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Image--\r\n";

    uint16_t imageLen = frame->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=Image");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = frame->buf;
    size_t fbLen = frame->len;
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
      
    esp_camera_fb_return(frame);

     client.stop();
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }

    
 
   // esp_camera_fb_return(frame);
 // });
 
  //server.begin();
}
 
void loop(){
  
  }
