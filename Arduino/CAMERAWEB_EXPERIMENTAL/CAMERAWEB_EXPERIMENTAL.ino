#include "esp_camera.h"
#include <WiFi.h>
#include "ESPAsyncWebServer.h"




WiFiServer server(80);
//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

const char* ssid = "Wn-VKCGMR";
const char* password = "test1234";

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  if(psramFound()){  //a way to determine between ESP32 Standard unit and an ESP32 WROVER unit?
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get(); // create a sensor
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
   // here begins the server connection
   Serial.println("Setting AP (Access Point)…");
   WiFi.softAP(ssid, password); 
 
  //WiFi.begin(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address ");
  Serial.println(IP);
  Serial.print("MAC address ");
  Serial.println(WiFi.softAPmacAddress());

 // server.begin();
 // while (WiFi.status() != WL_CONNECTED) {
  //  delay(500);
  //  Serial.print(".");
 // }
//  Serial.println("");
 // Serial.println("WiFi connected");

 // startCameraServer();

 // Serial.print("Camera Ready! Use 'http://");
 // Serial.print(WiFi.localIP());
 // Serial.println("' to connect");
}

void loop() {

  WiFiClient client = server.available();    //Listen for incoming clients

  if (client) 
  {                                          //If a new client connects,
    Serial.println("New Client.");           //print a message out in the serial port





    while (client.connected()) 
    {           
    Serial.println("Client connected.");
    Serial.println(client.available());

     if (client.available() > 0) 
     {
      // read the bytes incoming from the client:
      char thisChar = client.read();
      // echo the bytes back to the client:
      server.write(thisChar);
      // echo the bytes to the server as well:
      Serial.write(thisChar);
     }


    }
    //client.stop();
    //Serial.println("Client disconnected.");
    //Serial.println();
  }
  // put your main code here, to run repeatedly:
  delay(10000);
}
