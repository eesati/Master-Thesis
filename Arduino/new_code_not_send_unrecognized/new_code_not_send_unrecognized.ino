#include "esp_camera.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "WiFi.h"
#include "Arduino.h"

String serverName = "192.168.0.12";   // REPLACE WITH Raspberry Pi IP ADDRESS

String serverPath = "/";     // 
bool is_send_photo = false;
#define relayPin 2 // pin 12 can also be used
unsigned long currentMillis = 0;
unsigned long openedMillis = 0;
long interval = 5000;           // open lock for ... milliseconds
 
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

String ssid = "Wn-VKCGMR";
String password = "test1234";

const int serverPort = 9000;

WiFiClient client;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();
 
static face_id_list id_list = {0};
dl_matrix3du_t *image_matrix =  NULL;
camera_fb_t * fb = NULL;
 
dl_matrix3du_t *aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
 
void setup() {
  Serial.begin(115200);
 
  digitalWrite(relayPin, LOW);
  pinMode(relayPin, OUTPUT);
 
  camera_config_t config;
 // config.ledc_channel = LEDC_CHANNEL_0;
 // config.ledc_timer = LEDC_TIMER_0;
 // config.pin_d0 = 5;
 // config.pin_d1 = 18;
 // config.pin_d2 = 19;
 // config.pin_d3 = 21;
 // config.pin_d4 = 36;
 // config.pin_d5 = 39;
 // config.pin_d6 = 34;
 // config.pin_d7 = 35;
 // config.pin_xclk = 0;
 // config.pin_pclk = 22;
 // config.pin_vsync = 25;
 // config.pin_href = 23;
 // config.pin_sscb_sda = 26;
 // config.pin_sscb_scl = 27;
 // config.pin_pwdn = 32;
 // config.pin_reset = -1;
 // config.xclk_freq_hz = 20000000;
 // config.pixel_format = PIXFORMAT_JPEG;
 // config.frame_size = FRAMESIZE_SVGA;
 // config.jpeg_quality = 12;
 // config.fb_count = 1;


// old up
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
    


  WiFi.begin(ssid.c_str(), password.c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");


  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
 
  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
 
  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  read_face_id_from_flash(&id_list);// Read current face data from on-board flash
}
 
void rzoCheckForFace() {
  currentMillis = millis();
  if (run_face_recognition()) { // face recognition function has returned true
    Serial.println("Face recognised");
    digitalWrite(relayPin, HIGH); //close (energise) relay
    openedMillis = millis(); //time relay closed
  }
  if (currentMillis - interval > openedMillis){ // current time - face recognised time > 5 secs
    digitalWrite(relayPin, LOW); //open relay
  }
}
 
bool run_face_recognition() {
  bool faceRecognised = false; // default
  int64_t start_time = esp_timer_get_time();
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
 
  int64_t fb_get_time = esp_timer_get_time();
  Serial.printf("Get one frame in %u ms.\n", (fb_get_time - start_time) / 1000); // this line can be commented out
 
  image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
  if (!res) {
    Serial.println("to rgb888 failed");
    dl_matrix3du_free(image_matrix);
  }
 
  esp_camera_fb_return(fb);
 
  box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
 
  if (net_boxes) {
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
 
      int matched_id = recognize_face(&id_list, aligned_face);
      if (matched_id >= 0) {
        is_send_photo = true;
        Serial.printf("Match Face ID: %u\n", matched_id);
        faceRecognised = true; // function will now return true
      } else {
        Serial.println("No Match Found");
        matched_id = -1;
      }
    } else {
      Serial.println("Face Not Aligned");
    }
 
    free(net_boxes->box);
    free(net_boxes->landmark);
    free(net_boxes);
  }
 
  dl_matrix3du_free(image_matrix);
  return faceRecognised;
}
 
void loop() {
  rzoCheckForFace();
  if(is_send_photo);
  {
   
    if (client.connect(serverName.c_str(), serverPort))
    {
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
      for (size_t n=0; n<fbLen; n=n+1024)
      {
        if (n+1024 < fbLen) {
          client.write(fbBuf, 1024);
          fbBuf += 1024;
        }
        else if (fbLen%1024>0) 
        {
          size_t remainder = fbLen%1024;
          client.write(fbBuf, remainder);
        }
      }   
      client.print(tail);
      esp_camera_fb_return(frame);
      client.stop();
    }  
      is_send_photo = false;
    }
}
