#include <NTPClient.h>
#include <WiFiUdp.h>

#include "esp_camera.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "WiFi.h"
#include "Arduino.h"


#include "ESP32_MailClient.h"
//new
#include "fb_gfx.h"
#include <HTTPClient.h>
#include <base64.h>
#include <ArduinoJson.h>


#define CAMERA_MODEL_ESP_EYE
#include "camera_pins.h"

#include "mbedtls/aes.h"



// ED25519
#include <Crypto.h>
#include <Ed25519.h>
#include <RNG.h>
#include <utility/ProgMemUtil.h>
#include <string.h>
//#include <SPI.h>
//#include <WiFi101.h>

// ED 25519 


   uint8_t publicKey[32]  = {0x3d, 0x40, 0x17, 0xc3, 0xe8, 0x43, 0x89, 0x5a,
                   0x92, 0xb7, 0x0a, 0xa7, 0x4d, 0x1b, 0x7e, 0xbc,
                   0x9c, 0x98, 0x2c, 0xcf, 0x2e, 0xc4, 0x96, 0x8c,
                   0xc0, 0xcd, 0x55, 0xf1, 0x2a, 0xf4, 0x66, 0x0c};

uint8_t privateKey[32] = {0x4c, 0xcd, 0x08, 0x9b, 0x28, 0xff, 0x96, 0xda,
                   0x9d, 0xb6, 0xc3, 0x46, 0xec, 0x11, 0x4e, 0x0f,
                   0x5b, 0x8a, 0x31, 0x9f, 0x35, 0xab, 0xa6, 0x24,
                   0xda, 0x8c, 0xf6, 0xed, 0x4f, 0xb8, 0xa6, 0xfb};



uint8_t signature[64]  = {0x92, 0xa0, 0x09, 0xa9, 0xf0, 0xd4, 0xca, 0xb8,
                   0x72, 0x0e, 0x82, 0x0b, 0x5f, 0x64, 0x25, 0x40,
                   0xa2, 0xb2, 0x7b, 0x54, 0x16, 0x50, 0x3f, 0x8f,
                   0xb3, 0x76, 0x22, 0x23, 0xeb, 0xdb, 0x69, 0xda,
                   0x08, 0x5a, 0xc1, 0xe4, 0x3e, 0x15, 0x99, 0x6e,
                   0x45, 0x8f, 0x36, 0x13, 0xd0, 0xf1, 0x1d, 0x8c,
                   0x38, 0x7b, 0x2e, 0xae, 0xb4, 0x30, 0x2a, 0xee,
                   0xb0, 0x0d, 0x29, 0x16, 0x12, 0xbb, 0x0c, 0x00};


//String const mymessage= "something";
 //  const void * message = mymessage.c_str();


// Watchdog
#include "soc/rtc_wdt.h"
#include "esp_task_wdt.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"



//size_t len=mymessage.length();



typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)



#define emailSenderAccount    "espeye32@gmail.com"    
#define emailSenderPassword   "espusecase1234"
#define emailRecipient        "espeye32@gmail.com"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "ESP32 Test Email"

// The Email Sending data object contains config and data to send
SMTPData smtpData;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String timestampClient;
String dayStamp;
String timeStamp;



//String serverName = "192.168.0.18"; 
String serverName = "192.168.0.12";   // REPLACE WITH Raspberry Pi IP ADDRESS

String serverPath = "/";     // 
bool is_send_photo = false;
int matched_id= -1; 
#define relayPin 2 // pin 12 can also be used
unsigned long currentMillis = 0;
unsigned long openedMillis = 0;
long interval = 5000;           // open lock for ... milliseconds
 
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7


//tring ssid = "WN-52CC85";
//String password = "9ed9e32222";
String ssid = "Wn-VKCGMR";
String password = "test1234";

const int serverPort = 8585;

WiFiClient client;
//WiFiSSLClient client;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80; //  normal 80 
  mtmn_config.pyramid = 0.707; // normal 0.707
  mtmn_config.pyramid_times = 4;  // normal 4
  mtmn_config.p_threshold.score = 0.9; // 0.6
  mtmn_config.p_threshold.nms = 0.7; //0.7
  mtmn_config.p_threshold.candidate_number = 20;  //20 // this shows the number of candidate windows for each stage. 
  mtmn_config.r_threshold.score = 0.8;//0.7
  mtmn_config.r_threshold.nms = 0.7;//0.7
  mtmn_config.r_threshold.candidate_number = 10; //10 
  mtmn_config.o_threshold.score = 0.8;//0.7
  mtmn_config.o_threshold.nms = 0.7;//0.7
  mtmn_config.o_threshold.candidate_number = 1; // 1
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();
 
static face_id_list id_list = {0};
dl_matrix3du_t *image_matrix =  NULL;
camera_fb_t * fb = NULL;
 
dl_matrix3du_t *aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);

void register_face(dl_matrix3du_t *image_matrix, box_array_t *net_boxes) {

dl_matrix3du_t *aligned_face = NULL;
    //int matched_id = 0;

    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        //return matched_id;
     }

}


static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}


static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...){
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}


static int enrolling_run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
    dl_matrix3du_t *aligned_face = NULL;
    int matched_id = 0;

    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
        if (true){
            int8_t left_sample_face = enroll_face(&id_list, aligned_face);

            if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
            }
            Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            //rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            if (left_sample_face == 0){
                //is_enrolling = 0;
                Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
            }
        } 
    } else {
        Serial.println("Face Not Aligned");
        //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
    }

    dl_matrix3du_free(aligned_face);
    return matched_id;
}




static void start_enrolling(){
    //camera_fb_t * fb = NULL;
   bool detected = true;
    //esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();
    while(detected) {
    camera_fb_t *fb = esp_camera_fb_get();
    Serial.println("Here it takes the image ");
   // String str = (char*)fb->buf;
   // Serial.println(str);
   // String encoded = base64::encode(fb->buf,fb->len);
   // String decoded = base64::encode(encoded);
    
   // Serial.println(decoded);

    //uint8_t *buf;
    // *fb->buf=NULL;
    // decoded.getBytes(fb->buf,fb->len);
    Serial.println("Decode before and fb afer this");
    
   // Serial.print(encoded);
    if (!fb) {
        Serial.println("Camera capture failed");
        
    }

    //fb->buf=buf;
    //free(buf);
    size_t out_len, out_width, out_height;
    uint8_t * out_buf;
    bool s;
    
    int face_id = 0;
    if(fb->width > 400){  // this checks if the image is bigger then do not recognize
        size_t fb_len = 0;
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            
        } 
       // esp_camera_fb_return(fb);
        //int64_t fr_end = esp_timer_get_time();
        //Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        
    }

    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    if (!image_matrix) {
          
        esp_camera_fb_return(fb);
        Serial.println("dl_matrix3du_alloc failed");
       
     
    }

    out_buf = image_matrix->item;  // here both are same pointers either one can fill it out 
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf); // this function must be in our case because it somehow fills the parameters from the image 
    
    if(!s){
        dl_matrix3du_free(image_matrix);
        Serial.println("to rgb888 failed");
       
    }

    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
    Serial.println("Face Detecting");
    if (net_boxes){
      Serial.println("Face Detected");
        detected = false;
        
          face_id = enrolling_run_face_recognition(image_matrix, net_boxes);
       
       // draw_face_boxes(image_matrix, net_boxes, face_id);
       // free(net_boxes->score);
       // free(net_boxes->box);
       // free(net_boxes->landmark);
       // free(net_boxes);
    } 
    //else 
        //{
        Serial.println("Face not detected");
        //free(net_boxes->score);
        //free(net_boxes->box);
        //free(net_boxes->landmark);
        //free(net_boxes);
        //free(out_buf);
        //dl_matrix3du_free(image_matrix);
        //esp_camera_fb_return(fb);
        //start_enrolling();
        
        
        //}
    //jpg_chunking_t jchunk = {req, 0};
    Serial.println("Before image matrix ");
    //s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    dl_matrix3du_free(image_matrix);
    //if(!s){

    Serial.println("After image matrix ");
     //   Serial.println("JPEG compression failed");
     //   return ESP_FAIL;
    //}
  esp_camera_fb_return(fb);
   Serial.println("After camera image return ");
   }
    //int64_t fr_end = esp_timer_get_time();
  //  Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start)/1000), detected?"DETECTED ":"", face_id);
   // return res;
   Serial.println("Reaching the end of start enrolling ");
}


 HTTPClient http;

char * key = "abcdefghijklmnop";

 String encryptImage(char *input,char * key){
    unsigned char output[strlen(input)];

  mbedtls_aes_context aes; 
  mbedtls_aes_init( &aes );
  mbedtls_aes_setkey_enc( &aes, (const unsigned char*) key, strlen(key) * 8 );
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, (const unsigned char*)input, output);
  mbedtls_aes_free( &aes );

 String datas="";
  for (int i = 0; i < strlen(input); i++) {
    char str[3];
    sprintf(str, "%02x", (int)output[i]);
    datas.concat(str);
    Serial.print(str);
  }
  return datas;
  }

void setup() {


 // rtc_wdt_protect_off();
//rtc_wdt_disable();
//rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
//rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
//rtc_wdt_set_time(RTC_WDT_STAGE0, 550);  


   //disableCore0WDT();
  //disableCore1WDT();
 
   
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  // digitalWrite(relayPin, LOW); removed on 19 Octover
  //pinMode(relayPin, OUTPUT);
 
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
// new below removed on 19 October evening
// #define PWDN_GPIO_NUM    -1
//#define RESET_GPIO_NUM   -1
//#define XCLK_GPIO_NUM    4
//#define SIOD_GPIO_NUM    18
//#define SIOC_GPIO_NUM    23
//#define Y9_GPIO_NUM      36
//#define Y8_GPIO_NUM      37
//#define Y7_GPIO_NUM      38
//#define Y6_GPIO_NUM      39
//#define Y5_GPIO_NUM      35
//#define Y4_GPIO_NUM      14
//#define Y3_GPIO_NUM      13
//#define Y2_GPIO_NUM      34
//#define VSYNC_GPIO_NUM   5
//#define HREF_GPIO_NUM    27
//#define PCLK_GPIO_NUM    25

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


  // config.pixel_format = PIXFORMAT_YUV422;
  // config.pixel_format = PIXFORMAT_RGB565;
 // config.pixel_format = PIXFORMAT_GRAYSCALE;
   config.pixel_format = PIXFORMAT_JPEG; //default
  //config.frame_size = FRAMESIZE_SVGA; //commented on 19 October
  //config.jpeg_quality = 10; //commented on 19 October
  //config.fb_count = 1;  //commented on 19 October
    
if(psramFound()){  //a way to determine between ESP32 Standard unit and an ESP32 EYE
    Serial.println("PSRAM FOUND");
    config.frame_size = FRAMESIZE_QVGA; // QVGA //FRAMESIZE_UXGA
    config.jpeg_quality = 10; // 10
    config.fb_count = 1;
    } else {
      Serial.println("PSRAM NOT FOUND");
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif


  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
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
  //drop down frame size for higher initial frame rate

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +5 = 3600*5
  timeClient.setTimeOffset(3600);

  
  //sensor_t * s = esp_camera_sensor_get();
  //s->set_framesize(s, FRAMESIZE_SVGA);   
  //s->set_framesize(s, FRAMESIZE_QVGA);   // 320×240 pixels
  
 
  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);

    /*
   dl_matrix3d_t **temp;
  (*temp)->w=1;
  (*temp)->h=1;
  (*temp)->c=512;
  (*temp)->n=1;
  (*temp)->stride=512;
  *((*temp)->item)=0.07;

  
    id_list.head=0;
  id_list.tail=1;
  id_list.count=1;
  id_list.size=7;
  id_list.confirm_times=5;
  
  id_list.id_list=temp;
  */
  read_face_id_from_flash(&id_list);
 /* Serial.println("Head ");
  Serial.println(id_list.head);
  Serial.println("Tail ");
  Serial.println(id_list.tail);
  Serial.println("Count ");
  Serial.println(id_list.count);

  Serial.println("Size ");
  
  Serial.println(id_list.size);

  Serial.println("Confirm Times ");
  Serial.println(id_list.confirm_times);
  Serial.println("Matrix 3D ");
 

  dl_matrix3d_t **temp2=id_list.id_list;
    Serial.println("id_list.id_list.w");
    Serial.println((*temp2)->w);
    Serial.println("id_list.id_list.h");
     Serial.println((*temp2)->h);
    Serial.println("id_list.id_list.c");
    Serial.println((*temp2)->c);
    Serial.println("id_list.id_list.n");
    Serial.println((*temp2)->n);
    Serial.println("id_list.id_list.stride");
    Serial.println((*temp2)->stride);
    Serial.println("id_list.id_list.item");
    Serial.println(*((*temp2)->item));
    */
    // dl_matrix3d_t **temp2=id_list.id_list;
   // Serial.println("id_list.id_list.item");
   // Serial.println((long) *temp2, DEC);
//Serial.println((*temp)->*item);
   //start_enrolling();
  //Serial.println("Reaching the end of setup");
  
}


void look_for_face() {
  currentMillis = millis();
  if (run_face_recognition()) { // face recognition function has returned true
    Serial.println("Sending image to Hyperledger Fabric");
    digitalWrite(relayPin, HIGH); //close (energise) relay
    openedMillis = millis(); //time relay closed
    
  }
  
  
  if (currentMillis - interval > openedMillis){ // current time - face recognised time > 5 secs
    digitalWrite(relayPin, LOW); //open relay
  }
}
 
bool run_face_recognition() {

    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;



   static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    
  bool faceRecognised = false; // default
  int64_t start_time = esp_timer_get_time();
  fb = esp_camera_fb_get();


    logMemory();
Serial.println("Image captured");
   while(!timeClient.update()) {

   timeClient.forceUpdate();
        }

  formattedDate = timeClient.getFormattedDate();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
 else {
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;

         }

         
  int64_t fb_get_time = esp_timer_get_time();
 // Serial.printf("Getting a frame in %u ms.\n", (fb_get_time - start_time) / 1000); // this line can be commented out
 
  image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3); //default 4th parameter is 3 for rgb
  Serial.println("Allocating memory");
  uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
  if (!res) {
    Serial.println("to rgb888 failed");
    dl_matrix3du_free(image_matrix);
  }
   
    fr_ready = esp_timer_get_time();
  // esp_camera_fb_return(fb); // commented out on 6 January 
  box_array_t *net_boxes = NULL;

   
  net_boxes = face_detect(image_matrix, &mtmn_config);
  

  Serial.println("After Detection");
  fr_face = esp_timer_get_time();
    fr_recognize = fr_face;
 
  if (net_boxes) {
    
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
        Serial.println("FACE ID ALIGNED");
        //Serial.println(&id_list);
      matched_id = recognize_face(&id_list, aligned_face);
      fr_recognize = esp_timer_get_time();
      if (matched_id >= 0) {
        is_send_photo = true;
        Serial.printf("There is a person recognized and their matched ID: %u\n", matched_id);
        faceRecognised = true; // function will now return true
      } else {
        
          Serial.println("There is no person recognized although detected");
         if (!MailClient.sendMail(smtpData))
          Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
         is_send_photo = true;
        matched_id = -1;
        faceRecognised = true;
      }
    } else {
      Serial.println("Face Not Aligned");
    }
    //dl_matrix3du_free(image_matrix);
    free(net_boxes->box);
    free(net_boxes->landmark);
    free(net_boxes);
    //dl_matrix3du_free(aligned_face);
  }
  
     //dl_matrix3du_free(aligned_face); // added this on 20th october
     dl_matrix3du_free(image_matrix);
  
  fr_encode = esp_timer_get_time();



  int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;
        
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
        Serial.printf("MJPG: %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u=%u \n",
            
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
            avg_frame_time, 1000.0 / avg_frame_time,
            (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)process_time
        );








  
  //esp_camera_fb_return(fb);
  return faceRecognised;
}

void sendCallback(SendStatus msg) {
  // Print the current status
  Serial.println(msg.info());

  // Do something when complete
  if (msg.success()) {
    Serial.println("----------------");
  }
}

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

void logMemory() {
 Serial.println(ESP.getPsramSize() - ESP.getFreePsram());
}

void feedTheDog(){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  // feed dog 1
  TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG1.wdt_feed=1;                       // feed dog
  TIMERG1.wdt_wprotect=0;                   // write protect
}
 
void loop() {








  rtc_wdt_protect_off();
rtc_wdt_disable();

disableCore0WDT();
disableLoopWDT();
smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);

  // For library version 1.2.0 and later which STARTTLS protocol was supported,the STARTTLS will be 
  // enabled automatically when port 587 was used, or enable it manually using setSTARTTLS function.
  //smtpData.setSTARTTLS(true);

  // Set the sender name and Email
  smtpData.setSender("ESP32", emailSenderAccount);

  // Set Email priority or importance High, Normal, Low or 1 to 5 (1 is highest)
  smtpData.setPriority("High");

  // Set the subject
  smtpData.setSubject(emailSubject);

  // Set the message with HTML format
  smtpData.setMessage("<div style=\"color:#2f4468;\"><h1>Intruder Detected !</h1><p>- Sent from ESP32 board</p></div>", true);
  // Set the email message in text format (raw)
  //smtpData.setMessage("Hello World! - Sent from ESP32 board", false);

  // Add recipients, you can add more than one recipient
  smtpData.addRecipient(emailRecipient);
  //smtpData.addRecipient("YOUR_OTHER_RECIPIENT_EMAIL_ADDRESS@EXAMPLE.com");

  smtpData.setSendCallback(sendCallback);

  //while(!timeClient.update()) {
  //  timeClient.forceUpdate();
  //}

    //timestampClient = timeClient.getFormattedDate();
  //Serial.println(formattedDate);
    
  look_for_face(); 
  //esp_camera_fb_return(fb); // added on 6 January this works here
  if(is_send_photo) {
      
    if (client.connect(serverName.c_str(), serverPort))
    {
      //camera_fb_t * frame = NULL; // commented out on & January 
      //frame = esp_camera_fb_get();
   
       
      HTTPClient http;
            
      http.begin("http://"+serverName+":"+serverPort+"/esptest");

      String str = (char*)fb->buf; // changed here the frame to fb
     // String encoded = "This is encrypted";

     Serial.println("Encoding");
      String encoded = base64::encode(fb->buf,fb->len); // changed here the frame to fb  // our image is in bits but the max is 4000 bytes for string in esp32

        Serial.println("Encoded");    
            //started with ed25518
     /*    const void * message = encoded.c_str();
        

        size_t len=encoded.length();


         RNG.begin("TestEd25519 1.0");

    // Perform the tests.
   // testFixedVectors();
    Ed25519::sign(signature, privateKey, publicKey,
                  message, len);
      // Serial.println("Signed");

     if (memcmp(signature, signature, 64) == 0) {
      //  Serial.println("ok");
    } else {
      //  Serial.println("failed");
      
    Serial.println();
      }


      bool verified = Ed25519::verify(signature, publicKey, message, len);
   // elapsed = micros() - start;
    if (verified) {
     //   Serial.println("verified");
    } else {
      //  Serial.println("failed");
    }

   */   
     // char input [encoded.length()]; // mbytes limit for char array is 100 bytes. 
      //Serial.println(encoded.length());
      //encoded.toCharArray(input, encoded.length());
      //Serial.println(input);
      //String encryptedimage = encryptImage(input, key);

      //Serial.println(encryptedimage);
      // here the encryption comes min_face



       while(!timeClient.update()) {
   
        timeClient.forceUpdate();
            }

        timestampClient = timeClient.getFormattedDate();
      
      http.addHeader("Content-Type", "application/json");
     // int httpResponseCode = http.POST("{\"ImageData:\":\""+encoded+"\"}");

      String face= "Face_id_";
      String matched_id_string= String(matched_id);
      String face_id= face + matched_id_string;
      //int httpResponseCode = http.POST("{\"DeviceId\":\"Device_id_1\",\"ImageData\":\""+encoded+"\"}");
      int httpResponseCode = http.POST("{\"DeviceId\":\"Device_id_1\",\"Face_Id\":\""+face_id+"\",\"Timestamp\":\""+formattedDate+"\",\"timeStamp\":\""+timestampClient+"\",\"ImageData\":\""+encoded+"\"}");
      //http.POST("{\"DeviceId\":\"Place_Device_ID_here\",\"ImageData\":\""+encoded+"\"}");
     
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
       
      // Free resources
      http.end();
          
      //Serial.println("Connection successful!");    
      //String head = "--Image\r\nContent-Disposition: form-data; name=\"fileToUpload\"; filename=\"fileToUpload\"\r\nContent-Type: image/jpeg\r\n\r\n";
      //String tail = "\r\n--Image--\r\n";
      
       uint16_t imageLen = fb->len; // changed here the frame to fb
       Serial.println(imageLen);
      //uint16_t extraLen = head.length() + tail.length();
     // uint16_t totalLen = imageLen + extraLen;
      
      //client.println("POST " + serverPath + " HTTP/1.1");
      //client.println("Host: " + serverName);
      //client.println("Content-Length: " + String(totalLen));
     //client.println("Content-Type: multipart/form-data; boundary=Image");
     // client.println();
     // client.print(head);
      
     // uint8_t *fbBuf = frame->buf;
      //size_t fbLen = frame->len;m
     // for (size_t n=0; n<fbLen; n=n+1024)
     // {
     //   if (n+1024 < fbLen) {
     //     client.write(fbBuf, 1024);
      //    fbBuf += 1024;
     //   }
     //   else if (fbLen%1024>0) 
     //   {
     //     size_t remainder = fbLen%1024;
     //     client.write(fbBuf, remainder);
     //   }
    //  }   
    //  client.print(tail);
      
     // esp_camera_fb_return(frame);
      
     // client.stop();
    }  
    
      is_send_photo = false;
      delay(3000);
    }
    
    //delay(2000);

    smtpData.empty();
    esp_camera_fb_return(fb);
    
}
