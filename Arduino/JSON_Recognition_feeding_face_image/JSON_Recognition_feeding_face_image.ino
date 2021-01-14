#include "esp_camera.h"
#include "fd_forward.h"
#include "fr_forward.h"
//#include "fr_flash.h"
#include "WiFi.h"
#include "Arduino.h"

//new
#include "fb_gfx.h"
#include <HTTPClient.h>
#include <base64.h>
#include <ArduinoJson.h>

#define CAMERA_MODEL_ESP_EYE
#include "camera_pins.h"


#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;



//String serverName = "192.168.0.12"; 
String serverName = "192.168.0.18"; // REPLACE WITH Raspberry Pi IP ADDRESS

String serverPath = "/";     // 
bool is_send_photo = false;
int matched_id= -1; 
#define relayPin 2 // pin 12 can also be used
unsigned long currentMillis = 0;
unsigned long openedMillis = 0;
long interval = 5000;           // open lock for ... milliseconds
 
#define ENROLL_CONFIRM_TIMES 1
#define FACE_ID_SAVE_NUMBER 7

String ssid = "WN-52CC85";
String password = "9ed9e32222";

//String ssid = "Wn-VKCGMR";
//String password = "test1234";

const int serverPort = 8585;

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

//face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
//dl_matrix3du_t *image_matrix =  NULL;
camera_fb_t * fb = NULL;
 
//dl_matrix3du_t *aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);

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

 //StaticJsonBuffer<300> JSONBuffer; //Memory pool

 

void setup() {

 


  
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
  config.pixel_format = PIXFORMAT_JPEG; 
 // config.frame_size = FRAMESIZE_SVGA; commented on 19 October
 // config.jpeg_quality = 10; commented on 19 October
 // config.fb_count = 1;  commented on 19 October
    
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
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

   http.begin("http://"+serverName+":"+serverPort+"/images");
   int httpCode = http.GET();                                        //Make the request
 
    if (httpCode > 0) { //Check for the returning code
 
        String payload = http.getString();
        Serial.println(httpCode);
        Serial.printf("First len = %d, txt = %s\r\n",payload.length(),payload.c_str());
        //Serial.println(payload);

        const size_t bufferSize = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(8) + 370;
        DynamicJsonBuffer jsonBuffer(bufferSize);
      
        JsonObject& parsed = jsonBuffer.parseObject(payload.c_str()); //Parse message
        const char * imagebase64 = parsed["imageData"];
        Serial.println(imagebase64);
        
      }
 
    else {
      Serial.println("Error on HTTP request");
    }
 
    http.end();
 
  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);

  start_enrolling();
  Serial.println("Reaching the end of setup");
 // dl_matrix3du_t *aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3); // 320,240 // dl_matrix3d.h in this file this is for allocating memory for the 3 d matrix
    //if (!aligned_face) {
        //esp_camera_fb_return(fb);
      //  Serial.println("dl_matrix3du_alloc failed");
       // httpd_resp_send_500(req);
        //return ESP_FAIL;

        
    }

//    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);  // dl_matrix3d.h in this file this is for allocating memory for the 3 d matrix
//    if (!image_matrix) {
        //esp_camera_fb_return(fb);
//        Serial.println("dl_matrix3du_alloc failed");
        
 //   }

  /* image_matrix->w=1;
   image_matrix->h=1;
   image_matrix->c=512;
   image_matrix->n=1;
   image_matrix->stride=512;
   *(image_matrix->item)=0.07;
 */
  /* 
   dl_matrix3du_t **temp;
  (*temp)->w=1;
  (*temp)->h=1;
  (*temp)->c=512;
  (*temp)->n=1;
  (*temp)->stride=512;
  *((*temp)->item)=0.07;
  */
  
   /* id_list.head=0;
  id_list.tail=1;
  id_list.count=1;
  id_list.size=7;
  id_list.confirm_times=5;
  
  id_list.id_list=temp;
  */
   

 /* dl_matrix3du_t *aligned_face = NULL;
    int matched_id = 0;

   // aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3); // again allocating memory, not sure why
   // if(!aligned_face){
     //   Serial.println("Could not allocate face recognition buffer");
       // return matched_id;
   // }
    //if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){ // we dont need the net boxes, therefore can be removed
       // if (is_enrolling == 1){
            int8_t left_sample_face = enroll_face(&id_list, aligned_face);
                  Serial.printf("Left Sample Face: %d\n", left_sample_face);
            if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
            }
            Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            //rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            if (left_sample_face == 0){
                   // is_enrolling = 0;
                Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
            }
            */
    
  /*
  face_id_list *l = &id_list;
              for (int i = 0; i < l->count; i++)
                 {
                    uint8_t head = (l->head + i) % l->size;
                      dl_matrix3d_t *face_id_vector = l->id_list[head];
                       // you can process the face_id_vector, its dimension is (1, 1, 1, 512)
                      Serial.printf("This is w");
                      Serial.println(face_id_vector->w);
                      Serial.printf("This is h");
                      Serial.println(face_id_vector->h);
                       Serial.printf("This is c");
                      Serial.println(face_id_vector->c);
                       Serial.printf("This is n");
                      Serial.println(face_id_vector->n);
                      Serial.printf("This is stride");
                      Serial.println(face_id_vector->stride);
                      Serial.printf("This is item");
                      //fptp_t *myitem = &(face_id_vector->item);
                      Serial.println(*face_id_vector->item);
                      
                  }
  */
  //read_face_id_from_flash(&id_list);
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
     //dl_matrix3d_t **temp2=id_list.id_list;
   // Serial.println("id_list.id_list.item");
   // Serial.println((long) *temp2, DEC);
//Serial.println((*temp)->*item);
   
  
//}


void look_for_face() {
  currentMillis = millis();
  if (run_face_recognition()) { // face recognition function has returned true
    //Serial.println("Face recognised");
    digitalWrite(relayPin, HIGH); //close (energise) relay
    openedMillis = millis(); //time relay closed
  }
  if (currentMillis - interval > openedMillis){ // current time - face recognised time > 5 secs
    digitalWrite(relayPin, LOW); //open relay
  }
}
 
static bool run_face_recognition() {
    
  // dl_matrix3du_t *image_matrix =  NULL;
  bool faceRecognised = false; // default

   
  int64_t start_time = esp_timer_get_time();
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
 
  int64_t fb_get_time = esp_timer_get_time();
  Serial.printf("Get one frame in %u ms.\n", (fb_get_time - start_time) / 1000); // this line can be commented out
 
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
  if (!res) {
    Serial.println("to rgb888 failed");
    dl_matrix3du_free(image_matrix);
  }
 
  
  box_array_t *net_boxes = NULL;
  net_boxes = face_detect(image_matrix, &mtmn_config);


 
  if (net_boxes) {
      dl_matrix3du_t *aligned_face = NULL;
      aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return false;

        }
    
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
        Serial.println("FACE ID IN LIST");
        //Serial.println(&id_list);
      matched_id = recognize_face(&id_list, aligned_face);
      
      if (matched_id >= 0) {
        is_send_photo = true;
        Serial.printf("Match Face ID: %u\n", matched_id);
        faceRecognised = true; // function will now return true
      } else {
        Serial.println("No Match Found");
         is_send_photo = true;
        matched_id = -1;
        faceRecognised = true;
      }
    } else {
      Serial.println("Face Not Aligned");
    }
    dl_matrix3du_free(image_matrix);
    free(net_boxes->box);
    free(net_boxes->landmark);
    free(net_boxes);
    dl_matrix3du_free(aligned_face);
  }
  else {
    dl_matrix3du_free(image_matrix);
   // dl_matrix3du_free(aligned_face);
 // dl_matrix3du_free(aligned_face); // added this on 20th october
  //dl_matrix3du_free(image_matrix);
  }
  esp_camera_fb_return(fb);
  return faceRecognised;
}



 
void loop() {
  Serial.println("Now starting the loop ");
  look_for_face(); 
  if(is_send_photo) {
      
    if (client.connect(serverName.c_str(), serverPort))
    {
      camera_fb_t * frame = NULL;
      frame = esp_camera_fb_get();

     
            
      http.begin("http://"+serverName+":"+serverPort+"/esptest");

      String str = (char*)frame->buf;
      String encoded = base64::encode(frame->buf,frame->len);

      http.addHeader("Content-Type", "application/json");
     // int httpResponseCode = http.POST("{\"ImageData:\":\""+encoded+"\"}");

      int httpResponseCode = http.POST("{\"DeviceId\":\"Device_id_1\",\"ImageData\":\""+encoded+"\"}");
      //http.POST("{\"DeviceId\":\"Place_Device_ID_here\",\"ImageData\":\""+encoded+"\"}");
     
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
       
      // Free resources
      http.end();
          
      //Serial.println("Connection successful!");    
      //String head = "--Image\r\nContent-Disposition: form-data; name=\"fileToUpload\"; filename=\"fileToUpload\"\r\nContent-Type: image/jpeg\r\n\r\n";
      //String tail = "\r\n--Image--\r\n";
      
       uint16_t imageLen = frame->len;
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
      //size_t fbLen = frame->len;
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
    
      esp_camera_fb_return(frame);
      
     // client.stop();
    }  
      is_send_photo = false;
    }
    delay(2000);
}
