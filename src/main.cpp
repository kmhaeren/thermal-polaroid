/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include <EEPROM.h>            // read anpyd write from flash memory


// define the number of bytes you want to access
#define EEPROM_SIZE 1

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
#define LAMP 4

#define DTR_PIN           14
 
#define num_dots          2
#define heat_time         80
#define heat_interval     2

int c_round(int a){

  if(a < 128){
    return 0;
  }
  else{
    return 255;
  }

}

double clip(double a){

  if(a > 255){
    return 255;
  }
  else if(a < 0){
    return 0;
  }
  else{
    return a;
  }

}

uint8_t * dither(uint8_t* inmat, size_t h, size_t w){
  for(int j = 0; j < h-1; j++){
    for(int i = 1; i<w-1; i++){
      double old_p = inmat[j*w + i];
      double new_p = c_round(old_p);

      inmat[j*w + i] = new_p;
      double err = old_p - new_p;

      inmat[(j*w) + i + 1] = clip((double)inmat[(j*w) + i + 1] + err * 7.0/16);
      inmat[(j+1)* w + i - 1] = clip((double)inmat[(j+1)* w + i - 1] + err * 3.0/16);
      inmat[(j+1)* w + i] = clip((double)inmat[(j+1)* w + i] + err * 5.0/16);
      inmat[(j+1)* w + i + 1] = clip((double)inmat[(j+1)* w + i + 1] + err * 1.0/16);

    }
  }
  for (int j = 0; j < h; j++) {
    for (int i = 1; i<w; i++) {
    
      if (inmat[j*w + i] < 122) {
        inmat[j*w + i] = 1;
      }
      else {
        inmat[j*w + i] = 0;
      }
    }
  }

  return inmat;
}


void writePrinter(uint8_t* msg, int l){
  while(digitalRead(DTR_PIN) == HIGH){
    
  }
      for(int i = 0; i< l; i++){
        Serial2.write(msg[i]);
    }
}


typedef struct {
    uint32_t filesize;
    uint32_t reserved;
    uint32_t fileoffset_to_pixelarray;
    uint32_t dibheadersize;
    int32_t width;
    int32_t height;
    uint16_t planes;
    uint16_t bitsperpixel;
    uint32_t compression;
    uint32_t imagesize;
    uint32_t ypixelpermeter;
    uint32_t xpixelpermeter;
    uint32_t numcolorspallette;
    uint32_t mostimpcolor;
} bmp_header_t;

static const int BMP_HEADER_LEN = 54;


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
 
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 15, 13);


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
  config.frame_size = FRAMESIZE_XGA; 
  config.jpeg_quality = 1;
  config.fb_count = 1;

  pinMode(DTR_PIN, INPUT);

  esp_err_t err = esp_camera_init(&config);
  
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
      
  /*if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
 
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }*/

  camera_fb_t * fb = NULL;
  sensor_t * s = esp_camera_sensor_get();

  s->set_contrast(s, 2);
  s->set_wb_mode(s, 4);  

  //pinMode(LAMP, OUTPUT);
  
//  digitalWrite(LAMP, HIGH);
//  delay(500);

  fb = esp_camera_fb_get();  
//  delay(500);

//  digitalWrite(LAMP, LOW);

  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  uint8_t * buf = NULL;
  size_t buf_len = 0;

  frame2bmp(fb, &buf, &buf_len);
/*
  EEPROM.begin(EEPROM_SIZE);
  int pictureNumber = EEPROM.read(0) + 1;
  String path = "/picture" + String(pictureNumber) +".bmp";


  fs::FS &fs = SD_MMC; 
  File file = fs.open(path.c_str(), FILE_WRITE);

  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->width*fb->height); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  EEPROM.end();
*/

 // Serial2.begin(9600, SERIAL_8N1, 15, 13);

  uint8_t msg_dtr[] = {29, 97, 32};
  writePrinter(msg_dtr, 3);
  uint8_t msg_sets[] = {27, 55, num_dots, heat_time, heat_interval};
  writePrinter(msg_sets, 5);

  
  bmp_header_t * bitmap  = (bmp_header_t*)&buf[2];

  Serial.println();
  Serial.print("w:");
  Serial.println(bitmap->width);
  Serial.print("h:");
  Serial.println(bitmap->height);

  uint8_t *img = buf;  
  size_t width = fb->width;
  size_t height = fb->height;

  int i = 0;
  for(int c = bitmap->fileoffset_to_pixelarray; c < buf_len; c+=3){
      uint8_t gray = (uint8_t) (0.3*(float)img[c]+0.59*(float)img[c+1]+0.11*(float)img[c+2]);
      img[i] = gray;
      i++;
  }

  int count = 0;
  for(int r = 1; r < height; r+=2){
    for(int c = 1; c < width; c+=2){
      int temp = 0;
      
      for(int k = -1; k<2; k++){
        for(int l = -1; l<2; l++){
           temp += img[(r+k)*width + (c+l)];
        }
      }
    img[count] = (uint8_t) (temp/9);

    count++;
    }
  }

  height /= 2;
  width /= 2;
  
  img = dither(img, height, width);



  count = 0;
  for(int r = 0; r < height;r+=8){
    for(int c = 0; c < width; c++){
      int temp = 0;
      for(int i = 0; i< 8; i++){
              temp += (img[(r+i)*width + c]) * pow(2, (7-i));        
      }

      img[count] = temp;
      count++;
    }
  }

  height /= 8;

  Serial.println("START");
  int temp_w = 200;

  for(int c = 0; c < width; c+=temp_w){
    
    int print_w = 0;

    if(width - c > temp_w){
      print_w = temp_w;
    }
    else{
      print_w = width - c;
    }

    uint8_t msg1[] = {18, 42, print_w, height};
    writePrinter(msg1, 4);

    for(int i = 0; i < print_w; i++){
      for(int r = 0; r < height; r++){
        uint8_t msg2[] = {img[r*width + c+i]};
        Serial.print((int) img[r*width + c+i]);
        Serial.print(" ");
        writePrinter(msg2, 1);
      }
    }

    Serial.println();
    uint8_t msg[] = {29, 47, 3};
    writePrinter( msg, 3);
  }

  Serial.println("STOP");
  esp_camera_fb_return(fb);

  
}



void loop() {  
}