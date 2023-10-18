#include "esp_camera.h"
#include <WiFi.h>
#include <ArduinoWebsockets.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#define DESIRED_HUE 35
#define HUE_RANGE 3

#include "camera_pins.h"

using namespace websockets;
WebsocketsClient client;

// Enter your WiFi credentials
const char* ssid = "ESP32_ACCESS_POINT";
const char* password = "StrongPasswordForAP";

const char* serverHost = "192.168.4.1";
const uint16_t serverPort = 8888;

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QQVGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 31;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_saturation(s, 2);


  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");

  while (!client.connect(serverHost, serverPort, "/")) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Socket connected");
}

void loop() {
  captureAndSendPhoto();
  delay(250); // For only changing servo a set time
}

void captureAndSendPhoto() {
  // Take a picture
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  } else {
    Serial.println("Camera capture successful!");
  }

  Serial.print("Width: " + String(fb->width)+ " ");
  Serial.print("Height: " + String(fb->height)+ " ");
  Serial.print("Format: " + String(fb->format)+ " ");
  Serial.print("Length: " + String(fb->len) + " ");
  Serial.println("Free Memory: " + String(ESP.getFreeHeap()) + " bytes");

  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("Non-JPEG data not implemented");
    return;
  }

  uint32_t imageSize = fb->width * fb->height * 3;
  uint8_t *rgbData = (uint8_t *)malloc(sizeof(uint8_t)*imageSize);
  uint8_t *hueData = (uint8_t *)malloc(sizeof(uint8_t)*fb->width * fb->height);

  bool jpeg_converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgbData);
  if (!jpeg_converted) {
    Serial.println("Unable to convert to RGB888.");
    return;
  }

  // Send RGB Data to TTGO T-Display
  if (rgbData) {
    Serial.println("First Pixel Data of rgbData:");
    for (int i = 0; i < 3; i++) { // Print the first 32 bytes for example
      Serial.print(rgbData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    int maxXPos = findMaxXpos(hueData, DESIRED_HUE, HUE_RANGE);
    Serial.println("Hue Value: " + String(rgbToHue(rgbData, hueData, imageSize)));
    Serial.println("X-Position of Max Red: " + String(maxXPos));

    // Send image and x position to TTGO Display
    client.sendBinary((const char*) fb->buf, fb->len);
    client.send(String(maxXPos));
  } else {
    Serial.println("rgbData is empty or not allocated");
  }

  esp_camera_fb_return(fb);
  free(rgbData);
  free(hueData);
}

uint8_t rgbToHue(uint8_t *rgbData, uint8_t *hueData, uint32_t imageSize) {
  #define NO_HUE 255 // Greyscale
  int hue, delta, maxColour, minColour;
  uint8_t red, green, blue;
  
 for (int i = 0; i<imageSize-2; i+=3){
    hue = 0;
    red = rgbData[i+2];
    green = rgbData[i+1];
    blue = rgbData[i];

    maxColour = max(red, max(green, blue));
    minColour = min(red, min(green, blue));
    delta = maxColour - minColour;
    if (2 * delta <= maxColour) hue = NO_HUE;
    else {
      if (red == maxColour) hue = 42 + 42 * (green - blue) / delta;
      else if (green == maxColour) hue = 126 + 42 * (blue - red) / delta;
      else if (blue == maxColour) hue = 210 + 42 * (red - green) / delta;
    }
  hueData[i/3] = hue;
  }
  return hueData[0];
}

int findMaxXpos(uint8_t *hueValues, int desired, int tolerance)  {
  int *xPos = (int *)calloc(160, sizeof(int)); 
  int maxX = 0;
  int positionOf = -1;

  // the 14th pixel on the 15th row = 160 * 15 - 1 + 14

  for(int y = 0; y < 120; y++){
      for(int x = 0; x < 160; x++){
          if(((desired - tolerance) < hueValues[160*y + x]) && ((desired + tolerance) > hueValues[160*y + x])){
              xPos[x] = xPos[x] + 1;
          }
      }
  }

  for(int i = 0; i < 160; i++) {
      if(xPos[i] > maxX) {
          maxX = xPos[i];
          positionOf = i;
      }
  }

  // For printing out histogram
  // for (int i = 0; i < 160; i++) {
  //   int numBars = map(data[i], 0, maxX, 0, 20);  // Scale the bars to fit within 20 characters
  //   Serial.print("Index ");
  //   Serial.print(i);
  //   Serial.print(": ");
  //   for (int j = 0; j < numBars; j++) {
  //     Serial.print("I");
  //   }
  //   Serial.println();
  // }

  free(xPos);
  return positionOf;
}

