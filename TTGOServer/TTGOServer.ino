#include <SPI.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <TJpg_Decoder.h>
#include <TFT_eSPI.h>
#include <NewPing.h>

#define servoPin 2
#define trigPin 33
#define echoPin 32

#define motorPin1 26 // Motor driver input 1
#define motorPin2 27 // Motor driver input 2
#define motorEnablePin 12 // Motor driver enable pin

#define maxDistance 400
#define frontOfBoat 17.0 // Distance between ultrasonic sensor and front of boat

const char* ssid = "ESP32_ACCESS_POINT";
const char* password = "StrongPasswordForAP";

using namespace websockets;
WebsocketsServer server;
WebsocketsClient client;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
NewPing sonar(trigPin, echoPin, maxDistance);

// Declare global variables for the image data
const uint8_t* rgbData = nullptr;
size_t imageSize = 0;
int maxXPos = -1;
float measuredDistance = 0.05;

int uptime = 700; // Neutral position
int downtime = 20000 - 700;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  tft.begin();
  tft.setRotation(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true); // We need to swap the colour bytes

  pinMode(servoPin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  ledcSetup(0,500,8);
  ledcAttachPin(motorEnablePin,0); 

  // The jpeg image can be scaled by a factor of 1, 2, 4, or 8
  // The decoder must be given the exact name of the rendering function above
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);

  Serial.println();
  Serial.println("Setting AP...");

  tft.setCursor(0, 75, 4);
  tft.print("Connecting");
  tft.setCursor(55, 105, 4);
  tft.print("to");
  tft.setCursor(0, 135, 4);
  tft.print("ESP32CAM");

  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address : ");
  Serial.println(IP);

  server.listen(8888);
}

void loop() {
  if (server.poll()) {
    client = server.accept();
    drawInterface();
  }

  if (client.available()) {
    client.poll();

    WebsocketsMessage msg = client.readBlocking();

    uint32_t t = millis();

    // // Assign the message data to the global variables
    rgbData = (const uint8_t*)msg.c_str();
    imageSize = msg.length();

    // Draw the image, top left at 0,0
    TJpgDec.drawJpg(0, 7, rgbData, imageSize);

    // How much time did rendering take
    t = millis() - t;
    Serial.print(t);
    Serial.println(" ms");

    WebsocketsMessage msg2 = client.readBlocking();
    maxXPos = msg2.data().toInt();

    if (maxXPos >= 0) {
      drawingText(maxXPos, 170);
    } else {
      tft.setTextColor(TFT_RED,TFT_BLACK); 
      tft.fillRect(70, 170, 77, 20,TFT_BLACK);
      tft.setCursor(70, 170, 4);   
      tft.println("N/A");
    }
    drawingText(measuredDistance, 130);
    drawingText(uptime, 210);
  }
  setPWM(maxXPos);
  runMotor(maxXPos);
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  // Stop further decoding as the image is running off the bottom of the screen
  if (y >= tft.height()) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // Return 1 to decode the next block
  return 1;
}

void drawingText(uint32_t deltaT, int yPos){ 
   tft.setTextColor(TFT_RED,TFT_BLACK); 
   tft.fillRect(70, yPos, 77, 20, TFT_BLACK);
   tft.setCursor(70, yPos, 4);   
   tft.print(deltaT);
}

void drawInterface() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK); 
  tft.setCursor(0, 132, 2);   
  tft.print("Distance");
  tft.setCursor(0, 172, 2);   
  tft.print("Max X-Pos");
  tft.setCursor(0, 212, 2);   
  tft.print("Uptime");
}

void setPWM(int xPosition) {
  int desiredXPos = 80;
  if (xPosition >= 0) {
    int error = desiredXPos - xPosition; // Calculate the error
    float kp = 4.0; // Proportional gain, adjust as needed

    // Constrain the servo position to prevent extreme values
    uptime = constrain(700 + kp * error, 400, 1100);
    downtime = 20000 - uptime;

    // Set the servo position
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(uptime);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(downtime);

  }
  else {
    // If no red pixel found, set a default position
    uptime = 700;
    downtime = 20000 - uptime;
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(uptime);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(downtime);
  }
}

void runMotor(int redXPosition) {
  static float oldMeasuredDistance = 0.05;
  float measuredDuration = sonar.ping();
  measuredDistance = (measuredDuration / 2) * 0.0343;
  if (measuredDistance < 0.05) measuredDistance = oldMeasuredDistance;
  oldMeasuredDistance = measuredDistance;

  Serial.print(String(measuredDistance));
  Serial.println(" cm");


  if (frontOfBoat + 35.0 <= measuredDistance && redXPosition >= 0) {
    Serial.println("Running motor");
    ledcWrite(0, 125); 
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (frontOfBoat + 30.0 <= measuredDistance && measuredDistance < frontOfBoat + 35.0) {
    Serial.println("Running motor");
    ledcWrite(0, 100); 
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (frontOfBoat + 25.0 <= measuredDistance && measuredDistance < frontOfBoat + 30.0) {
    Serial.println("Braking");
    ledcWrite(0, 75); 
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    ledcWrite(0, 0); // it is 0%
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
}

