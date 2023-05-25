#include <mcp2515.h>
MCP2515 mcp2515(SS); //Default hardware CS pin (UNO=10, MEGA=53, ESP32=5 etc.)

struct can_frame canMsg;

unsigned long oldTime = 0;
int cntr = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  
  while (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("MCP2515 init failure!");
  }
  
  mcp2515.setBitrate(CAN_100KBPS); //set the bitrate to your requirements
  mcp2515.setNormalMode();
}
 
void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    cntr++;
  }
 
  if ((millis()-oldTime)>1000) {
    oldTime = millis();
    Serial.print(cntr);
    Serial.println(" msg/sec");
    cntr = 0;      
  }
}
