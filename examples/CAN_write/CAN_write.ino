#include <mcp2515.h>
MCP2515 mcp2515(SS); //Default hardware CS pin (UNO=10, MEGA=53, ESP32=5 etc.)

struct can_frame canMsg1;
struct can_frame canMsg2;

void setup() {
  canMsg1.can_id  = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x8E;
  canMsg1.data[1] = 0x87;
  canMsg1.data[2] = 0x32;
  canMsg1.data[3] = 0xFA;
  canMsg1.data[4] = 0x26;
  canMsg1.data[5] = 0x8E;
  canMsg1.data[6] = 0xBE;
  canMsg1.data[7] = 0x86;

  canMsg2.can_id  = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x0E;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x08;
  canMsg2.data[4] = 0x01;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0xA0;
  
  Serial.begin(115200);
  delay(300);
  
  while (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("MCP2515 init failure!");
  }
  
  mcp2515.setBitrate(CAN_100KBPS); //set the bitrate to your requirements
  mcp2515.setNormalMode();
  
  Serial.println("Example: Write to CAN");
}

void loop() {
  mcp2515.sendMessage(&canMsg1);
  mcp2515.sendMessage(&canMsg2);

  Serial.println("Messages sent");
  
  delay(100);
}
