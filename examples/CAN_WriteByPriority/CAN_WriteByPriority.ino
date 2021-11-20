#include <SPI.h>
#include <mcp2515.h>

#define CS_PIN 10

#define MSG_PER_BUFFER 3
#define TX_BUFFERS 3

MCP2515 mcp2515(CS_PIN);

struct can_frame msg;

void setup() {
 
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setConfigMode();
  mcp2515.setTXPriority(MCP2515::TXB0, MCP2515::PRIORITY_HIGH);
  mcp2515.setTXPriority(MCP2515::TXB1, MCP2515::PRIORITY_MEDIUM);
  mcp2515.setTXPriority(MCP2515::TXB2, MCP2515::PRIORITY_LOW);
  mcp2515.setNormalMode();

  msg.can_id  = 0x0F6;
  msg.can_dlc = 8;
  msg.data[0] = 0x00;
  msg.data[1] = 0x00;
  msg.data[2] = 0x00;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;
}

uint8_t index = 0;
void loop(){
  for(uint8_t i = 0; i < MSG_PER_BUFFER * TX_BUFFERS; i++){
    msg.data[0] = index++;
    mcp2515.sendMessage(MCP2515::TXB0, &msg);
    msg.data[0] = index++;
    mcp2515.sendMessage(MCP2515::TXB1, &msg);
    msg.data[0] = index++;
    mcp2515.sendMessage(MCP2515::TXB2, &msg);
  }
  delay(10000);
}