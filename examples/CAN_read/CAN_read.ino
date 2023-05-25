#include <mcp2515.h>
MCP2515 mcp2515(SS); //Default hardware CS pin (UNO=10, MEGA=53, ESP32=5 etc.)

struct can_frame canMsg;

unsigned long timer = 0;
char debug_time[8];

void setup() {
  Serial.begin(115200);
  delay(300);
  
  while (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("MCP2515 init failure!");
  }
  
  mcp2515.setBitrate(CAN_100KBPS); //set the bitrate to your requirements

  /*
    //Set MASK0 and MASK1 to 0x7FF to close the module from all IDs.
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
    
    //Set FILTER0...5 to only receive messages from what IDs you need (delete unwanted).
    mcp2515.setFilter(MCP2515::RXF0, false, 0x...);
    mcp2515.setFilter(MCP2515::RXF1, false, 0x...);
    mcp2515.setFilter(MCP2515::RXF2, false, 0x...);
    mcp2515.setFilter(MCP2515::RXF3, false, 0x...);
    mcp2515.setFilter(MCP2515::RXF4, false, 0x...);
    mcp2515.setFilter(MCP2515::RXF5, false, 0x...);
  */
  
  mcp2515.setNormalMode();
  
  Serial.println("  Time  | IDs |         Data");
  Serial.println("--------+-----+------------------------");

  timer = millis();
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    snprintf(debug_time, 8, "%7lu", millis() - timer);
    timer = millis();
    
    Serial.print(debug_time); // print the time since the last message
    Serial.print(" | ");
    
    char can_id[4];
    snprintf(can_id, 4, "%03lX", canMsg.can_id);
    Serial.print(can_id); // print ID
    Serial.print(" | ");
    
    for (int i = 0; i < canMsg.can_dlc; i++)  {  // print the data
      if (canMsg.data[i] < 0x10) Serial.print(0);
      Serial.print(canMsg.data[i], HEX);
      Serial.print(' ');
    }

    Serial.println();      
  }
}