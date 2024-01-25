#include <SPI.h>
#include <mcp2515.h>

/* Configuration defines */
#define MCP2512_CS_PIN      10
#define MCP2515_CLOCK       MCP_8MHZ
#define MCP2515_CAN_SPEED   CAN_250KBPS

/* Internal variables */
static unsigned long currentTime;
static unsigned long previousTime;
static MCP2515 mcp2515(MCP2512_CS_PIN);

/* Example frames */
can_frame canMsg1 = { .can_id = 0x100, .can_dlc = 1, .data = { 0x00 } };
can_frame canMsg2 = { .can_id = 0x500, .can_dlc = 8, .data = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07} };


void setup() {
  while (!Serial);
  Serial.begin(115200);

  if(mcp2515.reset() != MCP2515::ERROR_OK)
  {
    Serial.println("reset error");
  }
  if(mcp2515.setBitrate(MCP2515_CAN_SPEED, MCP2515_CLOCK) != MCP2515::ERROR_OK)
  {
    Serial.println("setBitrate error");
  }
  if(mcp2515.setNormalMode() != MCP2515::ERROR_OK)
  {
    Serial.println("setNormalMode error");
  }
}


static void _100ms_task(void)
{
  /* Enter your code here */

  mcp2515.sendMessage(&canMsg1);
  canMsg1.data[0]++;
}

static void _500ms_task(void)
{
  /* Enter your code here */
  
  mcp2515.sendMessage(&canMsg2);
}

static void _1000ms_task(void)
{
  /* Enter your code here */
}

static void simpleScheduler(void)
{
  currentTime = millis();

  if(currentTime > previousTime)
  {
    previousTime = currentTime;

    if(currentTime % 100 == 0)
    {
      _100ms_task();
    }
    if(currentTime % 500 == 0)
    {
      _500ms_task();
    }
    if(currentTime % 1000 == 0)
    {
      _1000ms_task();
    }
  }
}

void loop() {
  simpleScheduler();

  /* Another non-blocking code */
}