#include <mcp2515_can.h>
#include <mcp_can.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>

const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN);

unsigned char len;
unsigned char msgBuf[8];

long unsigned int servo_pos;
long unsigned int min_pos;
long unsigned int max_pos;
byte ID;
byte recID;
int sendID;
byte filtClass;

Servo gripper;

void inline debugPrint(String data)
{
  #ifdef DEBUG_PRINT
  Serial.print(data);
  #endif
}

void inline debugPrintInt(int data) 
{
  #ifdef DEBUG_PRINT
  Serial.print(data);
  #endif
}

void inline debugPrintHex(int data) 
{
  #ifdef DEBUG_PRINT
  Serial.print(data,HEX);
  #endif
}

void inline space() 
{
  #ifdef DEBUG_PRINT
  Serial.print(" ");
  #endif
}

void inline newLine() 
{
  #ifdef DEBUG_PRINT
  Serial.println();
  #endif
}

void setup() 
{
  #ifdef DEBUG_PRINT
  Serial.begin(115200);
  #endif


    
  while (CAN_OK != CAN.begin(CAN_250KBPS, MCP_8MHz))
  {
    debugPrint("CanBeetle initialization failed!!");
    delay(100); 
  }
  
  gripper.attach(A1);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  
  ID = EEPROM.read(0);
  sendID = ID | 0x100;
  filtClass = (ID & 0xF0) + 0xF;
  
  CAN.init_Mask(0, 0, 0x1FF);
  CAN.init_Mask(1, 0, 0x1FF);
  CAN.init_Filt(0, 0, 0xFF);
  CAN.init_Filt(1, 0, filtClass);
  CAN.init_Filt(2, 0, ID);

  unsigned char msg[8];
  CAN.MCP_CAN::sendMsgBuf(sendID,0,0,msg);
  debugPrint("Connected!");
  newLine();

}

void loop() {
  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, msgBuf);
    recID = CAN.getCanId();

    if (msgBuf[0] == 0xAA) //roleCall command
    {
      unsigned char msg[8];
      debugPrint("0xAA roleCall: ");
      debugPrint("sendID: ");
      debugPrint("0x");
      debugPrintHex(sendID);
      newLine();
      newLine();
      CAN.MCP_CAN::sendMsgBuf(sendID,0,0,msg);
      
    } else if (msgBuf[0] == 0x40) //change pos command
    {
      servo_pos = msgBuf[1] << 8 | msgBuf[2];
      if (servo_pos >= (min_pos+25) and servo_pos <= (max_pos-25))
       {
       gripper.write(servo_pos);
        
       debugPrint("servo: ");
       debugPrintInt(servo_pos);
       newLine();
       newLine();
       }
      
    } else if (msgBuf[0] == 0x50)
    {
      min_pos = msgBuf[1] << 8 | msgBuf[2];    
      debugPrint("min_pos: ");
      debugPrintInt(min_pos);

    } else if (msgBuf[0] == 0x51)
    {
      max_pos = msgBuf[1] << 8 | msgBuf[2];    
      debugPrint("min_pos: ");
      debugPrintInt(min_pos);
      newLine();
      newLine();

    } else if (msgBuf[0] == 0x10) //dataRead command
    {
      unsigned char msg[8];
      debugPrint("0x10 dataRead: ");
      int A0data = analogRead(A0);
      debugPrint("A0data: ");
      debugPrintInt(A0data);
      msg[0] = A0data >> 8;
      msg[1] = A0data;

      int A1data = analogRead(A1);
      debugPrint(" A1data: ");
      debugPrintInt(A1data);
      msg[2] = A1data >> 8;
      msg[3] = A1data;

      newLine();
      newLine();
   
      CAN.MCP_CAN::sendMsgBuf(sendID,0,4,msg);
      
    } else if (msgBuf[0] == 0xCE and recID == ID) //changeID command
    {
      debugPrint("0xCE: ");
      
      EEPROM.put(0, msgBuf[1]);
      ID = msgBuf[1];
      sendID = ID | 0x100;

      filtClass = (ID & 0xF0) + 0xF;
      
      CAN.init_Filt(1,0, filtClass);
      CAN.init_Filt(2, 0, ID);

      debugPrint("filtClass: ");
      debugPrint("0x");
      debugPrintHex(filtClass);
      debugPrint(" ID: ");
      debugPrint(" 0x");
      debugPrintHex(ID);
      newLine();
      newLine();
    }
  }

}
