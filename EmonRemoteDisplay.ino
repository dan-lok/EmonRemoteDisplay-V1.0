
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include "LedControl.h"

#define serialMonitor false
/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn 
 pin 11 is connected to the CLK 
 pin 10 is connected to LOAD 
 We have only a single MAX72XX.
 */
LedControl lc=LedControl(7,5,6,2);

unsigned long delaytime=100;

/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/
//#define MAX485_DE      3
//#define MAX485_RE_NEG  2


SoftwareSerial mon(2, 3); // RX, TX
// instantiate ModbusMaster object
ModbusMaster node;

static uint8_t pzemSlaveAddr = 0x01;
int LEDPIN = 13;


/*
  void preTransmission()
  {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
  }

  void postTransmission()
  {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  }
*/

void setup()
{
  // pinMode(MAX485_RE_NEG, OUTPUT);
  //  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  // digitalWrite(MAX485_RE_NEG, 0);
  // digitalWrite(MAX485_DE, 0);

  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,5);
  /* and clear the display */
  lc.clearDisplay(0);
    lc.shutdown(1,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(1,5);
  /* and clear the display */
  lc.clearDisplay(1);
  
  Serial.begin(9600);

//  Serial1.begin(9600);

mon.begin(9600);
  node.begin(pzemSlaveAddr, mon);

  pinMode(13, OUTPUT);
  digitalWrite(LEDPIN,0);
             // Callbacks allow us to configure the RS485 transceiver correctly
             //  node.preTransmission(preTransmission);
             //  node.postTransmission(postTransmission);
}

void loop()
{
int vol ;
int amp ;
int watt ;
uint8_t result;
digitalWrite(LEDPIN,1);
result = node.readInputRegisters(0x0000, 9);
digitalWrite(LEDPIN,0);
if(result == node.ku8MBSuccess){
  
  float voltage = node.getResponseBuffer(0x0000) / 10.0;
vol = int(voltage);
  uint32_t tempdouble = 0x00000000;

  float current;
  tempdouble = node.getResponseBuffer(0x0001);
  tempdouble |= node.getResponseBuffer(0x0002) << 16;
  current = tempdouble /1000.0;
  
  amp = int(current*100);
  
  float power;
  tempdouble = node.getResponseBuffer(0x0003);
  tempdouble |= node.getResponseBuffer(0x0004) << 16;
  power = tempdouble /100.0;
  watt = int(power);
  
  float energy;
  tempdouble = node.getResponseBuffer(0x0005);
  tempdouble |= node.getResponseBuffer(0x0006) << 16;
  energy = tempdouble /100.0;

  float frequency;
  tempdouble = node.getResponseBuffer(0x0007);
  frequency = tempdouble / 10.0;
  
  float powfact;
  tempdouble = node.getResponseBuffer(0x0008);
  powfact = tempdouble / 100.0;

if(serialMonitor){
Serial.print(voltage);
  Serial.print("V   ");

    Serial.print(current);
  Serial.print("A   ");

    Serial.print(power);
  Serial.print("W   ");

  Serial.print(frequency);
  Serial.print("Hz   ");
  
    Serial.print(powfact);
  Serial.print("pf   ");

    Serial.print(energy);
  Serial.print("Wh   ");
  Serial.println();
 /// Serial.println(result, HEX);\
  
}
  

   lc.clearDisplay(1);
  //delay(delaytime);

  ledDisplay(vol,1,4,false);
  ledDisplay(amp,1,0,true);
  ledDisplay(watt,0,0,false);

  } else {
    if(serialMonitor){
      Serial.println("Failed to read modbus");
    }
    
    lc.clearDisplay(1);
   //lc.clearDisplay(0);
    writeArduinoOn7Segment();
    //delay(100);
  }
  

  //writeArduinoOn7Segment();

  delay(200);
}

/*
 * display
 */
void ledDisplay(int value, int line, int start, boolean dec){
  
  
int d = value /1000;  //5000/1000 = 5

if(value > 999){
//  Serial.println(d);
lc.setDigit(line,start +3,d,false);
}

int j = value - d*1000;  //
  int y = j/100;

  if(value > 99){
  //  Serial.println(y);
lc.setDigit(line,start +2,y,dec);
  }

int l = y*100;  //200
 int g = (j - l)/10 ;

 if(value > 9){
 // Serial.println(g);
lc.setDigit(line,start + 1,g,false);
 }

int k = j - ((j/10)*10);
//Serial.println(k);
lc.setDigit(line,start,k,false);

}

/*
 This method will display the characters for the
 word "Arduino" one after the other on digit 0. 
 */
void writeArduinoOn7Segment() {

  lc.setRow(0,3,B00000001);
  delay(delaytime);
  lc.setRow(0,2,B00000001);
  delay(delaytime);
  lc.setRow(0,1,B00000001);
  delay(delaytime);
  lc.setRow(0,0,B00000001);
  delay(delaytime);
  lc.clearDisplay(0);
  delay(delaytime);
} 
