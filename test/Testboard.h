/*
TEST BOARD
*/

#include <Arduino.h>
//#include <mcp_can.h>
//#include <mcp_can_dfs.h>
//#include <SPI.h>

// AnalogInputs
const int MAFMAPpin =A0;  //Azul MAF/MAP signal in A0 (0-5V) (option: Advance Measurm)
const int MAP2pin = A1;  //Laranja MAP2 signal in A1 (0-5V) (option: Clutch Switch)
const int IATpin = A2;  //Verde IAT sensor in A2 (5-0V) (option: CoolantTemp sensor)
const int TPSpin = A3;  //Vermelho TPS signal in A3 (0-5V signal)
const int AFRpin = A4;  //Branco AFR signal in A4 (0-5V signal)
const int BRKPressurePin= A5;  //Preto BreakPressure

//DIP Switch : ON is OV and OFF is 5V from pullups. Logic value is inverted in 74HC165 reading.
bool inputisAdvance=false; //ergo: is not MAP/MAF; logic value=!DIP[0]; inputA0
bool inputisMAP=false; //ergo: is not MAF; logic value = DIP[1] inputA0
bool inputisClutchSW=false; //ergo: is not MAP2; logic value = !DIP[2]; inputA1
bool inputisCoolantTemp=false; //ergo: is not IAT; logic value = DIP[3]; inputA4
bool inputisBRKSwitch=false; //ergo: is not BRKPressure; logic value = DIP[4]; software
bool useRPMLimiter=false; //logic value =!DIP[5]; software
bool mazdaECUOn=false; //logic value = !DIP[6]; sofware
bool rpmfromIGN=false; //ergo: RPM fromECU; logicvalue = DIP[7]; software

// DigitalInputs
const int RPMpin = 3;  //Preto: Must be this pin to use free interrupt for RPM counting

//Digital Outputs
const int RPMLimiterpin=8; //verde: testar pois parece afectado pelo estado do pin D7

//Digital Outputs Internal connection to 74HC165
int loadPin=7; //Pin7 Branco: HC1-Parallel Load when low shift when High 
int clockEnablePin=4;//Pin 4 Amarelo: HC15-CE or ClockInhibit when High, no changes in output
int dataInPin=5; //Pin 5 Laranja: HC7-Q7 Complementary Serial Output (negated)
int clockINPin=6;//Pin6 Azul: HC-2CP

byte Dipswitch1=B00000000;

// Serial.println (unsigned char array)
void printA (unsigned char myStr[8]) 
{
  String msg="";
  for (int i = 0; i < 8 - 1; i++) 
  {
    msg=msg+myStr[i] + ",";
  }
    Serial.println(msg);
    delay(10);
}

void readSensorDipswitch ()  
{

//Dipswitch1=!B00000000;
//Write pulse to load pin;
  digitalWrite(loadPin,LOW); //Pin7 Branco: HC1-Parallel Load when low shift when High
  delayMicroseconds(5);
  digitalWrite(loadPin,HIGH);
  delayMicroseconds(5);
  //Get data from 74HC165
  digitalWrite(clockEnablePin, LOW);//Pin 4 Amarelo: HC15-CE or ClockInhibit when High, no changes in output
  //byte Dipswitch1=shiftIn(dataInPin, clockINPin, MSBFIRST);//Pin 5 Laranja: HC7-Q7 Complementary Serial Output (negated)
  for (int i = 0; i < 8; i++)
  {
    bitWrite(Dipswitch1,i,digitalRead(dataInPin));
    digitalWrite(clockINPin, LOW);//Pin6 Azul: HC-2CP
    digitalWrite(clockINPin, HIGH);//Pin6 Azul: HC-2CP
    //Serial.println(Dipswitch1, BIN);
    //delay(100);
  }
  digitalWrite(clockEnablePin, HIGH);
  digitalWrite(clockINPin, LOW);//Pin6 Azul: HC-2CP
  Serial.print("FINAL DIP: ");
  Serial.println(Dipswitch1, BIN);
} 

void readAnalogInputs()
{
  float v1=0;
  for (int i = 0; i < 6; i++)
  {
    v1=analogRead(i);
    v1=analogRead(i);
    v1=5.0*v1/1024;
    Serial.print("A");Serial.print(i);Serial.print(": ");Serial.print(v1);Serial.println("V");
  }
  Serial.println("");Serial.println("");Serial.println("");Serial.println("");Serial.println("");
  Serial.println("");Serial.println("");Serial.println("");
  delay(1000);
}


void setup()
{
  Serial.begin(9600);
  pinMode(MAFMAPpin, INPUT);
  pinMode(MAP2pin, INPUT);
  pinMode(IATpin, INPUT);
  pinMode(TPSpin, INPUT);
  pinMode(AFRpin, INPUT);
  pinMode(BRKPressurePin, INPUT);
  pinMode(RPMpin, INPUT);
  pinMode(dataInPin, INPUT);
  //for HC165
  pinMode(loadPin, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockINPin, OUTPUT);
  pinMode(RPMLimiterpin, OUTPUT);
  
  digitalWrite(loadPin, HIGH);
  digitalWrite(clockINPin, LOW);
  digitalWrite(clockEnablePin, HIGH);
  digitalWrite(RPMLimiterpin, HIGH);
  delay(500);
  digitalWrite(RPMLimiterpin, LOW);
  Serial.println("setup done");
  delay(10);
}


void loop() 
{
  readSensorDipswitch();
  delay(1000); 
  readAnalogInputs();
  
} 