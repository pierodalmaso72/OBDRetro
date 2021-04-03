/*
Includes code from 'Tachometer using micros' from InterlinkKnight Last update: 05/23/2019
used to count the frequency for RPM

Mazdadmode: if False simple OBD Mode0x01 if true sim Mazda mx5 >2005
*/

#include <Arduino.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

MCP_CAN CAN(9); // Set CS to pin 9
unsigned long int canId = 0x000;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
String BuildMessage = "";
int MSGIdentifier = 0;

// AnalogInputs
const int MAFMAPpin =A0;  //Azul MAF/MAP signal in A0 (0-5V) (option: Advance Measurm)
const int MAP2pin = A1;  //Laranja MAP2 signal in A1 (0-5V) (option: Clutch Switch)
const int IATpin = A2;  //Verde IAT sensor in A2 (5-0V) (option: CoolantTemp sensor)
const int TPSpin = A3;  //Vermelho TPS signal in A3 (0-5V signal)
const int AFRpin = A4;  //Branco AFR signal in A4 (0-5V signal)
const int BRKPressurePin= A5;  //Preto BreakPressure

//DIP Switch : ON is OV and OFF is 5V from pullups. Logic value is inverted in 74HC165 reading.
byte Dipswitch1=B11111111;
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
int dataInPin=5; //Pin 5 Laranja: HC7-Q7 Complementary Serial Output (negated)
int clockINPin=6;//Pin6 Azul: HC-2CP

//GLOBALS
bool answering=false;
bool sending=false;

const bool sCAN=false;

// RPM Calibration:
byte const PulsesPerRevolution = 4;  // Set how many pulses there are on each revolution. Default: 2.
const unsigned long ZeroTimeout = 200000;  // For high response time, a good value would be 100000. For reading very low RPM, a good value would be 300000.
byte const numReadings = 2;  // Calibration for smoothing RPM: Number of samples for smoothing. The higher, the more smoothing, but it's going to react slower to changes. 1 = no smoothing. Default: 2.
// RPM Variables:
volatile unsigned long LastTimeWeMeasured;  // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long PeriodBetweenPulses = ZeroTimeout+1000;  // Stores the period between pulses in microseconds.// It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
volatile unsigned long PeriodAverage = ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.// It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
unsigned long FrequencyRaw;  // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
unsigned long FrequencyReal;  // Frequency without decimals.
unsigned long RPM;  // Raw RPM without any processing.
unsigned int PulseCounter = 1;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.
unsigned long PeriodSum; // Stores the summation of all the periods to do the average.
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;  // Stores the last time we measure a pulse in that cycle.
unsigned long CurrentMicros = micros();  // Stores the micros in that cycle.
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;  // Stores the extra value added to the ZeroTimeout to debounce it.
// RPM Variables for smoothing tachometer:
unsigned long readings[numReadings];  // The input.
unsigned long readIndex;  // The index of the current reading.
unsigned long total;  // The running total.
unsigned long average;  // The RPM value after applying the smoothing.
int average2;  // limpar ... The RPM value after applying the smoothing.
int currentRPM=0;


/*FUNCTIONS ARE BELLOW (SOME NOT USED ARE IN END OF FILE)
============================================================*/
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

void Pulse_Event()  // The interrupt runs this to calculate the period between pulses:
{

  PeriodBetweenPulses = micros() - LastTimeWeMeasured;  // Current "micros" minus the old "micros" when the last pulse happens. the period (microseconds) between both pulses. avoids the overflow of the "micros".
  LastTimeWeMeasured = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.
  if(PulseCounter >= AmountOfReadings)  // If counter for amount of readings reach the set limit:
  {
    PeriodAverage = PeriodSum / AmountOfReadings;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounter = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

    // Change the amount of readings depending on the period between pulses. To be very responsive, ideally we should read every pulse but at Higher speeds .... 
    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
    AmountOfReadings = RemapedAmountOfReadings;  // Set amount of readings as the remaped value.
  }
  else
  {
    PulseCounter++;  // Increase the counter for amount of readings by 1.
    PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
  }

//Serial.println("Interrupt Processed!");
}  // End of Pulse_Event.

void startCAN() // keywords : CAN_250KBPS CAN_500KBPS CAN_1000KBPS
{
START_INIT:
  if (CAN_OK == CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS Shield init ok!");
  }
  else
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
    goto START_INIT;
  }
}

void readSensorDipswitch ()  
{
//Write pulse to load pin;
  digitalWrite(loadPin,LOW);
  delayMicroseconds(5);
  digitalWrite(loadPin,HIGH);
  delayMicroseconds(5);
  //Get data from 74HC165
  for (int i = 0; i < 8; i++)
  {
    bitWrite(Dipswitch1,i,digitalRead(dataInPin));
    digitalWrite(clockINPin, LOW);
    digitalWrite(clockINPin, HIGH);
    //Serial.println(Dipswitch1, BIN);
  }
  digitalWrite(clockINPin, LOW);
  Serial.print("DIP SWITCH: ");Serial.println(Dipswitch1, BIN);
//Set vars with dipswitches state
inputisAdvance =      bitRead(Dipswitch1,7);// ! bitRead(Dipswitch1,0); 
inputisMAP =          bitRead(Dipswitch1,6); 
inputisClutchSW =     bitRead(Dipswitch1,5);//! bitRead(Dipswitch1,2); 
inputisCoolantTemp =  bitRead(Dipswitch1,4); 
inputisBRKSwitch =    bitRead(Dipswitch1,3);
useRPMLimiter =       bitRead(Dipswitch1,2); //! bitRead(Dipswitch1,5); 
mazdaECUOn =          bitRead(Dipswitch1,1);// ! bitRead(Dipswitch1,6); 
rpmfromIGN =          bitRead(Dipswitch1,0); 
/*
Serial.print(inputisAdvance);  
Serial.print(inputisMAP); 
Serial.print(inputisClutchSW);
Serial.print(inputisCoolantTemp);
Serial.print(inputisBRKSwitch);
Serial.print(useRPMLimiter);
Serial.print(mazdaECUOn);
Serial.println(rpmfromIGN);
delay(2000);
*/
}

int RPMCalc() 
{
if(rpmfromIGN==true) 
{
  LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
  CurrentMicros = micros();  // Store the micros() in a variable.
  if(CurrentMicros < LastTimeCycleMeasure)
  {
    LastTimeCycleMeasure = CurrentMicros;
  }
  // Calculate the frequency:
  FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.
  // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
  if(PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra)
  {  // If the pulses are too far apart that we reached the timeout for zero:
    FrequencyRaw = 0;  // Set frequency as 0.
    ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else
  {
    ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }
  FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
  // Calculate the RPM:
  RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by 60 seconds to get minutes.
  RPM = RPM / 10000;  // Remove the decimals.
  // Smoothing RPM:
  total = total - readings[readIndex];  // Advance to the next position in the array.
  readings[readIndex] = RPM;  // Takes the value that we are going to smooth.
  total = total + readings[readIndex];  // Add the reading to the total.
  readIndex = readIndex + 1;  // Advance to the next position in the array.
  if (readIndex >= numReadings)  // If we're at the end of the array:
  {
    readIndex = 0;  // Reset array index.
  }
  // Calculate the average:
  average = total / numReadings;  // The average value it's the smoothed result.
  average2=round(average)/10; // para so mostar dezenas
  average2=average2*10;
  //Serial.print(average2); Serial.println(" RPM");
  return average2;
} 
  else //RPM signal is 0-5V analog signal - Need to find free analogic pin and use it: TO BE DONE IF EVER NECESSARY 
  {
    return average2;
  } 
}

unsigned char* getAirFlow (bool mazdamode,bool loadmode, int reps, bool demo)
{
  int v1=0;
  int v2=0;
  int temp;
  unsigned char load=0;
  unsigned char airflow=0;
  for (int i = 0; i < reps; i++)
  {
    //temp=analogRead(MAFMAPpin);
    v1=v1+analogRead(MAFMAPpin);
    if (inputisClutchSW==false) 
    {temp=analogRead(MAP2pin);v2=analogRead(MAP2pin);}
    if(reps>1){delay(10);}
  }
  v1=v1/reps/4;
  v2=v2/reps/4;
  if (inputisClutchSW==false && v2>v1) {v1=v2;} //v2=255 para debug
  load=v1; airflow=v1;
  if (demo==true) {load=random(0,25); airflow=load;};//Serial.println(airflow); 
  if (mazdamode==true) 
    {static unsigned char Load[8] ={3, 98, 0, 67, 0, load, 0, 0 }; Load[5]=load; return Load;}
  if (mazdamode==false && loadmode==true) 
    {static unsigned char Load[8] = {4, 65, 4, load, 0, 0, 0, 0,};Load[3]=load;return Load;}
  if (inputisMAP==false && loadmode==false)    
    {static unsigned char  MAF[8] = {4, 65, 16, airflow, 224, 185, 147, 0}; MAF[3]=airflow; return MAF;}
  if (inputisMAP==true && loadmode==false)     
    {static unsigned char  MAP[8] = {4, 65, 11, airflow, 224, 185, 147, 0}; MAP[3]=airflow; return MAP;}
}

unsigned char* getNTC(bool mazdamode, int reps, bool demo) 
{
  int v1=0;
  int temp;
  unsigned char temperature=0;
  for (int i = 0; i < reps; i++)
  {
    //temp=analogRead(IATpin);
    v1=v1+analogRead(IATpin);
    if(reps>1){delay(10);}
    }
  v1=v1/reps;
  //v1 = -v1 * (8.2 / 98) + 68.07551 + 40;
  v1=-v1/4+255;
  temperature=v1;
  if (demo==true) {temperature=-random(0,255)+255;} 
  if (mazdamode==false && inputisCoolantTemp==false) 
    {static unsigned char IATSensor[8] = {4, 65, 15, temperature, 0, 185, 147, 0}; IATSensor[3]=temperature; return IATSensor;}
  else if(mazdamode==true && inputisCoolantTemp==false) 
    {static unsigned char IATSensor[8] = {0, 0, 0, 0, temperature, 0, 0, 0}; IATSensor[4]=temperature; return IATSensor;}
  else if (mazdamode==false && inputisCoolantTemp==true) 
    {static unsigned char CoolantTemp[8] = {4, 65, 5, temperature, 0, 185, 147,0}; CoolantTemp[3]=temperature; return CoolantTemp;}
  else if  (mazdamode==true && inputisCoolantTemp==true) 
    {static unsigned char CoolantTemp[8] = {temperature, 0, 0, 0, 0, 0, 0, 0}; CoolantTemp[0]=temperature; return CoolantTemp;} 
}

unsigned char* getTPS(bool mazdamode,int reps, bool demo)
{
  int v1=0;
  int temp;
  unsigned char tps=0;
  for (int i = 0; i < reps; i++) 
  {
    //temp=analogRead(TPSpin);
    v1=v1+analogRead(TPSpin);
    if(reps>1){delay(10);}
    }
  v1=v1/reps/4;
  tps=v1;
  if (demo==true) {tps=random(0,200);} //Serial.println(tps); 
  if (mazdamode==true)  {static unsigned char TPS[8] = {tps, 255, 255, 255, 255, 255, 255, 255};TPS[0]=tps; return TPS;}
  else {static unsigned char TPS[8] = {4, 65, 17, tps, 0, 185, 147, 0};TPS[3]=tps; return TPS;}
}

unsigned char* getLambda(bool mazdamode, int pid,int reps, bool demo)
{
  int v1=0;
  int temp;
  unsigned char lambda=0;
  for (int i = 0; i < reps; i++) 
  {
    //temp=analogRead(AFRpin);
    v1=v1+analogRead(AFRpin);
    if(reps>1){delay(10);}
  }
  v1=v1/reps/4;
  lambda=v1;  
  if (demo==true) {lambda=random(120,130);} //Serial.println(lambda);
  if (mazdamode==false && pid==20)  
   {static unsigned char Lambda[8] = {4, 65, 20, lambda, 255, 0, 0, 0}; Lambda[3]=lambda;return Lambda;}//narrow
  if (mazdamode==false && pid==36) 
    {static unsigned char Lambda[8] = {4, 65, 36, lambda, 0, 0, 0, 0}; Lambda[3]=lambda; return Lambda;}//wide
  if (mazdamode==false && pid==52)  
   {static unsigned char Lambda[8] = {4, 65, 52, lambda, 0, 0, 0, 0}; Lambda[3]=lambda; return Lambda;}//wide
  else 
   {static unsigned char Lambda[8] = {4, 65, 36, lambda, 0, 0, 0, 0}; Lambda[3]=lambda; return Lambda;}
}

unsigned char* getRPM(bool mazdamode, int reps, bool demo)
{
  float temp;
  int temp2;
  unsigned char rpm;
  //temp2=RPMCalc();
  temp2=currentRPM;
  //Serial.print(temp2);
  //Serial.println(" RPM (da funcao)");
  temp=255.0*temp2/16300.0;
  //Serial.print(temp);
  //Serial.println(" RPM float[0-255}");
  rpm=min(temp,255);
  if (demo==true) {rpm=random(35,90);} 
  if (mazdamode==true)  {static unsigned char RPM[8] = {rpm, 255, 255, 255, 255, 255, 255, 255}; RPM[0]=rpm; return RPM;}
  else {static unsigned char RPM[8] = {4, 65, 12, rpm, 0, 0, 0, 0}; RPM[3]=rpm; return RPM;}
};

unsigned char* getAdvance(bool mazdamode,int reps, bool demo)
{
  unsigned char advance=0;
  int v1=0;
  int temp;
  for (int i = 0; i < reps; i++) 
  {
    //temp=analogRead(MAFMAPpin);
    v1=v1+analogRead(MAFMAPpin);
    if(reps>1){delay(10);}
  }
  v1=v1/reps/4;
  advance=v1;
  if (demo==true) {advance=random(110,155);}
  if (mazdamode==false) {static unsigned char Advance[8] = {4, 65, 14, advance, 0, 185, 147,0}; Advance[3]=advance;return Advance;}
  else {static unsigned char Advance[8] = {advance, 0, 0, 0, 0, 0, 0, 0}; Advance[0]=advance; return Advance;}
}

unsigned char* getBreakData(int reps, bool demo)
{
  int v1=0;
  int temp;
  unsigned char bp=0;
  unsigned char bs=0;
  for (int i = 0; i < reps; i++) 
  {
    //temp=analogRead(BRKPressurePin);
    v1=v1+analogRead(BRKPressurePin);
    if(reps>1){delay(10);}
  }
  v1=v1/reps/4;
  if (inputisBRKSwitch) {bs=v1;} else {bp=v1;}
  if (demo==true) {bp=random(0,100);bs=200*random(0,1);}
  static unsigned char BreakD[8] = {bp, 0, bs, 0, 0, 0, 0, 0}; BreakD[0]=bp; BreakD[2]=bs;return BreakD;
}

unsigned char* getClutchSwitch(int reps, bool demo)
{
  int v1=0;
  int temp;
  unsigned char sw=0;
  for (int i = 0; i < reps; i++) 
  {
    //temp=analogRead(MAP2pin);
    v1=v1+analogRead(MAP2pin);
    if(reps>1){delay(10);}
    }
  v1=v1/reps/4;
  if (v1<100) {v1=0;}
  else {v1=255;}
  sw=v1;
  static unsigned char ClutchSw[8] = {0, sw, 0, 0, 0, 0, 0, 0}; ClutchSw[1]=sw;return ClutchSw;
}

void CANAnswer()
{
  //unsigned long StartTime = millis();
  CAN.readMsgBuf(&len, buf);
  canId = CAN.getCanId();
  for (int i = 0; i < len; i++)
  {
    BuildMessage = BuildMessage + buf[i] + ",";
  }

  //Serial.print("<");
 // Serial.print(canId, HEX);
  //Serial.print(",");
 // Serial.println(BuildMessage);

  //PID answers
  if(BuildMessage=="2,1,4,0,0,0,0,0," && !inputisAdvance)     {CAN.sendMsgBuf(0x7E8, 0, 8, getAirFlow(false,true,1, false)); //Serial.println(">01 EngineLoad");
  }
  if(BuildMessage=="2,1,5,0,0,0,0,0," && inputisCoolantTemp)  {CAN.sendMsgBuf(0x7E8, 0, 8, getNTC(false,1,false)); //Serial.println(">01 CoolantTemp");
  }
  if(BuildMessage=="2,1,11,0,0,0,0,0," && !inputisAdvance)    {CAN.sendMsgBuf(0x7E8, 0, 8, getAirFlow(false,false,1,false)); //Serial.println(">01 MAP");
  }
  if(BuildMessage=="2,1,12,0,0,0,0,0,")                       {CAN.sendMsgBuf(0x7E8, 0, 8, getRPM(false,1,false)); //Serial.println(">01 RPM");
  }
  if(BuildMessage=="2,1,14,0,0,0,0,0," && inputisAdvance)     {CAN.sendMsgBuf(0x7E8, 0, 8, getAdvance(false,1,false)); //Serial.println(">01 Advance");
  }
  if(BuildMessage=="2,1,15,0,0,0,0,0," && !inputisCoolantTemp){CAN.sendMsgBuf(0x7E8, 0, 8, getNTC(false,1,false)); //Serial.println(">01 IAT");
  }
  if(BuildMessage=="2,1,16,0,0,0,0,0," && !inputisAdvance)    {CAN.sendMsgBuf(0x7E8, 0, 8, getAirFlow(false,false,1, false)); //Serial.println(">01 MAF");
  }
  if(BuildMessage=="2,1,17,0,0,0,0,0,")                       {CAN.sendMsgBuf(0x7E8, 0, 8, getTPS(false,1,false)); //Serial.println(">01 TPS");
  }
  if(BuildMessage=="2,1,20,0,0,0,0,0,")                       {CAN.sendMsgBuf(0x7E8, 0, 8, getLambda(false,20,1,false)); //Serial.println(">01 AFR B1Volts (narrow)");
  }
  if(BuildMessage=="2,1,52,0,0,0,0,0,")                       {CAN.sendMsgBuf(0x7E8, 0, 8, getLambda(false,52,1,false)); //Serial.println(">01 AFR");
  }
  if(BuildMessage=="2,1,36,0,0,0,0,0,")                       {CAN.sendMsgBuf(0x7E8, 0, 8, getLambda(false,36,1,false)); //Serial.println(">01 Lambda");
  }
  
  //Mode0x22 answers
  if(BuildMessage=="3,34,0,67,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getAirFlow(true,true,1,false)); //Serial.println(">22h Load");
  }
  if(BuildMessage=="3,34,0,14,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getAdvance(true,1,false)); //Serial.println(">22h IgnAdv");
  }


  //OBD2 MODE0x01 PID 0 DATA
  unsigned char SupportedPID00[8] =     {65, 0, 255, 255, 255, 255, 0, 0}; //Perfeito
  unsigned char SupportedPID20[8] =     {65, 32, 255, 255, 255, 255, 0, 0};    //Perfeito
  unsigned char SupportedPID40[8] =     {65, 64, 255, 255, 255, 255, 0, 0};
  unsigned char SupportedPID60[8] =     {65, 96, 255, 255, 255, 254, 0, 0};
  unsigned char SupportedPID80[8] =     {65, 128, 255, 255, 255, 254, 0, 0};
  unsigned char SupportedPID00v4[8] =   {4, 65, 0, 255, 255, 255, 254, 0};
  unsigned char MilCleared[7] =         {4, 65, 63, 34, 224, 185, 147};
  //Mode0x01 PID0x0
  if(BuildMessage=="2,1,0,0,0,0,0,0,")          {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00); delay(5); //Serial.println(">01 PID0");
                                                 /*CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00v4); Serial.println(">01 PID0 Extra");*/
                                                 }
  if(BuildMessage=="2,1,0,153,153,153,153,153,"){CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00); delay(5); // Serial.println(">01 PID0");
                                                 CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00v4);// Serial.println(">01 PID0 Extra");
                                                 }
  if(BuildMessage=="2,1,32,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID20); //Serial.println(">01 PID0 0-20h");
                                                }
  if(BuildMessage=="2,1,64,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID40); //Serial.println(">01 PID0 20-40h");
                                                }
  if(BuildMessage=="2,1,96,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID60); //Serial.println(">01 PID0 40-60h");
                                                }
  if(BuildMessage=="2,1,128,0,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID80); //Serial.println(">01 PID0 60-80h");
                                                }
  if(BuildMessage=="2,1,1,0,0,0,0,0,")          {CAN.sendMsgBuf(0x7E8, 0, 7, MilCleared); //Serial.println(">01 PID1 MIL");
                                                }
  
  //NOT USED
  //if(BuildMessage=="2,1,61,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,2,1,false)); Serial.println(">01 CTA2Temp");}
  //if(BuildMessage=="2,1,13,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getvSpeed(1,false)); Serial.println(">01 Speed");}
  //if(BuildMessage=="2,1,47,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getFuelLevel(1,false)); Serial.println(">01 FuelLev");}
  //if(BuildMessage=="2,1,60,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,1,1,false)); Serial.println(">01 CAT1Temp");}
  //if(BuildMessage=="2,1,62,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,3,1,false)); Serial.println(">01 CAT3Temp");}
  //if(BuildMessage=="2,1,63,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,4,1,false)); Serial.println(">01 CAT4Temp");}
  //if(BuildMessage=="2,1,66,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getBatteryV(1,false)); Serial.println(">01 BatteryV");}
  //if(BuildMessage=="2,1,70,0,0,0,0,0,") {CAN.sendMsgBuf(0x7E8, 0, 8, getAmbientTemp(1,false)); Serial.println(">01 AirTemp");}

  //if(BuildMessage=="3,34,0,60,0,0,0,0,")    {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(true,1,1,false)); Serial.println(">22h FuelLev");/*CAN_DataFrequency("Mode22");*/}
}

//Broadcast Rates MazdaSim
int sendHcounter = 0; // com BDR=10 e HxL=10 40-50HZ/7HZ com  BDR=0 e HxL=10:314HZ/35HZ
int sendLcounter = 0;
int SendHRate=1; // Main Parameter for MAZDA SIM broadcast 
int SendLxHRate=5; // number of times that low priority msgs are silenced SendRate=10,SendLxH=10:H45Hz,L7Hz); SendRate=0,SendHxL=10->H314Hz,L35Hz; SendRate=100,SendHXL=5->H5Hz,L1hz (com 3 variaveis pedidas em modo01 a 7hz)

void MazdaECUbrodcast()
{
  if(mazdaECUOn==true) 
  {
    if(sendHcounter==SendHRate)
    {
      unsigned char* RPM=getRPM(true,1,false);
      unsigned char rpm= RPM[0];
      unsigned char* TPS=getTPS(true,1,false);
      unsigned char tps= TPS[0];
      unsigned char mazdaspeed=0;
      unsigned char Mazda201[8] = {rpm, 1, 200 ,200, mazdaspeed, 0, min(tps,200), 0};
      CAN.sendMsgBuf(0x201, 0, 8, Mazda201);
      //printA(Mazda201);Serial.println("SentH: RPM,TPS, SPEED");
      delay(1);
      
      CAN.sendMsgBuf(0x85, 0, 8, getBreakData(1,false)); 
      //Serial.println("SentH: Break data");
      delay(1);
      CAN.sendMsgBuf(0x231, 0, 8, getClutchSwitch(1,false)); 
      //Serial.println("SentH: clutch");
      delay(1);
      sendLcounter++;
      sendHcounter=0;
      //CAN.sendMsgBuf(0x4b0, 0, 8, Mazdawheelspeed); Serial.println(">H WheelSpeed");
      //CAN_DataFrequency("MazdaH");
    }
    else {sendHcounter++;}
    if(sendLcounter==SendLxHRate) 
      {
       //Put here LOW Priority msgs. sent quitetimes lesss
        sendLcounter=0;
        CAN.sendMsgBuf(0x421, 0, 8, getNTC(true,1,false)); //Serial.println("SentL: CoolantTemp");
        delay(1);
        CAN.sendMsgBuf(0x240, 0, 8, getNTC(true,1,false)); //Serial.println("SentL: IAT");
        delay(1);
      }
    //CAN_DataFrequency("MazdaL");
  }
}

//Calculo do Tempo de resposta millis()
unsigned long StartTime = 0;
unsigned long EndTime = 0;
unsigned long DurationMillis = 0;
float BroadcastFreq = 0;
String BDFreqMarker = ""; //"MazdaH" "MazdaL" "Response" "Mode1" "Mode22"

void CAN_DataFrequency(String mark)
{
  if (mark == BDFreqMarker)
  {
    unsigned long t = millis();
    if (StartTime == 0)
    {
      StartTime = t;
      EndTime = 0;
    }
    else
    {
      EndTime = t;
      DurationMillis = EndTime - StartTime;
      BroadcastFreq = 1000.0 / DurationMillis;
      Serial.print(mark);
      Serial.print("- Frequency: ");
      Serial.print(BroadcastFreq);
      Serial.print("(");
      Serial.print(DurationMillis);
      Serial.println(" ms)");
      StartTime = 0;
      EndTime = 0;
    }
  }
}

void RPMLimiter (int rpm)
{
  //currentRPM=RPMCalc();
  Serial.println(currentRPM);
  //if (useRPMLimiter==1 && currentRPM>(rpm-200)) 
  if (currentRPM>(rpm-200)) 
  {digitalWrite(RPMLimiterpin, HIGH);delay(1);
    //Serial.println(currentRPM);
  }
  else {digitalWrite(RPMLimiterpin, LOW); delay(1);}
}

void MazdaCanScan(int startHex, int EndHex, byte *msg, int dlay)
{
  for (int id = startHex; id < EndHex; id++)
  {
    CAN.sendMsgBuf(id, 0, 8, msg);
    Serial.print("Can ID :  ");
    Serial.println(id);
    delay(dlay);
  }
}

void setup()
{
  Serial.begin(115200);
  startCAN();
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
  pinMode(clockINPin, OUTPUT);
  digitalWrite(loadPin, HIGH);
  digitalWrite(clockINPin, LOW);

  pinMode(RPMLimiterpin, OUTPUT);
  digitalWrite(RPMLimiterpin, HIGH);
  readSensorDipswitch();
  Serial.print("DIP1 InputisAdvance: "); Serial.println(inputisAdvance); 
  Serial.print("DIP2 inputisMAP: "); Serial.println(inputisMAP); 
  Serial.print("DIP3 InputisClutch: "); Serial.println(inputisClutchSW); 
  Serial.print("DIP4 InputisCoolantTemp: "); Serial.println(+inputisCoolantTemp); 
  Serial.print("DIP5 InputisBRKSwitch: "); Serial.println(inputisBRKSwitch); 
  Serial.print("DIP6 useRPMLimiter: "); Serial.println(useRPMLimiter);
  Serial.print("DIP7 mazdaECUOn: "); Serial.println(mazdaECUOn); 
  Serial.print("DIP8 rpmfromIGN: "); Serial.println(rpmfromIGN);

  attachInterrupt(digitalPinToInterrupt(RPMpin), Pulse_Event, FALLING);  // Enable interruption pin 3 when going from HIGH to LOW.
  delay(20000);  // We sometimes take several readings of the period to average. Since we don't have any readings stored we need a high enough value in micros() so if divided is not going to give negative values.

}

void loop() 
{
  
  currentRPM=RPMCalc();
  RPMLimiter(7500);
  
  //OBD ANSWER MODE IF REQUESTS EXIST
  if(CAN_MSGAVAIL == CAN.checkReceive()) 
  {
    answering=true;
    delay(0); //Not good for RPM stability; slows ELM data request in mode 1 (by delaying answer)
    CANAnswer();
    BuildMessage="";
    //Serial.println("RESPONSE FINISHED");
    //delay(100); //Good for RPM stability.breeding time for RPM mais rapido que o pedido do ELM no Scantool1.13
  } 

  //MAZDASIM: BROADCAST MAZDA FORMAT DATA TO CAN_BUS
  if(mazdaECUOn) {MazdaECUbrodcast();}

  // CANSCAN TO REVERSE ENGINEER
  if(sCAN==true) {byte Mazdatest[8] = {200,0,0,0,0,0,0,0}; MazdaCanScan( 0x0, 0xFF, Mazdatest, 5);} //Mazda SCAN IDs for reverse enginnering - RESPONDING IDs ON AIM
    
  currentRPM=0;
}






  /*Serial.print("DIP1 InputisAdvance: "); Serial.println(inputisAdvance); 
  Serial.print("DIP2 inputisMAP: "); Serial.println(inputisMAP); 
  Serial.print("DIP3 InputisClutch: "); Serial.println(inputisClutchSW); 
  Serial.print("DIP4 InputisCoolantTemp: "); Serial.println(+inputisCoolantTemp); 
  Serial.print("DIP5 InputisBRKSwitch: "); Serial.println(inputisBRKSwitch); 
  Serial.print("DIP6 useRPMLimiter: "); Serial.println(useRPMLimiter);
  Serial.print("DIP7 mazdaECUOn: "); Serial.println(mazdaECUOn); 
  Serial.print("DIP8 rpmfromIGN: "); Serial.println(rpmfromIGN); 
  delay(500); 
  */
  /*//TEST MODE OVERRIDING readsensorDip  
  inputisAdvance=false;
  inputisMAP = false; //with true: MAP sensor if is not MAF
  inputisClutchSW = true; //with true no MAP2 sensor instead: clutch
  inputisCoolantTemp = false; //NTC is intaketemperature
  inputisBRKSwitch = false;
  useRPMLimiter = false;
  mazdaECUOn = false;
  rpmfromIGN = true;
  */
 


/*MAZDA ID
MAZDA ID
0x201: RPM(0); Speed (4); TPS(6)
0x4B0: Wheelspeed LF(0,1),RF(2,3),LR(4,5),RR(6,7)
0x85: Breakpressure(0); BreakSwitch(2)
0x421: Coolant Temperature(0)
0x215: Pedal position (0)
0x81h: Steering Deg (3 - Graus rotacao) (2 - direccao)
0x240h: IAT (4)
0x420: Oil Switch (6)
0x213h: Clutch Switch (1)
Fuel Level (PID 0x2F) e AFR(Pid 0x34) são perguntados em modo0x1
Absolute Load(PID 0x0043); SparkAdvance (PID 0x000E); Cat1Temperature (PID 003C) são perguntados em modo0x22
*/

//THESE FUCNTIONS ARE NOT CURRENTLY BEING USED SO MOVED TO THE BOTTOM TO UNCLUTTER
/*
bool noLoad=true;
bool novSpeed=true;
bool noFuelLevel=true;
bool noCatTemp=true;
bool noBatteryV=true;
bool noAmbientTemp=true;

unsigned char* getvSpeed(int rep, bool demo)
{
  unsigned char vspeed=0;
  if (demo==true) {vspeed=random(0,255);}
  static unsigned char VSpeed[8] = {4, 65, 13, vspeed, 224, 185, 147, 0}; return VSpeed;
} 

unsigned char* getFuelLevel(int rep, bool demo)
{
  unsigned char fuel=0;
  if (demo==true) {fuel=random(0,255);}
  static unsigned char FuelLevel[8] = {4, 65, 47, fuel, 224, 185, 147,0 };
  return FuelLevel;
}

unsigned char* getCATTemp(bool mazdamode,int CAT, int rep, bool demo)
{ 
  unsigned char cattemp=0;
  if (demo==true) {cattemp=random(1,255);}
  if (mazdamode==false) 
  {
    if (CAT==1) {static unsigned char CATTemp[8] ={4, 65, 60, cattemp, 224, 185, 147, 0}; return CATTemp;}
    if (CAT==2) {static unsigned char CATTemp[8] ={4, 65, 61, cattemp, 224, 185, 147, 0}; return CATTemp;}
    if (CAT==3) {static unsigned char CATTemp[8] ={4, 65, 62, cattemp, 224, 185, 147, 0}; return CATTemp;}
    if (CAT==4) {static unsigned char CATTemp[8] ={4, 65, 63, cattemp, 224, 185, 147, 0}; return CATTemp;}
  }
  else {static unsigned char CATTemp[8] = {4, 98, 0, 60, cattemp, 1, 1, 1 }; return CATTemp;} //Mazda MX5 mode 0x22h
}

unsigned char* getBatteryV(int rep, bool demo)
{
  static char bv=0;
  if (demo==true) {bv=random(48,48);}
  static unsigned char BatteryVoltage[8] = {4, 65, 66, bv, 212, 0, 0, 0}; 
  return BatteryVoltage;
}

unsigned char* getAmbientTemp(int rep, bool demo)
{
  static char at=0;
  if (demo==true) {at=random(1,200);}
  static unsigned char AmbientAirTemp[8] = {4, 65, 70, at, 0, 185, 147, 0};
  return AmbientAirTemp;
}
*/
/*  //VALORES ALEOTORIOS TESTE PARA OBD2
    unsigned char rndCoolantTemp=,false;
    unsigned char rndRPM=random(35,90);
    unsigned char rndSpeed=random(0,255);
    unsigned char rndIAT=random(0,255);
    unsigned char rndMAF=random(0,10);
    unsigned char rndMAP=random(0,25);
    unsigned char rndAmbientAirTemp=random(0,200);
    unsigned char rndCAT1Temp=random(1,55);
    unsigned char rndAFR=random(110,155);
    unsigned char rndFuel=random(0,255);
    unsigned char rndAdv=random(110,155);
    unsigned char rndLoad=random(0,255);
    unsigned char rndTPS=random(0,200);
    unsigned char rndLambda=random(120,130);

     //MAZDA MX5 CAN SIM
    unsigned char rndMazdaRPM=random(0,142); //varia de 0-256 para 0-16300 rpm
    unsigned char rndMazdaTPS=random(0,142);
    unsigned char rndMazdaSpeed=random(40,110);
    unsigned char rndMazdaBreakP=random(0,100);
    unsigned char rndMazdaBreakSw=200*random(0,1);
  
//SENSORS MODE 0x01 DATA
    unsigned char EngineLoad[8] =         {4, 65, 4, 255, 0, 0, 0,0};
    unsigned char CoolantTemp[8] =        {4, 65, 5, rndCoolantTemp, 0, 185, 147,0};
    unsigned char MAP [8] =               {4, 65, 11, rndMAP, 0, 0, 0, 0};
    unsigned char RPM [8] =               {4, 65, 12, rndRPM, 0, 0, 0, 0};
    unsigned char vSpeed[8] =             {4, 65, 13, rndSpeed, 224, 185, 147, 0};
    unsigned char IATSensor[8] =          {4, 65, 15, IAT, 0, 185, 147, 0};
    unsigned char MAFSensor[8] =          {4, 65, 16, MAF, 0, 185, 147, 0};
    unsigned char LambdaSensor[8] =       {4, 65, 36, rndLambda, 0, 0, 0, 0};
    unsigned char FuelLevel[8] =          {4, 65, 47, rndFuel, 224, 185, 147,0 };
    unsigned char AFR[8] =                {4, 65, 52, rndAFR, 224, 185, 147, 0};
    unsigned char CAT1Temp[8] =           {4, 65, 60, rndCAT1Temp, 224, 185, 147, 0};
    unsigned char CAT2Temp[8] =           {4, 65, 61, rndCAT1Temp, 224, 185, 147, 0};
    unsigned char CAT3Temp[8] =           {4, 65, 62, rndCAT1Temp, 224, 185, 147, 0};
    unsigned char CAT4Temp[8] =           {4, 65, 63, rndCAT1Temp, 224, 185, 147, 0};
    unsigned char BatteryVoltage[8] =     {4, 65, 66, 48, 212, 0, 0, 0};
    unsigned char AmbientAirTemp[8] =     {4, 65, 70, rndAmbientAirTemp, 0, 185, 147, 0};
    unsigned char TPS[8] =                {4, 65, 17, rndTPS, 0, 185, 147, 0};

    
    //MODE 0x22 DATA FOR MAZDA MX5
    unsigned char MazdaCAT1Temp[8] =      {4, 98, 0, 60, rndCAT1Temp, 1, 1, 1 };
    unsigned char MazdaADV[8] =           {3, 98, 0, 14, rndAdv, 0, 0, 0 };
    unsigned char MazdaLoad[8] =          {3, 98, 0, 67, 0, rndLoad, 0, 0 };

    //Mazda ECUS Simulation
    unsigned char Mazda201[8] =           {rndMazdaRPM, 1, 200 ,200, rndMazdaSpeed, 0, rndMazdaTPS, 0};
    unsigned char Mazdawheelspeed[8] =    {40, 0,45 ,0, 50, 0, 55, 0};
    unsigned char MazdaBreak[8] =         {rndMazdaBreakP, 0, rndMazdaBreakSw, 0, 0, 0, 0, 0};
    unsigned char MazdaCoolant[8] =       {rndCoolantTemp, 0, 0, 0, 0, 0, 0, 0};
    unsigned char MazdaIAT[8] =           {0, 0, 0, 0, IAT, 0, 0, 0};
    */