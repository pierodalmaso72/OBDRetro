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

// CanBoard Inputs
float Vref = 5.0;        //Arduino Voltage level
int IATResistor = 10000; //Board TempSensor Resistor

int MAFPpin = A0; //Azul
int MAP2pin = A1; //Laranja
int IATpin = A2;  //Verde
int TPSpin = A3;  //Vermelho
int AFRpin = A4;  //Branco
int RPMpin = A5;  //Preto

// Outputs
int rx = 0;        // RX tentar evitar o uso
int tx = 1;        // TX tentar evita o uso
int MAF_MAPon = 3; // Usar MAP/MF se ON e Liberta A1 para entrada analog livre se OFF
int MAForMAP = 4;  //MAF ou MAP : ON = MAF e Liberta A1 para entrada analog livre
int MAP2on = 5;    // Usar MAP2 se ON
int IATon = 6;     // Usar IAT se ON
int TPSon = 7;     // Usar TPS se ON
int AFRon = 8;     // Usar AFR se ON
int RPMon = 10;    // Usar RPM se ON

//GLOBALS
bool noMAF=true;
bool noMAP = true;
bool noMAP2 = true;
bool noIAT = true;
bool noTPS = true;
bool noAFR = true;
bool noRPM = true;
bool noLoad=true;
bool noCoolant=true;
bool novSpeed=true;
bool noFuelLevel=true;
bool noCatTemp=true;
bool noBatteryV=true;
bool noAmbientTemp=true;
bool noAdvance=true;
bool noBreakPressure=true;
bool noMazda201=true;


int slowXfastPolling = 10;
int slowPolling = 0;
//char MAF;
//char MAP2;
//char IAT;
//char RPM;
//char TPS;
//char AFR;

/*FUNCTIONS BELLOW 
============================================================*/
void startCAN()
{
// keywords : CAN_250KBPS CAN_500KBPS CAN_1000KBPS
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

void CanScan(int startHex, int EndHex, byte *msg, int dlay)
{
  for (int id = startHex; id < EndHex; id++)
  {
    CAN.sendMsgBuf(id, 0, 8, msg);
    Serial.print("Can ID :  ");
    Serial.println(id);
    delay(dlay);
  }
}

//Mazdadmode: if False simple OBD Mode0x01 if true sim Mazda mx5 >2005
unsigned char* getMAF(int cycles)
{
  int MAFadc = analogRead(MAFPpin);
  //float MAFv = MAFadc / 1023.0 * Vref;
  //int MAF256 = MAFadc * 65535 / 1023 / 256;
  //char MAF = MAF256;
  //float MAFreal = MAF256 * 256.0 / 100.0;
  //analogWrite(3, MAFadc / 4); //gera pwm com duty proporcional ao potenciometro da MAF
  /*
  Serial.print("MAF Analog reading ");  Serial.println(MAFadc);
  Serial.print("MAF Voltage reading "); Serial.println(MAFv);
  */
}
unsigned char* getMAP(bool usemap2, int cycles){};
unsigned char* getEngineLoad(bool mazdamode,int cycles)
{
  unsigned char load=0;
  if (mazdamode==false) {static unsigned char EngineLoad[8] = {4, 65, 4, load, 0, 0, 0, 0}; return EngineLoad;}
  else {static unsigned char EngineLoad[8] = {3, 98, 0, 67, 0, load, 0, 0 }; return EngineLoad;} //Mazda MX5 Mode0x22
}
unsigned char* getTPS(bool mazdamode,int cycles){};
unsigned char* getAFR(int cycles){};
unsigned char* getRPM(bool mazdamode, int rpmtype, int cycles){};
unsigned char* getIAT(bool mazdamode, int cycles) 
{
  int IATadc = analogRead(IATpin);
  //float IATv = IATadc / 1023.0 * Vref;
  //float IATr = (1023.0 / IATadc) - 1;
  //IATr = IATResistor / IATr;
  //float IATTemp = 1 / (log(IATr / IATResistor) / 3950 + 1 / 298.15) - 273.15;
  int IATcalc = -IATadc * (8.2 / 98) + 68.07551 + 40;
  unsigned char IAT = IATcalc;
  if (mazdamode==false) {static unsigned char IATSensor[8] = {4, 65, 15, IAT, 0, 185, 147, 0}; return IATSensor;}
  else {static unsigned char IATSensor[8] = {0, 0, 0, 0, IAT, 0, 0, 0}; return IATSensor;}
  /*
  Serial.print("IAT Analog reading ");  Serial.println(IATadc);
  Serial.print("IAT Voltage reading "); Serial.println(IATv);
  Serial.print("IAT Resistance reading "); Serial.println(IATr);
  Serial.print("IAT Temp reading "); Serial.println(IATTemp);
  Serial.print("IAT Calc "); Serial.println(IATcalc);
  */
}
unsigned char* getCoolantTemp(bool mazdamode, int cycles) 
{
  unsigned char coolanttemp=0;
  if (mazdamode==false) {static unsigned char CoolantTemp[8] = {4, 65, 5, coolanttemp, 0, 185, 147,0}; return CoolantTemp;}
  else {static unsigned char CoolantTemp[8] = {coolanttemp, 0, 0, 0, 0, 0, 0, 0}; return CoolantTemp;}
}
unsigned char* getvSpeed(int cycles)
{
  static unsigned char vSpeed[8] = {4, 65, 13, 0, 224, 185, 147, 0};
  return vSpeed;
} 
unsigned char* getFuelLevel(int cycles)
{
  static unsigned char FuelLevel[8] = {4, 65, 47, 0, 224, 185, 147,0 };
  return FuelLevel;
}
unsigned char* getCATTemp(bool mazdamode,int CAT, int cycles)
{ 
  if (mazdamode==false) 
  {
    if (CAT==1) {static unsigned char CATTemp[8] ={4, 65, 60, 0, 224, 185, 147, 0}; return CATTemp;}
    if (CAT==2) {static unsigned char CATTemp[8] ={4, 65, 61, 0, 224, 185, 147, 0}; return CATTemp;}
    if (CAT==3) {static unsigned char CATTemp[8] ={4, 65, 62, 0, 224, 185, 147, 0}; return CATTemp;}
    if (CAT==4) {static unsigned char CATTemp[8] ={4, 65, 63, 0, 224, 185, 147, 0}; return CATTemp;}
  }
  else {static unsigned char CATTemp[8] = {4, 98, 0, 60, 0, 1, 1, 1 }; return CATTemp;} //Mazda MX5 mode 0x22h
}
unsigned char* getBatteryV(int cycles)
{
  static unsigned char BatteryVoltage[8] = {4, 65, 66, 0, 212, 0, 0, 0}; 
  return BatteryVoltage;
}
unsigned char* getAmbientTemp(int cycles)
{
  static unsigned char AmbientAirTemp[8] = {4, 65, 70, 0, 0, 185, 147, 0};
  return AmbientAirTemp;
}
unsigned char* getAdvance(bool mazdamode,int cycles)
{
  unsigned char advance=0;
  if (mazdamode==false) {static unsigned char Advance[8] = {4, 65, 14, advance, 0, 185, 147,0}; return Advance;}
  else {static unsigned char Advance[8] = {advance, 0, 0, 0, 0, 0, 0, 0}; return Advance;}
}
unsigned char* getBreakPressure(bool mazdamode, int cycles)
{

}
void Answer()
{
  //unsigned long StartTime = millis();
  CAN.readMsgBuf(&len, buf);
  canId = CAN.getCanId();
  for (int i = 0; i < len; i++)
  {
    BuildMessage = BuildMessage + buf[i] + ",";
  }
  Serial.print("<");
  Serial.print(canId, HEX);
  Serial.print(",");
  Serial.println(BuildMessage);

  //PID answers
  if(BuildMessage=="2,1,4,0,0,0,0,0,"  && noLoad==false)        {CAN.sendMsgBuf(0x7E8, 0, 8, getEngineLoad(false,1)); Serial.println(">01 EngineLoad");}
  if(BuildMessage=="2,1,5,0,0,0,0,0,"  && noCoolant==false)     {CAN.sendMsgBuf(0x7E8, 0, 8, getCoolantTemp(false,1)); Serial.println(">01 CoolantTemp");}
  if(BuildMessage=="2,1,11,0,0,0,0,0," && noMAP==false)         {CAN.sendMsgBuf(0x7E8, 0, 8, getMAP(1,1)); Serial.println(">01 MAP");}
  if(BuildMessage=="2,1,12,0,0,0,0,0," && noRPM==false)         {CAN.sendMsgBuf(0x7E8, 0, 8, getRPM(false,1,1)); Serial.println(">01 RPM");/*CAN_DataFrequency("Mode1");*/}
  if(BuildMessage=="2,1,13,0,0,0,0,0," && novSpeed==false)      {CAN.sendMsgBuf(0x7E8, 0, 8, getvSpeed(1)); Serial.println(">01 Speed");}
  if(BuildMessage=="2,1,14,0,0,0,0,0," && noAdvance==false)     {CAN.sendMsgBuf(0x7E8, 0, 8, getAdvance(false,1)); Serial.println(">01 Advance");}
  if(BuildMessage=="2,1,15,0,0,0,0,0," && noIAT==false)         {CAN.sendMsgBuf(0x7E8, 0, 8, getIAT(false,1)); Serial.println(">01 IAT");}
  if(BuildMessage=="2,1,16,0,0,0,0,0," && noMAF==false)         {CAN.sendMsgBuf(0x7E8, 0, 8, getMAF(1)); Serial.println(">01 MAF");}
  if(BuildMessage=="2,1,17,0,0,0,0,0," && noTPS==false)         {CAN.sendMsgBuf(0x7E8, 0, 8, getTPS(false,1)); Serial.println(">01 TPS");}
  if(BuildMessage=="2,1,47,0,0,0,0,0," && noFuelLevel==false)   {CAN.sendMsgBuf(0x7E8, 0, 8, getFuelLevel(1)); Serial.println(">01 FuelLev");}
  if(BuildMessage=="2,1,52,0,0,0,0,0," && noAFR==false)         {CAN.sendMsgBuf(0x7E8, 0, 8, getAFR(1)); Serial.println(">01 AFR");}
  if(BuildMessage=="2,1,60,0,0,0,0,0," && noCatTemp==false)     {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,1,1)); Serial.println(">01 CAT1Temp");}
  if(BuildMessage=="2,1,61,0,0,0,0,0," && noCatTemp==false)     {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,2,1)); Serial.println(">01 CTA2Temp");}
  if(BuildMessage=="2,1,62,0,0,0,0,0," && noCatTemp==false)     {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,3,1)); Serial.println(">01 CAT3Temp");}
  if(BuildMessage=="2,1,63,0,0,0,0,0," && noCatTemp==false)     {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(false,4,1)); Serial.println(">01 CAT4Temp");}
  if(BuildMessage=="2,1,66,0,0,0,0,0," && noBatteryV==false)    {CAN.sendMsgBuf(0x7E8, 0, 8, getBatteryV(1)); Serial.println(">01 BatteryV");}
  if(BuildMessage=="2,1,70,0,0,0,0,0," && noAmbientTemp==false) {CAN.sendMsgBuf(0x7E8, 0, 8, getAmbientTemp(1)); Serial.println(">01 AirTemp");}
  if(BuildMessage=="2,1,36,0,0,0,0,0," && noAFR==false)         {CAN.sendMsgBuf(0x7E8, 0, 8, getAFR(1)); Serial.println(">01 Lambda");}

  //Mode0x22 answers
  if(BuildMessage=="3,34,0,60,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, getCATTemp(true,1,1)); Serial.println(">22h FuelLev");/*CAN_DataFrequency("Mode22");*/}
  if(BuildMessage=="3,34,0,14,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, getAdvance(true,1)); Serial.println(">22h IgnAdv");}
  if(BuildMessage=="3,34,0,67,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, getEngineLoad(true,1)); Serial.println(">22h Load");}


  //OBD2 MODE0x01 PID 0 DATA
  unsigned char SupportedPID00[8] =     {65, 0, 255, 255, 255, 255, 0, 0}; //Perfeito
  unsigned char SupportedPID20[8] =     {65, 32, 255, 255, 255, 255, 0, 0};    //Perfeito
  unsigned char SupportedPID40[8] =     {65, 64, 255, 255, 255, 255, 0, 0};
  unsigned char SupportedPID60[8] =     {65, 96, 255, 255, 255, 254, 0, 0};
  unsigned char SupportedPID80[8] =     {65, 128, 255, 255, 255, 254, 0, 0};
  unsigned char SupportedPID00v4[8] =   {4, 65, 0, 255, 255, 255, 254, 0};
  unsigned char MilCleared[7] =         {4, 65, 63, 34, 224, 185, 147};
  //Mode0x01 PID0x0
  if(BuildMessage=="2,1,0,0,0,0,0,0,")          {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00); delay(2); Serial.println(">01 PID0");
                                                 CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00v4); Serial.println(">01 PID0 Extra");}
  if(BuildMessage=="2,1,0,153,153,153,153,153,"){CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00); delay(2);  Serial.println(">01 PID0");
                                                 CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID00v4); Serial.println(">01 PID0 Extra");}
  if(BuildMessage=="2,1,32,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID20); Serial.println(">01 PID0 0-20h");}
  if(BuildMessage=="2,1,64,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID40); Serial.println(">01 PID0 20-40h");}
  if(BuildMessage=="2,1,96,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID60); Serial.println(">01 PID0 40-60h");}
  if(BuildMessage=="2,1,128,0,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, SupportedPID80); Serial.println(">01 PID0 60-80h");}
  if(BuildMessage=="2,1,1,0,0,0,0,0,")          {CAN.sendMsgBuf(0x7E8, 0, 7, MilCleared); Serial.println(">01 PID1 MIL");}


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

//Broadcast Rates MazdaSim
int sendHcounter = 0; // com BDR=10 e HxL=10 40-50HZ/7HZ com  BDR=0 e HxL=10:314HZ/35HZ
int sendLcounter = 0;
int SendHRate=100; // Main Parameter for MAZDA SIM broadcast 
int SendLxHRate=5; // number of times that low priority msgs are silenced SendRate=10,SendLxH=10:H45Hz,L7Hz); SendRate=0,SendHxL=10->H314Hz,L35Hz; SendRate=100,SendHXL=5->H5Hz,L1hz (com 3 variaveis pedidas em modo01 a 7hz)

void MazdaSIM()
{
  if(sendHcounter==SendHRate)
  {
    if(noMazda201==false) 
    {
      unsigned char* RPM=getRPM(true,3,1);
      unsigned char* TPS=getTPS(true,1);
      unsigned char rpm= RPM[1];
      unsigned char tps= TPS[1];
      unsigned char mazdaspeed=0;
      unsigned char Mazda201[8] = {rpm, 1, 200 ,200, mazdaspeed, 0, tps, 0};
      CAN.sendMsgBuf(0x201, 0, 8, Mazda201);  
      Serial.println("SentH: RPM,TPS, SPEED");
    }
    if (noBreakPressure=false)
    {  
      CAN.sendMsgBuf(0x85, 0, 8, getBreakPressure(true,1)); 
      Serial.println("SentH: Break data");
      sendLcounter++;
      sendHcounter=0;
    }
    //CAN.sendMsgBuf(0x4b0, 0, 8, Mazdawheelspeed); Serial.println(">H WheelSpeed");
    //CAN_DataFrequency("MazdaH");
    }
    else {sendHcounter++;}
    //Put here LOW Priority msgs. sent quitetimes lesss
    if(sendLcounter==SendLxHRate)

    {
      CAN.sendMsgBuf(0x421, 0, 8, getCoolantTemp(true,1)); Serial.println("SentL: CoolantTemp");
      CAN.sendMsgBuf(0x240, 0, 8, getIAT(true,1)); Serial.println("SentL: IAT");
      sendLcounter=0;
    //CAN_DataFrequency("MazdaL");
    }

}
 //   unsigned char MAP [8] =               {4, 65, 11, rndMAP, 0, 0, 0, 0};
 //   unsigned char RPM [8] =               {4, 65, 12, rndRPM, 0, 0, 0, 0};
 //   unsigned char MAFSensor[8] =          {4, 65, 16, MAF, 0, 185, 147, 0};
 //   unsigned char LambdaSensor[8] =       {4, 65, 36, rndLambda, 0, 0, 0, 0};
 //   unsigned char TPS[8] = {4, 65, 17, rndTPS, 0, 185, 147, 0};
 //   unsigned char AFR[8] =                {4, 65, 52, rndAFR, 224, 185, 147, 0};
    
    
    //MODE 0x22 DATA FOR MAZDA MX5
  //  unsigned char MazdaADV[8] =           {3, 98, 0, 14, rndAdv, 0, 0, 0 };

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  startCAN();
  pinMode(MAF_MAPon, INPUT_PULLUP);
  pinMode(MAForMAP, INPUT_PULLUP);
  pinMode(MAP2on, INPUT_PULLUP);
  pinMode(IATon, INPUT_PULLUP);
  pinMode(TPSon, INPUT_PULLUP);
  pinMode(AFRon, INPUT_PULLUP);
  pinMode(RPMon, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  noMAF=true;
  noMAP=true;
  noIAT=true;
  noTPS=true;
  noAFR=true;
  noRPM=true;
  noLoad=true;
  noCoolant=true;
  novSpeed=true;
  noFuelLevel=true;
  noCatTemp=true; 
  noBatteryV=true;
  noAmbientTemp=true;
  noAdvance=true;
  /*=============================================================
    loop: CHECK FOR MSGS AND SEND RESPONSES
  =============================================================*/
    if(CAN_MSGAVAIL == CAN.checkReceive())
    {
      Answer();
      BuildMessage="";
      Serial.println("RESPONSE FINISHED");
      //CAN_DataFrequency("Response");
      //delay(25);
    }
  // MAF MAP READING
  byte estadopino=digitalRead(MAF_MAPon);
  if (estadopino==HIGH)
  {
    estadopino=digitalRead(MAForMAP);
    if(estadopino==HIGH) {getMAF(1);noMAF=0;}
    else {estadopino=digitalRead(MAP2on);getMAP(estadopino,1);noMAP=0;}
  }
  // IAT SLOW POLLING
  
  if(slowPolling==slowXfastPolling)
  {
    slowPolling=0;
    if(digitalRead(IATon)) {noIAT=0;getIAT(false,1);}
    if(digitalRead(TPSon)) {noTPS=0;getTPS(false,1);}
    if(digitalRead(AFRon)) {noAFR=0;getAFR(1);}
  }
  else {slowPolling++;} 
  if(digitalRead(RPMon)) {noRPM=0;getRPM(false,1,1);}
  
  //MAZDA MX5 >2005 SPECIFIC FUNCTIONS
  //bool MazdaSim=true;
  bool MazdaSim=false;
  bool sCAN=false;
  //MAZDASIM: BROADCAST MAZDA FORMAT DATA TO CAN_BUS
  
  if(MazdaSim==true)
  {
    MazdaSIM();
  }
  
    /*
    =============================================================
    loop: SCAN IDs - RESPONDING IDs ON AIM
    =============================================================*/
    if(sCAN==true)
    {
      byte Mazdatest[8] = {200,0,0,0,0,0,0,0};
      CanScan( 0x0, 0xFF, Mazdatest, 5);
    }

}


  /*//VALORES ALEOTORIOS TESTE PARA OBD2
    char rndCoolantTemp=random(200,255);
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