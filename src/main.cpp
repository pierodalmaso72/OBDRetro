#include <Arduino.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

MCP_CAN CAN(9);                                      // Set CS to pin 9
unsigned long int canId = 0x000;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
String BuildMessage="";
int MSGIdentifier=0;



// CanBoard Inputs
float Vref=4.95;  //Arduino Voltage level
int   IATResistor=10000;  //Board TempSensor Resistor
int   MAFPpin=A0;
int   MAP2pin=A1;
int   IATpin = A2;
int   RPMpin=A3;
int   TPSpin=A4;
int   AFRpin=A5;
char  IAT;
char  MAF;

// Outputs
int   PWMOutpin=3;

//Broadcast Rates
int broadcastH=0; // com BDR=10 e HxL=10 40-50HZ/7HZ com  BDR=0 e HxL=10:314HZ/35HZ
int broadcastL=0;

//Calculo do Tempo de resposta millis()
unsigned long StartTime = 0;
unsigned long EndTime = 0;
unsigned long DurationMillis = 0;
float BroadcastFreq = 0;
String BDFreqMarker=""; //"MazdaH" "MazdaL" "Response" "Mode1" "Mode22"

//GLOBALS


/*=============================================================
=============================================================


FUNCTIONS BELLOW


=============================================================
============================================================*/
void startCAN()
{
    // keywords : CAN_250KBPS CAN_500KBPS CAN_1000KBPS
    START_INIT:
      if(CAN_OK == CAN.begin(CAN_500KBPS))
        {Serial.println("CAN BUS Shield init ok!");}
      else
      {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
      }
}
/*=============================================================*/
void getMessage ()
{
    //unsigned long StartTime = millis();
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
    for(int i = 0; i<len; i++)
      {
        BuildMessage = BuildMessage + buf[i]+ ",";
      }
    Serial.print("<"); Serial.print(canId,HEX);Serial.print(",");
    Serial.println(BuildMessage);
}
/*=============================================================*/
void getMAF()
{
    int MAFadc=analogRead(MAFPpin);
    float MAFv=MAFadc/1023.0*Vref;
    int MAF256=MAFadc*65535/1023/256;
    char MAF=MAF256;
    float MAFreal=MAF256*256.0/100.0;
    analogWrite(3,MAFadc/4); //gera pwm com duty proporcional ao potenciometro da MAF
    /*
    Serial.print("MAF Analog reading ");  Serial.println(MAFadc);
    Serial.print("MAF Voltage reading "); Serial.println(MAFv);
    */
}
/*=============================================================*/
void getIAT()
{
      int IATadc = analogRead(IATpin);
      float IATv=IATadc/1023.0*Vref;
      float IATr=(1023.0/IATadc)-1;
      IATr= IATResistor/IATr;
      float IATTemp= 1/(log(IATr/IATResistor)/3950+1/298.15)-273.15;
      int IATcalc=-IATadc*(8.2/98)+68.07551+40;
      char IAT=(IATcalc);
      /*
      Serial.print("IAT Analog reading ");  Serial.println(IATadc);
      Serial.print("IAT Voltage reading "); Serial.println(IATv);
      Serial.print("IAT Resistance reading "); Serial.println(IATr);
      Serial.print("IAT Temp reading "); Serial.println(IATTemp);
      Serial.print("IAT Calc "); Serial.println(IATcalc);
      */
}
/*=============================================================*/
void CanScan (int startHex, int EndHex, byte *msg, int dlay)
{
  for (int id=startHex; id<EndHex;id++)
  {
    CAN.sendMsgBuf(id, 0, 8, msg);
    Serial.print("Can ID :  ");  Serial.println(id);
    delay(dlay);
  }
}
/*=============================================================*/
void CAN_DataFrequency(String mark)
{
  if(mark==BDFreqMarker)
  {
    unsigned long t=millis();
    if(StartTime==0)
    {
      StartTime=t;
      EndTime=0;
    }
    else
    {
      EndTime=t;
      DurationMillis=EndTime-StartTime;
      BroadcastFreq=1000.0/DurationMillis;
      Serial.print(mark);Serial.print("- Frequency: ");
      Serial.print(BroadcastFreq);
      Serial.print("(");
      Serial.print(DurationMillis);Serial.println(" ms)");
      StartTime=0;
      EndTime=0;
     }
  }
}

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  pinMode(PWMOutpin, OUTPUT);  // configura  o pino como saída
  startCAN();

}

void loop() {
  // put your main code here, to run repeatedly:
/*
    =============================================================
    loop: GET CAR VALUES OR GENERATE RANDOM FOR TESTS
    =============================================================
    */
    getIAT();
    getMAF();
    //VALORES ALEOTORIOS TESTE PARA OBD2
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

    /*
    =============================================================
    loop: BUILD DATA FRAMES
    =============================================================*/
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

    //OBD2 MODE0x01 PID 0 DATA
    unsigned char SupportedPID00[8] =     {65, 0, 255, 255, 255, 255, 0, 0}; //Perfeito
    unsigned char SupportedPID20[8] =     {65, 32, 255, 255, 255, 255, 0, 0};    //Perfeito
    unsigned char SupportedPID40[8] =     {65, 64, 255, 255, 255, 255, 0, 0};
    unsigned char SupportedPID60[8] =     {65, 96, 255, 255, 255, 254, 0, 0};
    unsigned char SupportedPID80[8] =     {65, 128, 255, 255, 255, 254, 0, 0};
    unsigned char SupportedPID00v4[8] =   {4, 65, 0, 255, 255, 255, 254, 0};
    unsigned char MilCleared[7] =         {4, 65, 63, 34, 224, 185, 147};

    /*
    =============================================================
    loop: CHECK FOR MSGS AND SEND RESPONSES
    =============================================================
    */
    if(CAN_MSGAVAIL == CAN.checkReceive())
    {
      getMessage();
      //PID answers
      if(BuildMessage=="2,1,4,0,0,0,0,0,")          {CAN.sendMsgBuf(0x7E8, 0, 8, EngineLoad); Serial.println(">01 EngineLoad");}
      if(BuildMessage=="2,1,5,0,0,0,0,0,")          {CAN.sendMsgBuf(0x7E8, 0, 8, CoolantTemp); Serial.println(">01 CoolantTemp");}
      if(BuildMessage=="2,1,11,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, MAP); Serial.println(">01 MAP");}
      if(BuildMessage=="2,1,12,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, RPM); Serial.println(">01 RPM");/*CAN_DataFrequency("Mode1");*/}
      if(BuildMessage=="2,1,13,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, vSpeed); Serial.println(">01 Speed");}
      if(BuildMessage=="2,1,15,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, IATSensor); Serial.println(">01 IAT");}
      if(BuildMessage=="2,1,16,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, MAFSensor); Serial.println(">01 MAF");}
      if(BuildMessage=="2,1,17,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, TPS); Serial.println(">01 TPS");}
      if(BuildMessage=="2,1,47,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, FuelLevel); Serial.println(">01 FuelLev");}
      if(BuildMessage=="2,1,52,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, AFR); Serial.println(">01 AFR");}
      if(BuildMessage=="2,1,60,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, CAT1Temp); Serial.println(">01 CAT1Temp");}
      if(BuildMessage=="2,1,61,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, CAT2Temp); Serial.println(">01 CTA2Temp");}
      if(BuildMessage=="2,1,62,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, CAT3Temp); Serial.println(">01 CAT3Temp");}
      if(BuildMessage=="2,1,63,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, CAT4Temp); Serial.println(">01 CAT4Temp");}
      if(BuildMessage=="2,1,66,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, BatteryVoltage); Serial.println(">01 BatteryV");}
      if(BuildMessage=="2,1,70,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, AmbientAirTemp); Serial.println(">01 AirTemp");}
      if(BuildMessage=="2,1,36,0,0,0,0,0,")         {CAN.sendMsgBuf(0x7E8, 0, 8, LambdaSensor); Serial.println(">01 Lambda");}

      //Mode0x22 answers
      if(BuildMessage=="3,34,0,60,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, MazdaCAT1Temp); Serial.println(">22h FuelLev");/*CAN_DataFrequency("Mode22");*/}
      if(BuildMessage=="3,34,0,14,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, MazdaADV); Serial.println(">22h IgnAdv");}
      if(BuildMessage=="3,34,0,67,0,0,0,0,")        {CAN.sendMsgBuf(0x7E8, 0, 8, MazdaLoad); Serial.println(">22h Load");}
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

      BuildMessage="";
      Serial.println("RESPONSE FINISHED");
      //CAN_DataFrequency("Response");
      //delay(25);
    }
    /*
    =============================================================

    loop: MAZDA MX5 >2005 SPECIFIC FUNCTIONS

    =============================================================
    */
    //bool MazdaSim=true;
    bool MazdaSim=false;
    bool sCAN=false;
    /*
    =============================================================
    loop: MAZDASIM: BROADCAST MAZDA FORMAT DATA TO CAN_BUS
    =============================================================*/
    BDFreqMarker="MazdaH";
    int broadcastRate=100;
    int RatioLxH=5; // number of times that low priority msgs are silenced
    // aprox BDR=10,RLxH=10-45Hz,7Hz); BDR=0,HxL=10-314Hz,35Hz ; BDR=100,HXL=5-5Hz,1hz // com 3 variaveis pedidas em modo01 a 7hz

    if(MazdaSim==true)
    {
      //Put here high Priority msgs.Always sent
      if(broadcastH==broadcastRate)
      {
        CAN.sendMsgBuf(0x201, 0, 8, Mazda201);  Serial.println(">H RPM,TPS, SPEED");
        CAN.sendMsgBuf(0x85, 0, 8, MazdaBreak); Serial.println(">H Break");
        broadcastL++;
        broadcastH=0;
        //CAN.sendMsgBuf(0x4b0, 0, 8, Mazdawheelspeed); Serial.println(">H WheelSpeed");
        //CAN_DataFrequency("MazdaH");
      }
      else {broadcastH++;}
      //Put here LOW Priority msgs. sent quitetimes lesss
      if(broadcastL==RatioLxH)
      {
        CAN.sendMsgBuf(0x421, 0, 8, MazdaCoolant); Serial.println(">L Coolant");
        CAN.sendMsgBuf(0x240, 0, 8, MazdaIAT); Serial.println(">L IAT");
        broadcastL=0;
        //CAN_DataFrequency("MazdaL");
      }
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



/* MAZDA ID
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