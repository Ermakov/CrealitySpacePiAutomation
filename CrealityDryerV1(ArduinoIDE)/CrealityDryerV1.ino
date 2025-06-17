#include <Wire.h>
#define keyONin     17  // Button 1 in ON/OFF   // 0 - ADDRESS
#define keyDOWNin   16  // Button 2 in UP       // 1 - all time 0x00
#define keyUPin     15  // Button 3 in DOWN     // 2 - cursor
#define keySETin    14  // Button 4 in SET      // 3,4 - SV (bit4 in byte2 if 1 -> 1xx) 
                                                // 5,6 - PV_C (bit4 in byte2 if 1 -> 1xx)
#define keyONout    6   // Button 1 out         // 7 - F/C E5 - C; EA - F
#define keyDOWNout  7   // Button 2 out         // 8-13 - material
#define keyUPout    8   // Button 3 out         // 14,15 - PV_RH
#define keySETout   9   // Button 4 out         // 16-17 - time HH:MM:SS

// Soft I2C sniffer
#define i2cSoftADDR   0x7E  // not shift
#define SoftSCL       2 
#define SoftSDA       3
#define i2cDelay      200   // uS
#define i2cREADY      B00000000     // check time out before start receiving
#define i2cRECEIVING  B00000001     // Receiving data
#define i2cSTOP       B00000010     // Stop all data received
#define i2cSTART      B00000100     // Start receiving
#define i2cBUSY       B00001000     // BUSY (calculation)
#define i2cSTOPcalc   B00010000 
#define BufSizeSoft   22    // byte 0 - ADDRESS
#define i2cTimeAnswer 500   // mS
volatile byte BufferSoft[BufSizeSoft];
volatile byte byteNUM, bitNUM,byteTMP;
volatile byte i2cStatus = i2cSTOP;
volatile unsigned long i2cTimeSoft = 0;
bool RefreshBuffer = true;

// I2C Commands
#define i2cHTU21Dh       0xE5
#define i2cHTU21Dc       0xE3
#define i2cSetTime       0x10 // second byte TIME
#define i2cGetH          0x20 // get time Hours
#define i2cGetM          0x21 // Minutes
#define i2cGetS          0x22 // Sec
#define i2cGetPT         0x23 // temperature
#define i2cGetPH         0x24 // humidity
#define i2cGetSV         0x25 // target temperature
#define i2cGetMA         0x26 // Material
#define i2cSTATUS        0x27 // ON OFF ERROR (0-off; 1-on; 2-error/off; 3-error/on)
                              //             B00000000 B00000001 B00000010 B00000011

// Hard I2C
#define i2cHardADDR   0x40
#define BufSizeHard   33
volatile byte BufferHard[BufSizeHard];
volatile byte CMD = 0;
volatile byte CMDdata = 0;
volatile byte CMDstep = 0;
byte CMDmsb;
byte CMDlsb;
byte CMDnob; // number of out byte

// Button options
byte buttonPress = 0;
int KeyDelay = 0;
#define OtherKeyDelay   500
#define OnKeyDelay      1000   //millisecond
#define CountRepeat     3
#define FirstDelay      1000
byte FirstBoot = 4;


// Data
byte ctH = 0;
byte ctM = 0;
byte ctS = 0;       //current time H:M:S
byte cMaterial = 0; //current Material
byte cPV_C = 0;
byte cPV_RH = 0;
byte cSV = 0;       //current C,RH,SV
byte cur = 0;
byte cStat = 0;

// table for convert Digits
#define DigCount    11
const byte Digits[DigCount] = {0xAF,0xA0,0xCB,0xE9,0xE4,0x6D,0x6F,0xA8,0xEF,0xED,0x4F}; // decode digits 0,1,2,3,4,5,6,7,8,9,E E- error 
#define ERRHeat     225
#define ERRConv     255

// table for convert Cursor
const String CurName[5] = {"OFF","TIME","MATERIAL","SV","PV"};
#define tOFF       B00000000  // 0 - OFF
#define tTIME      B00000010  // 1 - TIME
#define tMATERIAL  B00000100  // 2 - MATERIAL
#define tSV        B00001000  // 3 - SV
#define tPV        B10000000  // 4 - PV

// table for convert Material
#define MatCount 12
const byte MaterialXOR[MatCount] =    {0x27,  0x37, 0xEB, 0xFD,0xE3, 0x19, 0x83,    0xFE,     0xF3,   0x93, 0x97,0xE1};
const String MaterialName[MatCount] = {"ABS","ASA","PETG","PC","PA","PET","PLA-CF","PETG-CF","PA-CF","PLA","TPU","PP"}; //decode HashXOR Material

//#define DEBUG

void setup() {
  // put your setup code here, to run once:
  delay(FirstDelay);
  Serial.begin(115200);
  pinMode(keyONout, OUTPUT);
  digitalWrite(keyONout,LOW);
  pinMode(keyUPout, OUTPUT);
  digitalWrite(keyUPout,LOW);
  pinMode(keyDOWNout, OUTPUT);
  digitalWrite(keyDOWNout,LOW);
  pinMode(keySETout, OUTPUT);
  digitalWrite(keySETout,LOW);
  pinMode(keyONin, INPUT);
  pinMode(keyUPin, INPUT);
  pinMode(keyDOWNin, INPUT);
  pinMode(keySETin, INPUT);
  Wire.begin(i2cHardADDR);    // Initialize I2C (Slave Mode)
}

void irqSCL() {
  if (i2cStatus == i2cREADY) {
    i2cStatus = i2cRECEIVING;
    byteNUM = 0;
    bitNUM = 0;
    byteTMP = 0x00;
  }
  if (i2cStatus == i2cRECEIVING) {
    if (bitNUM < 8) {
      byteTMP = byteTMP << 1;
      byteTMP |= digitalRead(SoftSDA);
      bitNUM ++;
    }
    else if (byteNUM <= BufSizeSoft) {
      BufferSoft[byteNUM] = byteTMP;
      byteNUM ++;
      byteTMP = 0x00;
      bitNUM = 0;
    }
    if (byteNUM == BufSizeSoft) {
      i2cStatus = i2cSTOP; // bytes recieved and stop
    }
  }
  i2cTimeSoft = micros();
}

void I2CSofDetectEndOfData() {
  detachInterrupt(digitalPinToInterrupt(SoftSCL));                  //noInterrupts();
  if (((micros() - i2cTimeSoft) > i2cDelay) && (i2cStatus == i2cSTART)) {
    i2cStatus = i2cREADY ; // end of reception i2cSoft
  }
  attachInterrupt(digitalPinToInterrupt(SoftSCL), irqSCL, RISING);  //interrupts();
}

void I2C_TxHandler(void) {
  if (CMDnob > 0) {
    Wire.write(CMDmsb);
  }
  if (CMDnob == 2) {
    Wire.write(CMDlsb);
  }
}

void I2C_RxHandler(int numBytes) {
  for (byte i = 0; i < numBytes; i = i + 1) {
      BufferHard[i] = Wire.read();
  }
  RefreshBuffer = true;
  if (CMD == 0) {
    CMDstep = 1;
    CMD = BufferHard[0];
    BufferHard[0] = 0;
    if (numBytes == 2) {
      CMDdata = BufferHard[1];
      BufferHard[1] = 0;
    }
    else {
      CMDdata = 0;
    }
  }
}

void ScanPressKey() {
  byte b1st = digitalRead(keyONin);
  if (b1st != digitalRead(keyONout)) {digitalWrite(keyONout, b1st);}
  byte b2st = digitalRead(keyUPin);
  if (b2st != digitalRead(keyUPout)) {digitalWrite(keyUPout, b2st);}
  byte b3st = digitalRead(keyDOWNin);
  if (b3st != digitalRead(keyDOWNout)) {digitalWrite(keyDOWNout, b3st);}
  byte b4st = digitalRead(keySETin);
  if (b4st != digitalRead(keySETout)) {digitalWrite(keySETout, b4st);}
}

byte SoftPressKey(byte nKey, int kD) {
  static unsigned long keyTime;
  static bool kP;
  if (nKey == 0) {
    kP = false;
  }
  else if (!kP) {
    kP = true;
    keyTime = millis();
    digitalWrite(nKey,HIGH);
  }
  else if ((millis() - keyTime) > kD) {
    digitalWrite(nKey,LOW);
    kP = false;
  }
  return kP;
}

byte DigToReadable(byte Hi, byte Low, boolean F=false) {
  byte dh = 255;
  byte dl = 255;
  for (byte i = 0; i < DigCount; i++) {
    if ((Hi & B11101111) == Digits[i]) { dh = i * 10; }
    if ((Low & B11101111) == Digits[i]){ dl = i;}
  }
  if ((dh == 255) || (dl == 255)) { return ERRConv; } // error covert
  if (dh == 100) { return ERRHeat; }                  // error on screen (Ex)
  if ((F == true) && ((Hi&B00010000) > 0)) { dh = dh + 100; }
  return dh + dl;
}

byte HexXORToMat(byte h) {
  for (byte i = 0; i < MatCount; i = i + 1) {
    if (h == MaterialXOR[i]) {
      return i;
    }
  }
  return 0xFF;
}

bool ReadBufferSoft() {
    byte MatXOR = 0x00;
    if (i2cStatus == i2cSTOP) {
      i2cStatus = i2cBUSY;
      if (BufferSoft[0] == i2cSoftADDR) {
        #ifdef DEBUG
          for (byte i = 0; i < BufSizeSoft; i += 1){Serial.print(BufferSoft[i],HEX); Serial.print(" "); }
          Serial.print("\r\n");
        #endif
        for (byte i = 8; i < 14; i = i + 1) { MatXOR = MatXOR ^ BufferSoft[i]; } // XOR Hash 
        cMaterial = HexXORToMat(MatXOR);
        cur = BufferSoft[2] & B10001110;
        cSV = DigToReadable(BufferSoft[3],BufferSoft[4],true);
        cPV_C = DigToReadable(BufferSoft[5],BufferSoft[6],true);
        cPV_RH = DigToReadable(BufferSoft[14],BufferSoft[15]);
        ctH = DigToReadable(BufferSoft[16],BufferSoft[17]);
        ctM = DigToReadable(BufferSoft[18],BufferSoft[19]);
        ctS = DigToReadable(BufferSoft[20],BufferSoft[21]);
        if ((ctH > 0) || (ctM > 0) || (ctS > 0)) {
          cStat |= B00000001;
        }
        else {
          cStat &= B00000010;
        }
        if (cPV_C == ERRHeat){
          cStat |= B00000010;
        }
      }
      i2cStatus = i2cSTOPcalc;
      return true;
    }
    else {
      return false;
    }
}

void DebugPB() {  // Debug serial print buffer
  static unsigned long TimeLoop;
  unsigned long mStmp = millis();
  if ((mStmp - TimeLoop) > 800) {       // Every 800mS
    if (i2cStatus == i2cSTOP) {
      for (byte i = 0; i < BufSizeSoft; i += 1) {Serial.print(BufferSoft[i],HEX); Serial.print(" "); }
        Serial.print("-");
        //Serial.print(i2cCOMMAND,HEX);
        //Serial.print(" ");
        //Serial.print(i2cDATA,HEX);
       // Serial.print("\r\n");
        ctH = DigToReadable(BufferSoft[16],BufferSoft[17]);
        ctS = DigToReadable(BufferSoft[20],BufferSoft[21]);
        cPV_RH = DigToReadable(BufferSoft[14],BufferSoft[15]);
        cPV_C = DigToReadable(BufferSoft[5],BufferSoft[6],true);
        //i2cStatus = i2cSTART;
        //i2cStatus = i2cSTOPcalc;
    }
    TimeLoop = mStmp;
  }
}

void Start() {
  static unsigned long sTime;
  static byte tmpCount;
  if (FirstBoot == 4) {
    i2cStatus = i2cSTART;
    FirstBoot--;
    sTime = millis();
    tmpCount = 0;
  }
  if ((millis()-sTime) > i2cTimeAnswer) {
    if (FirstBoot == 3) {
      BufferSoft[0] = 0;
      i2cStatus = i2cSTART;
      sTime = millis();
      FirstBoot--;
    }
    else if (FirstBoot == 2) {
      if ((i2cStatus != i2cSTOP) || (BufferSoft[0] != i2cSoftADDR)) {
        buttonPress = keyONout;
        KeyDelay = OnKeyDelay;
        FirstBoot--; 
      }
      else {
        FirstBoot = 0;
      }
      sTime = millis();
    }
    else if (FirstBoot == 1){
      if (buttonPress == 0){
        if (tmpCount < CountRepeat) {
          FirstBoot = 3;
        }
        else {
          FirstBoot = 255;
        }
        tmpCount++;
        sTime = millis();
      }
    }
  }
  if ((FirstBoot == 0) || (FirstBoot == 255)){
    Wire.onReceive(I2C_RxHandler);
    Wire.onRequest(I2C_TxHandler);
  }
}

void SetTime() {
  static unsigned long CmdTime;
  if ((millis()-CmdTime) > 50) {
    if (CMDstep == 1) {
      CMDstep++;
      if (CMDdata > 48) {
        CMDdata = 48;
      }
    }
    else if (CMDstep == 2) {
      buttonPress = keySETout;
      KeyDelay = OtherKeyDelay;
      CMDstep++;
    }
    else if (CMDstep == 3) {
      if (buttonPress == 0) {
        if (cur == tTIME){
          CMDstep++;
        }
        else {
          CMDstep--;
        }
      }
    }
    else if (CMDstep == 4) {
      if (CMDdata > ctH) {
        if (abs(CMDdata - ctH) < 24) {
          buttonPress = keyUPout;
          KeyDelay = OtherKeyDelay;
        }
        else {
          buttonPress = keyDOWNout;
          KeyDelay = OtherKeyDelay;
        }
      }
      else {
        if (abs(CMDdata - ctH) < 24) {
          buttonPress = keyDOWNout;
          KeyDelay = OtherKeyDelay;
        }
        else {
          buttonPress = keyUPout;
          KeyDelay = OtherKeyDelay;
        }  
      }
      CMDstep++;
    }
    else if (CMDstep == 5) {
      if (buttonPress == 0) {
        if (ctH == CMDdata) {
          CMDstep++;
        }
        else {
          CMDstep--;
        }
      }
    }
    CmdTime = millis();
    RefreshBuffer = true;
  }
  if (CMDstep == 6) {
    CMDstep = 0;
    CMD = 0;
  }
}


void loop(){
  unsigned long tmp;
  static unsigned long TimeCheck;
  I2CSofDetectEndOfData();
  if ((FirstBoot > 0) && (FirstBoot < 5)) {
    Start();
  }
  else {
    ReadBufferSoft();
  }
  if (!SoftPressKey(buttonPress,KeyDelay)) {
    ScanPressKey();               // Scan key on screen and "press"
    buttonPress = 0;
    KeyDelay = 0;
  }
  if (RefreshBuffer == true) {
    if (i2cStatus == i2cSTOPcalc) {
      i2cStatus = i2cSTART;
      RefreshBuffer = false;
    }
  }

  if (CMD == i2cSetTime) {
    SetTime();
  }
  else if (CMD == i2cGetH) {
    CMDmsb = ctH;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD == i2cGetM) {
    CMDmsb = ctM;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD == i2cGetS) {
    CMDmsb = ctS;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD == i2cGetPT) {
    CMDmsb = cPV_C;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD == i2cGetPH) {
    CMDmsb = cPV_RH;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD == i2cGetSV) {
    CMDmsb = cSV;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD == i2cGetMA) {
    CMDmsb = cMaterial;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD == i2cSTATUS) {
    CMDmsb = cStat;
    CMDnob = 1;
    CMD = 0; 
  }
  else if (CMD  == i2cHTU21Dh) {        //0xE5 Trigger Humidity Measurement 
    tmp = (cPV_RH + 6)*524;           // RH = -6 + 125*Srh/65536
    CMDmsb = (tmp>>8)&0xFF;
    CMDlsb = tmp&0xFF;
    CMDnob = 2;
    CMD = 0;
  }
  else if (CMD  == i2cHTU21Dc) {        // 0xE3 Trigger Temperature Measurement 
    tmp = (cPV_C + 47)*373;           // T = -46,85 + 175,72*St/65536
    CMDmsb = (tmp>>8)&0xFF;
    CMDlsb = tmp&0xFF;
    CMDnob = 2;
    CMD = 0;
  }
}



