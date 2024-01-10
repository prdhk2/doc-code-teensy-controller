/**************************\\******************************
  Biodigester RMC
  Description :
    Record :
      Temperature, pH, Humidity, Weight(Kg);
      Flow Gas Main, Flow Gas Outgoing, Storage Tank Gas;
    Communication Line:
      cable for Sensor, Wifi for Multiple Local Processor;
      SMS/GPRS/GSM for Gateway Main Processor to PC monitor;

 ***************************************************************/
#include <Wire.h>
#include <EEPROM.h>
#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <DFRobot_DHT11.h>  // tempoeratur

// Declaration of LCD_20x4
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

//Declaration of RTC
//time_t RTCTime;

//Declaration of DH DH  Temperature Ambien (TA) & Temperatur Liquid (TL)
DFRobot_DHT11 DHT;
#define DHT11_PINA 2  //Signal TA
// Declaration of Temperatur Liquid (TL)
int thermoCLK = 38;
int thermoCS = 37;
int thermoDO = 36;
//-------------------------------------------
int led = 13;  // on board 4.1
int beep = 28;
//deklarasi mode protein--------------------------------------------
int SwMakMin = 29;       // active low  Makanan & Minuman
int SwLowProtein = 30;   // active low  Sayur-Sayuran
int SwHighProtein = 31;  // active low  Daging & Ikan
int SwExtra = 32;        // active low  Daging & Ikan
int valLowProtein = 0;
int valHighProtein = 0;
char ModeBahan[15];


//deklarasi timer_Motor
int val_hMtr;       // hour
int addr_val_hMtr;  // alamat EEPROM hMtr
int val_mMtr;       // menute
int addr_val_mMtr;  // alamat EEPROM mMtr
int val_tMtr;       // second
int addr_val_tMtr;  // alamat EEPROM tMtr


//deklarasi SMS ----------------------------------------------------------
int btSendSMS = 33;  // active low
boolean val_btSendSMS;

//deklarasi Flow FD : =============================================
//const int turbinFDPin = 6;  // the pin that the push turbin is attached to
//int val_flowFD;
int val_FD;
// Variables will change: FD
int turbinFDState = 0;        // current state of the turbin FD
int turbinFDPushCounter = 0;  // counter for the number of turbin FD presses
int lastturbinFDState = 0;    // previous state of the turbin
float mtneFD_pulse;           // factor methane FD per pulse
float val_mtneFD_inpulse;     // methaneFD in total pulse
//unsigned int addrFD = 0;      // lokasi FD di EEPROM.write


//deklasasi Flow FU : ============================
//const int turbinFUPin = 9;  // the pin that the push turbin is attached to
int val_flowFU;
int val_FU;
// Variables will change: FU
int turbinFUState = 0;        // current state of the turbin FD
int turbinFUPushCounter = 0;  // counter for the number of turbinD FU presses
int lastturbinFUState = 0;    // previous state of the turbin FU
float mtneFU_pulse;           // factor methane FU per pulse
float val_mtneFU_inpulse;     // methane FU in total pulse
//unsigned int addrFU = 10;      // lokasi FD di EEPROM.write


//deklarasi Flow Tanki Storage : =============================
float val_TS;
double sum;
int count = 1;


//deklarasi temperatur ambient & liquid -----------------------------------------------------------

float DHT_temp_A;  // variable to store the value coming from the DHT.sensor calibration.//DHT_Async dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
//float DHT_temp_L;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
uint8_t degree[8] = { 140, 146, 146, 140, 128, 128, 128, 128 };

//deklarasi pH----------------------------PH
float calibration = 5.0;  //change this value to calibrate
float b;
int buf[10], temp;
//'''''
float voltage;
float pHValue;
float phNew;
//deklarasi pin analog
int adcPH;
//.................
int pHSense = A9;
int samples = 10;
float adc_resolution = 1024.0;

//deklarasi weight -------------------------------------

float a = 0.0;

//deklarasi  ----------------------------------

float valTemp_A;
float valTemp_L;

float val_inByte2;  // buffer in Serial2 data Flow FD
float val_inByte5;  // buffer in Serial5 data Weight


float val_Weight;    // buffer out Weight
float val_Pressure;  // buffer out pressure

//deklarasi buffer panel output ------------------------------
float bfvalTemp_A = valTemp_A;
//float bfval_Metan = val_Metan;
float bfvalTemp_L = valTemp_L;
float bfval_pH = phNew;
float bfval_FD = val_FD;
float bfval_FU = val_FU;
float bfval_Tank = (val_FD - val_FU);
float bfval_Weight = val_Weight;
float bfval_Pressure = val_Pressure;

int cloop_mc;
int cloop_mt;
int cloop_h;
int cloop_exeSMSgateway;

//END of Declaration  Buffer ------------------------------------

//Sub Routine ######################################################

//lcd_splash()  #####################################################
void lcd_splash() {
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  BIODIGESTER  RMC  ");
  lcd.setCursor(0, 1);
  lcd.print(" PT. RIZQI SEMESTA  ");
  lcd.setCursor(0, 2);
  lcd.print("Bougenville-Fontana ");
  lcd.setCursor(0, 3);
  lcd.print(" Kemayoran Jakarta  ");
  delay(2000);
  lcd.backlight();
  lcd.clear();
}
// end of lcd_splash()##########################################
//pc_splash() ##################################################
void pc_splash() {

  Serial1.println("   BIODIGESTER  Renik Methane Capture    ");
  Serial1.println("=========================================");
  Serial1.println("         PT. RIZQI SEMESTA               ");
  Serial1.println("Bougenville-Fontana Tower BF-39K1 Jakarta");
  delay(2500);
}
// end of pc_splash()############################################


//ph_Logging()###################################################
float ph(float voltage) {
  return 7 + ((2.5 - voltage) / 0.144);
}
//newone more -----------------------------------------------------
void pH_Logging() {
  int measurings = 0;
  for (int i = 0; i < samples; i++) {
    measurings += analogRead(pHSense);
    delay(50);
  }
  float voltage = (5 / adc_resolution) * measurings / samples;  //
  //phNew = (voltage);
  phNew = -ph(voltage) * (14 / 5);
  delay(500);
}
// end on pH_Logging()##########################################

// zerowaste
// Start of FD & FU methane Logging()%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//FD_methane_Logging()######################################################
void FD_methane_Logging() {  // Serial 1 T4.0 to Serial4 T4.1

  if (Serial4.available()) {    // T4.0 Tx  to  Rx T4.1 ;  T4.0 Rx to Tx T4.1
    // read the incoming byte:
    turbinFUPushCounter = Serial4.read();        // baca Serial4
  }


  // program dipindahkan ke sub processor T40  stand-alone
  /*
  // read the pushturbinFD input pin:
  turbinFDState = digitalRead(turbinFDPin);
  // compare the turbinFDState to its previous state
  if (turbinFDState != lastturbinFDState) {
    //   if the state has changed, increment the counter
    if (turbinFDState == HIGH) {
      // if the current state is HIGH then the turbinFD went from off to on:
      turbinFDPushCounter++;
      EEPROM.write(addrFD, turbinFDPushCounter); delay(200);     // save data counter pd lokasi FEEPROM.addr = 10
      //Serial.println("start");
      Serial.printf("number of turbinFD pushes: ", turbinFDPushCounter); Serial.println("  Pulse_sensor");
      val_mtneFD_inpulse = turbinFDPushCounter * mtneFD_pulse;
      val_FD = val_mtneFD_inpulse;     // load data update for sms
      Serial.printf("value methaneFD gas: %.3f", val_mtneFD_inpulse); Serial.println("  m3");
      //Serial2.print(val_mtneFD_inpulse);
    } else {
      // if the current state is LOW then the turbin went from on to off:
      //Serial.println("stop");
      Serial.println();
      delay(20);
    }
    // Delay a little bit to avoid bouncing
    delay(20);
  }
  // save the current state as the last state, for next time through the loop
  lastturbinFDState = turbinFDState;
  */
  // Batas yang dipindahkan ke sub processor T40 

}
// End of FD_methane_Logging()############################################

//FU_methane_Logging()####################################################
void FU_methane_Logging() {  // Serial 5 T4.0 to Serial6 T4.1
  
   if (Serial6.available()) {    // T4.0 Tx  to  Rx T4.1 ;  T4.0 Rx to Tx T4.1
    // read the incoming byte:
    turbinFUPushCounter = Serial6.read();        // baca Serial6
    Serial.println(turbinFUPushCounter);
    }
  
  
  // program dipindahkan ke sub processor T40  stand-alone
  /*
  
  // read the pushturbinFU input pin:
  turbinFUState = digitalRead(turbinFUPin);
  // compare the turbinState to its previous state
  if (turbinFUState != lastturbinFUState) {
    // if the state has changed, increment the counter
    if (turbinFUState == HIGH) {
      // if the current state is HIGH then the turbinFU went from off to on:
      turbinFUPushCounter++;
      EEPROM.write(addrFU, turbinFUPushCounter); delay(200);    // save data counter pd lokasi FEEPROM.addr = 10
      //Serial.println("start");
      Serial.printf("number of turbinFU pushes: ", turbinFUPushCounter); Serial.println("  Pulse_sensor");
      //Serial.print("number of methane FU gas: ");
      val_mtneFU_inpulse = turbinFUPushCounter * mtneFU_pulse;
      val_FU = val_mtneFU_inpulse;  // load data for SMS
      Serial.printf("value methane FU gas: %.3f", val_mtneFU_inpulse); Serial.println("  m3");
      //Serial2.print(val_mtneFU_inpulse);     
    } else {
      // if the current state is LOW then the turbinFU went from on to off:
      //Serial.println("stop");
      Serial.println();
      delay(20);
    }
    // Delay a little bit to avoid bouncing
    delay(20);
  }
  // save the current state as the last state, for next time through the loop
  lastturbinFUState = turbinFUState;

    */
  // Batas yang dipindahkan ke sub processor T40 
}
// End of FU_methane_Logging()############################################
// End of FD & FU methane Logging()%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//Weight_Logging() ########################################################
void Weight_Logging() {

  if (Serial5.available() > 0) {
    val_Weight = Serial5.read();
    if (val_Weight >= 100.00) (val_Weight = 0);
  }
}
// end of Weight() ###################################################


// sendSMS()##########################################################
//
//Start of SMS person1()==============================================
void sendSMS_person1() {
  // setting ke mode teks sms
  Serial3.write("AT+CMGF=1\r\n");
  delay(300);
  // setting nomor tujuan person #1
  Serial3.write("AT+CMGS=\"08161364504\"\r\n");  // nomor tujuan sms 08161364504 a/n : pak Ferry
  delay(200);
  // setting isi teks sms
  Serial3.printf("RMC PASAR_KOJA:$| "); delay(20);
  Serial3.printf("FD=%.4f", val_mtneFD_inpulse); delay(20); Serial3.print("m3| "); delay(20);
  Serial3.printf("FU=%.4f", val_mtneFU_inpulse); delay(20); Serial3.println("m3| "); delay(20);
  Serial3.printf("TS=%.4f", val_TS); delay(20); Serial3.println("m3| "); delay(20);  
  Serial3.printf("TA=%.2f", valTemp_A); delay(20); Serial3.print("C| "); delay(20);
  Serial3.printf("TL=%.2f", valTemp_L); delay(20); Serial3.println("C| "); delay(20);
  Serial3.printf("pH=%.2f", phNew); delay(20); Serial3.println("| "); delay(20);
  Serial3.printf("Wm_mamin=%.2f", val_Weight); delay(20); Serial3.print("kg| "); delay(20);
  Serial3.printf("Ws_sayur=%.2f", val_Weight); delay(20); Serial3.print("kg| "); delay(20);
  Serial3.printf("Wh_Hewan=%.2f", val_Weight); delay(20); Serial3.println("kg| "); delay(20);
  Serial3.printf("Wt_Total=%.2f", val_Weight); delay(20); Serial3.println("kg| "); delay(20);
  Serial3.printf("tMtr= ", val_hMtr); delay(20); 
  Serial3.printf(" : ", val_mMtr); delay(20); 
  //Serial3.printf(": ", val_tMtr); delay(20); 
  Serial3.print("| ");
  Serial3.print("#");     // # = end of data text  sms
  delay(300);
  // mengirim char ctrl+z/esc untuk keluar dari menu sms
  Serial3.write((char)26);  // CTRL-Z
  delay(300);
  Serial1.println("Terkirim data Biodegister kepada  person1 : 08161364504 a/n : Bapak Ferry Pimpinan Biodegister");
  delay(200);
  
  //End of SMS person1++++++++++++++++++++++++++++++++++++++++++++++++++
}

// Start of SMS person2() ==============================================
void sendSMS_person2() {
  // setting ke mode teks sms
  Serial3.write("AT+CMGF=1\r\n");
  delay(300);
  // setting nomor tujuan person #2
  Serial3.write("AT+CMGS=\"081513859847\"\r\n");  // nomor tujuan sms  081513859847 pak Yudhi
  delay(200);
  // setting isi teks sms
  Serial3.printf("RMC PASAR_KOJA:$| "); delay(20);
  Serial3.printf("FD=%.4f", val_mtneFD_inpulse); delay(20); Serial3.print("m3| "); delay(20);
  Serial3.printf("FU=%.4f", val_mtneFU_inpulse); delay(20); Serial3.println("m3| "); delay(20);
  Serial3.printf("TS=%.4f", val_TS); delay(20); Serial3.println("m3| "); delay(20);  
  Serial3.printf("TA=%.2f", valTemp_A); delay(20); Serial3.print("C| "); delay(20);
  Serial3.printf("TL=%.2f", valTemp_L); delay(20); Serial3.println("C| "); delay(20);
  Serial3.printf("pH=%.2f", phNew); delay(20); Serial3.println("| "); delay(20);
  Serial3.printf("Wm_mamin=%.2f", val_Weight); delay(20); Serial3.print("kg| "); delay(20);
  Serial3.printf("Ws_sayur=%.2f", val_Weight); delay(20); Serial3.print("kg| "); delay(20);
  Serial3.printf("Wh_Hewan=%.2f", val_Weight); delay(20); Serial3.println("kg| "); delay(20);
  Serial3.printf("Wt_Total=%.2f", val_Weight); delay(20); Serial3.println("kg| "); delay(20);
  Serial3.printf("tMtr= ", val_hMtr); delay(20); 
  Serial3.printf(" : ", val_mMtr); delay(20); 
  //Serial3.printf(": ", val_tMtr); delay(20); 
  Serial3.print("| ");
  Serial3.print("#");     // # = end of data text  sms
  delay(300);
  // mengirim char ctrl+z/esc untuk keluar dari menu sms
  Serial3.write((char)26);  // CTRL-Z
  delay(100);
  Serial1.println("Terkirim data Biodegister kepada  person2 : 081513859847 a/n : Bapak Yudhi Pelaksana Biodegister");
  delay(200);
}

// End of SMS person2+++++++++++++++++++++++++++++++++++++++++++++++++++++


// Start of SMS person3()=================================================

void sendSMS_person3() {
  // setting ke mode teks sms
  Serial3.write("AT+CMGF=1\r\n");
  delay(300);
  // setting nomor tujuan person #4
  Serial3.write("AT+CMGS=\"081295292924\"\r\n");  // nomor tujuan sms  085156808903  a/n : Rahmat IT
  delay(200);
  // setting isi teks sms
  Serial3.printf("RMC PASAR_KOJA:$| "); delay(20);
  Serial3.printf("FD=%.4f", val_mtneFD_inpulse); delay(20); Serial3.print("m3| "); delay(20);
  Serial3.printf("FU=%.4f", val_mtneFU_inpulse); delay(20); Serial3.println("m3| "); delay(20);
  Serial3.printf("TS=%.4f", val_TS); delay(20); Serial3.println("m3| "); delay(20);  
  Serial3.printf("TA=%.2f", valTemp_A); delay(20); Serial3.print("C| "); delay(20);
  Serial3.printf("TL=%.2f", valTemp_L); delay(20); Serial3.println("C| "); delay(20);
  Serial3.printf("pH=%.2f", phNew); delay(20); Serial3.println("| "); delay(20);
  Serial3.printf("Wm_mamin=%.2f", val_Weight); delay(20); Serial3.print("kg| "); delay(20);
  Serial3.printf("Ws_sayur=%.2f", val_Weight); delay(20); Serial3.print("kg| "); delay(20);
  Serial3.printf("Wh_Hewan=%.2f", val_Weight); delay(20); Serial3.println("kg| "); delay(20);
  Serial3.printf("Wt_Total=%.2f", val_Weight); delay(20); Serial3.println("kg| "); delay(20);
  Serial3.printf("tMtr= ", val_hMtr); delay(20); 
  Serial3.printf(" : ", val_mMtr); delay(20); 
  //Serial3.printf(": ", val_tMtr); delay(20); 
  Serial3.print("| ");
  Serial3.print("#");     // # = end of data text  sms
  delay(300);
  // mengirim char ctrl+z/esc untuk keluar dari menu sms
  Serial3.write((char)26);  // CTRL-Z
  delay(100);
  Serial1.println("Terkirim data Biodegister kepada  person3 : 081295292924 a/n : Bapak Rahmat IT Pelaksana Biodegister");
  delay(200);
  
}
// End of SMS person3 +++++++++++++++++++++++++++++++++++++++++++++++++++++


// End of SMS person4++++++++++++++++++++++++++++++++++++++++++++++++
//

// LOGGING Timbangan
// modeProtein()=============================================================
void modeProtein() {
  valLowProtein = digitalRead(SwLowProtein);    // read the input Low
  valHighProtein = digitalRead(SwHighProtein);  // read the input High
  valLowProtein = !valLowProtein;
  valHighProtein = !valHighProtein;
  if ((valLowProtein == true) && (valHighProtein == true)) {
    //lcd.setCursor(16,3);
    //lcd.print("B");
    char ModeBahan[15] = "Both Mode";
    Serial1.println("Both Mode");
  } else {
    if (valLowProtein == true) {
      //lcd.setCursor(16,3);
      //lcd.print("L");
      char ModeBahan[15] = "Low Protein";
      Serial1.println("Low Protein");
    } else {
      if (valHighProtein == true) {
        //lcd.setCursor(16,3);
        //lcd.print("H");
        char ModeBahan[15] = "High Protein";
        Serial1.println("High Protein");
      } else {
        //lcd.setCursor(16,3);
        //lcd.print("N");
        char ModeBahan[15] = "None Mode";
        Serial1.println("None Mode");
      }
    }
  }
}

void exeSMSgatway() {
  cloop_mc++;
  if (cloop_mc >= 6) {  // one of a menute
    cloop_mc = 0;
    cloop_mt++;
  }
  if (cloop_mt >= 60) {  // one of a hour
    cloop_mc = 0;
    cloop_mt = 0;
    cloop_h++;
  }
  if (cloop_h >= 6) {  // one of an exeSMSgateway
    val_btSendSMS = false;
    cloop_mc = 0;
    cloop_mt = 0;
    cloop_h = 0;
  }
}

// End of Sub Routine &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//setup()###########################################################################
void setup() {
  //Initial Speed of Serial port.............
  Serial.begin(9600);     // speed rate timbangan monitor RealTerm FD
  Serial1.begin(9600);    // speed rate Local PC monitoring
  Serial2.begin(9600);    // speed rate FD
  Serial3.begin(115200);  // speed rate SMS
  Serial5.begin(4800);    // speed rate Weight record

  Serial4.begin(9600);    // speed data transfer for FD
  Serial6.begin(9600);    // speed data transfer for FU
  // initialize the lcd -----------------------
  lcd.init();
  lcd.clear();
  //Initial GPIO -------------------------------
  pinMode(led, OUTPUT);   

  // Dipindah ker sub processor Stand-alone
  /*
  //Initial FD :
  pinMode(turbinFDPin, INPUT);
  mtneFD_pulse = 0.001406846;  // methane per pulse in m3
  turbinFDPushCounter = EEPROM.read(addrFD); delay (100);  // retrive data fPROM addrFD;

  //Initial FU :
  pinMode(turbinFUPin, INPUT);
  mtneFU_pulse = 0.001406846;  // methane per pulse in m3
  turbinFUPushCounter = EEPROM.read(addrFU); delay (100);  // retrive data fPROM addrFU;
  */
  // Batas yg dipindah pada sub processor stand-alone


  //Initial Weight
  pinMode(SwMakMin, INPUT);
  pinMode(SwLowProtein, INPUT);
  pinMode(SwHighProtein, INPUT);

  pinMode(btSendSMS, INPUT);
  
  val_btSendSMS = true;
  lcd.createChar(0, degree);
  //delay(500);
  lcd_splash();
  pc_splash();
  cloop_mc = 0;
  cloop_mt = 0;
  cloop_h = 0;
  cloop_exeSMSgateway = 0;

  // initial timer_motor
  val_hMtr        = 00;  // hour
  addr_val_hMtr   = 30;  // alamat EEPROM hMtr
  val_mMtr        = 00;  // menute
  addr_val_mMtr   = 31;  // alamat EEPROM hMtr
  val_tMtr        = 00;  // second
  addr_val_tMtr   = 32;  // alamat EEPROM hMtr

  //digitalWrite(beep, HIGH);  // beep = off  ; active Low
}
// End of Setup()######################################################

float const_kalibtasi_TA = 0.00;
float const_kalibtasi_TL = 0.00;


//loop()  #############################################################
void loop() {
  //lcd_flow datalogging   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
  
  //FD: Flow data logging $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
  FD_methane_Logging();  // call sub program
  val_mtneFD_inpulse = turbinFDPushCounter * mtneFD_pulse;
  //val_mtneFD_inpulse = 0.2124;
  
  //lcd display FD  :
  lcd.setCursor(0, 0);
  lcd.printf("FD:%.3f", val_mtneFD_inpulse);  // the output is in a "val_mtneFD_inpulse"
  //lcd.printf("m3");

  //FU: Flow data loging #########################################################
  FU_methane_Logging();  // call sub program
  
  if (Serial6.available()) {    // T4.0 Tx  to  Rx T4.1 ;  T4.0 Rx to Tx T4.1
    // read the incoming byte:
    turbinFUPushCounter = Serial6.read();        // baca Serial6
    //val_mtneFU_inpulse = Serial6.read();   
    Serial.println(turbinFUPushCounter);
  }
  
  val_mtneFU_inpulse = turbinFUPushCounter * mtneFU_pulse;
  //val_mtneFU_inpulse = 0.1238;
  //lcd display FU  :
  lcd.setCursor(10, 0);
  lcd.printf("FU:%.3f", val_mtneFU_inpulse);  // the output is in a "val_mtneFU_inpulse"
  //lcd.print(turbinFUPushCounter);  // the output is in a "val_mtneFU_inpulse"

  //lcd.setCursor(18, 0);
  //lcd.printf("m3");

  //TS (Tanki Storage): gas data loging ##########################################################
  val_TS = (val_mtneFD_inpulse) - (val_mtneFU_inpulse);
  if (val_TS < 0) val_TS = 0;
  
  //lcd display TS  :
  lcd.setCursor(0, 1);
  lcd.printf("TS:%.4f", val_TS);


  // Temperatur Ambeien --------------------------
  DHT.read(DHT11_PINA);
  delay(20);
  if (DHT.temperature >= 225) DHT.temperature = 0;
  DHT_temp_A = (DHT.temperature);
  delay(50);
  valTemp_A = (DHT_temp_A);

  lcd.setCursor(0, 2);
  lcd.printf("TA:%.2f", valTemp_A);
  lcd.print("C ");

  // Temperatur Liquid--------------------------
  valTemp_L = thermocouple.readCelsius();

  // display TL
  lcd.setCursor(10, 2);
  lcd.printf("TL:%.2f", valTemp_L);
  lcd.print("C");

  //lcd_pH----------------------------------------------
  pH_Logging();
  if (phNew > 14) {
    phNew = 14;
  } else {
    if (phNew < 0)
      phNew = 0;
  }

  lcd.setCursor(12, 1);
  lcd.printf("pH:%.2f", phNew);
  


  //lcd_weight-------------------------------------------
  Weight_Logging();

  lcd.setCursor(0, 0);
  lcd.printf("Wt:%.2f", val_Weight);
  

  lcd.setCursor(17, 3);
  lcd.print(")");
  delay(20);  // send data to SMS indicator
  // END OF lcd pANEL----------------------------------------

  // PC Panel===============================================
  Serial1.println();
  Serial1.println("Hasil Record : BIODIGESTER Renik Methane Capture");
  Serial1.println();
  Serial1.print("Model bahan baku :  ");

  //modeProtein();

  //delay(100);
  lcd.setCursor(18, 3);
  lcd.print(")");
  delay(20);

  //Weight Record (in Kg)
  Serial1.println();
  Serial1.print("Waktu : ");
  //exe :   time_log()----------------------------------
  //time_log();
  Serial1.println();
  //end of exe : time_log()-----------------------------
  // weight Record ---------------------------------------------------
  Serial1.print("Berat bahan  : ");
  Serial1.print(val_Weight);
  Serial1.print("Kg");
  Serial1.println();

  //pH Record---------------------------------------------------------
  
  Serial1.print("pH Liquid    : ");
  Serial1.print(phNew);
  Serial1.println("     .");



  // temperatur record

  //valTemp_A = DHT.temperature;
  Serial1.printf("Temp Ambient :%.2f", valTemp_A);
  Serial1.print(" C");

  //DHT.read(DHT11_PINL);
/*
  Serial1.print("Temp Liquid  : ");
  Serial1.print(valTemp_L);
  Serial1.println(" C");
  Serial1.println("   ");
*/
  // flow record
  Serial1.printf("Flow gas (FD) (Arus hasil Digester) : %.6f", val_mtneFD_inpulse);
  //Serial1.print(val_FD);
  Serial1.print(" m3");
  Serial1.println("  .");
  Serial1.printf("Flow gas (FU) (Arus pemakaian Gas)  : %.6f", val_mtneFU_inpulse);
  //Serial1.print(val_FU);
  Serial1.print(" m3");
  Serial1.println();

  Serial1.printf("Storage Gas (TS = FD - FU )         : %.6f", val_TS);
  //Serial1.print(val_TS);
  Serial1.print(" m3");
  Serial1.println();
  delay(50);
  lcd.setCursor(19, 3);
  lcd.print(")");
  delay(30);        // send SMS indicator
  Serial1.println("--------------------------------------------------");
  delay(10);
  lcd.setCursor(17, 3);
  lcd.print("   ");         // clear SMS indicator
  digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(50);                // wait for a second

  // check Interval exeSMSgateway() #############################################################
  exeSMSgatway();
  //check Switch Request SMSgateway
  val_btSendSMS = digitalRead(btSendSMS);
  //
  //val_btSendSMS = !val_btSendSMS;
  if (val_btSendSMS == false) {
    Serial1.println(".... Data SMS SUDAH DIKIRIM  ke No ....");
    sendSMS_person1(); delay(200);
    sendSMS_person2(); delay(200);
    sendSMS_person3(); delay(200);
 
    /*
    Serial1.println(".... person1 : 08161364504   pak Ferry Pimpinan Biodegister ");  // pak Ferry Boss Degister
    Serial1.println(".... person2 : 081513859847  pak Yudhi Pelaksana Biodegister ");  // pak Yudhi Pelaksana Degister
    Serial1.println(".... person3 : 081295292924  pak Rahmat Pelaksana IT Biodegister ");  // pak Rachmat Komputer. lama085156808903 baru 081295292924
    */
    Serial1.println("---------------------------------------");

  }

  val_btSendSMS = true;

  // End of SMS Gatway ----------------------------------------------

  //
  digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW
  delay(50);               // wait for a second
}


// End of loop() &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
