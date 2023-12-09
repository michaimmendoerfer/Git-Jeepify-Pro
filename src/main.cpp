#include <Arduino.h>
#include <TFT_eSPI.h>
#include "TAMC_GT911.h"
#include <Adafruit_ADS1X15.h>
#include "../../jeepify.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#include "NotoSansMonoSCB20.h"

#define NODE_NAME "Jeep_PRO_V1"
#define NODE_TYPE BATTERY_SENSOR
#define VERSION   "V 0.71"

#define NAME_SENSOR_0 "Bat0"
#define NAME_SENSOR_1 "Bat1"
#define NAME_SENSOR_2 "Bat2"
#define NAME_SENSOR_3 "Bat3"
#define NAME_SENSOR_4 "Bat4"

#define PIN_VOLTAGE    35

#define D5 12
#define D6 13

TAMC_GT911 tp = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);
Adafruit_ADS1115 ads;

struct_Sensor S[NODE_TYPE];
struct_Peer   P[MAX_PEERS];

struct_Touch Touch;

StaticJsonDocument<300> doc;
String jsondata;

struct struct_Status {
  String    Msg;
  uint32_t  TSMsg;
};

struct_Status Status[MAX_STATUS];
u_int8_t TempBroadcast[6];

int PeerCount = 0;

bool Debug         = true;
bool SleepMode     = true;
bool ReadyToPair   = false;

bool ScreenChanged = true;
int  UpdateCount   = 0;
int  Mode          = S_STATUS;
int  OldMode       = 0;

uint32_t TSLastSend    = 0;
uint32_t TSLastContact = 0;
uint32_t TSScreenRefresh = 0;
uint32_t TSSend = 0;
uint32_t TSPair = 0;
uint32_t TSTouch = 0;

Preferences preferences;

TFT_eSPI TFT               = TFT_eSPI();
TFT_eSprite TFTBuffer      = TFT_eSprite(&TFT);
TFT_eSprite TFTGaugeSwitch = TFT_eSprite(&TFT);

float  ReadAmp (int A);
float  ReadVolt(int V);
void   SendMessage();
void   SendPairingRequest();

void   OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void   OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void   InitModule();
void   SavePeers();
void   GetPeers();
void   ReportPeers();
void   RegisterPeers();
void   ClearPeers();
void   ClearInit();

void   PushTFT();
void   ShowPairingScreen();
void   ShowEichen();
void   ShowVoltCalib(float V);

void   SetSleepMode(bool Mode);
void   SetDebugMode(bool Mode);
void   AddStatus(String Msg);

void   Eichen();
void   PrintMAC(const uint8_t * mac_addr);

int    TouchRead();

bool   isPeerEmpty(int PNr);
bool   isSensorEmpty(int SNr);


void InitModule() {
  preferences.begin("JeepifyInit", true);
  Debug     = preferences.getBool("Debug", true);
  SleepMode = preferences.getBool("SleepMode", false);
  
  strcpy(S[0].Name, NAME_SENSOR_0);
  S[0].Type     = 1;
  S[0].IOPort   = 0;
  S[0].NullWert = preferences.getInt("Null-0", 3134);
  S[0].VperAmp  = preferences.getFloat("Sens-0", 0.066);

  strcpy(S[1].Name, NAME_SENSOR_1);
  S[1].Type     = 1;
  S[1].IOPort   = 1;
  S[1].NullWert = preferences.getInt("Null-1", 3134);
  S[1].VperAmp  = preferences.getFloat("Sens-1", 0.066);

  strcpy(S[2].Name, NAME_SENSOR_2);
  S[2].Type     = 1;
  S[2].IOPort   = 2;
  S[2].NullWert = preferences.getInt("Null-2", 3150);
  S[2].VperAmp  = preferences.getFloat("Sens-2", 0.066);

  strcpy(S[3].Name, NAME_SENSOR_3);
  S[3].Type     = 1;
  S[3].IOPort   = 3;
  S[3].NullWert = preferences.getInt("Null-3", 3150);
  S[3].VperAmp  = preferences.getFloat("Sens-3", 0.066);

  strcpy(S[4].Name, NAME_SENSOR_4);
  S[4].Type     = 2;
  S[4].IOPort   = PIN_VOLTAGE;
  S[4].Vin      = preferences.getInt("Vin", 200);
  S[4].VperAmp  = 1;

  preferences.end();

  Serial.println("InitModule() fertig...");
}
void SavePeers() {
  Serial.println("SavePeers...");
  preferences.begin("JeepifyPeers", false);
  preferences.clear();
  
  char Buf[10] = {}; char BufNr[5] = {}; char BufB[5] = {}; String BufS;

  PeerCount = 0;

  for (int Pi=0; Pi< MAX_PEERS; Pi++) {
    if (!isPeerEmpty(Pi)) {
      PeerCount++;
      //P.Type...
      sprintf(BufNr, "%d", Pi); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].Type);
      preferences.putInt(Buf, P[Pi].Type);
      
      //P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = P[Pi].Name;
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(BufS);
      preferences.putString(Buf, BufS);
      
      //P.BroadcastAdress
      for (int b=0; b<6; b++) {
        sprintf(BufB, "%d", b); 
        strcpy(Buf, "MAC"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        preferences.putBytes(Buf, P[Pi].BroadcastAddress, 6);
      }
    }
  }
  preferences.putInt("PeerCount", PeerCount);

  preferences.end();
}
void GetPeers() {
  preferences.begin("JeepifyPeers", true);
  
  char Buf[10] = {}; char BufNr[5] = {}; char BufB[5] = {}; String BufS;
  
  PeerCount = 0;
  for (int Pi=0; Pi<MAX_PEERS; Pi++) {
    sprintf(BufNr, "%d", Pi); 

    // Peer gefüllt?
    strcpy(Buf, "Type-"); strcat(Buf, BufNr);
    if (Debug) { Serial.print("getInt("); Serial.print(Buf); Serial.print(" = "); Serial.print(preferences.getInt(Buf)); }
    if (preferences.getInt(Buf,0) > 0) {
      PeerCount++;
      // P.Type
      P[Pi].Type = preferences.getInt(Buf);
      // P.BroadcastAdress
      strcpy(Buf, "MAC-"); strcat (Buf, BufNr);
      preferences.getBytes(Buf, P[Pi].BroadcastAddress, 6);
      
      P[Pi].TSLastSeen = millis();
      
      // P.Name
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      BufS = preferences.getString(Buf);
      strcpy(P[Pi].Name, BufS.c_str());
      
      if (Debug) {
        Serial.print("GetPeers: P["); Serial.print(Pi); Serial.print("]: Type="); Serial.print(P[Pi].Type); 
        Serial.print(", Name="); Serial.print(P[Pi].Name);
        Serial.print(", MAC="); PrintMAC(P[Pi].BroadcastAddress); Serial.println();
      }
    }
  }
  preferences.end();
}
void ShowAllPreferences() {
  preferences.begin("JeepifyPeers", true);
  
  char Buf[10] = {}; char BufNr[5] = {}; char BufB[5] = {}; String BufS;
  
  PeerCount = 0;
  for (int Pi=0; Pi<MAX_PEERS; Pi++) {
    // Peer gefüllt?
    sprintf(BufNr, "%d", Pi); strcpy(Buf, "Type-"); strcat(Buf, BufNr);
    Serial.print("getInt("); Serial.print(Buf); Serial.print(" = "); Serial.print(preferences.getInt(Buf));
    if (preferences.getInt(Buf) > 0) {
      PeerCount++;
      // P.Type
      Serial.print(Buf); Serial.print(" = "); Serial.print(preferences.getInt(Buf));
      // P.BroadcastAdress
      for (int b=0; b<6; b++) {
        sprintf(BufB, "%d", b); 
        strcpy(Buf, "B"); strcat(Buf, BufB); strcat (Buf, "-"); strcat (Buf, BufNr);
        Serial.print(Buf); Serial.print(" = "); Serial.print(preferences.getUChar(Buf));
      }
      Serial.println();
      strcpy(Buf, "Name-"); strcat(Buf, BufNr);
      // P.Name
      Serial.print(Buf); Serial.print(" = "); Serial.println(preferences.getString(Buf));
    }
    preferences.end();
  }
  preferences.begin("JeepifyInit", true);
  Serial.print("Debug = "); Serial.print(preferences.getBool("Debug"));
  Serial.print("SleepMode = "); Serial.print(preferences.getBool("SleepMode"));
  Serial.print("Null-0 = "); Serial.println(preferences.getInt("Null-0"));
  Serial.print("Null-1 = "); Serial.println(preferences.getInt("Null-1"));
  Serial.print("Null-2 = "); Serial.println(preferences.getInt("Null-2"));
  Serial.print("Null-3 = "); Serial.println(preferences.getInt("Null-3"));
  Serial.print("Sens-0 = ");    Serial.println(preferences.getFloat("Sens-0"));
  Serial.print("Sens-1 = ");    Serial.println(preferences.getFloat("Sens-1"));
  Serial.print("Sens-2 = ");    Serial.println(preferences.getFloat("Sens-2"));
  Serial.print("Sens-3 = ");    Serial.println(preferences.getFloat("Sens-3"));
  Serial.print("Vin = ")   ; Serial.println(preferences.getInt("Vin"));
  
  preferences.end();
  delay(30000);
}
void ReportPeers() {
  TFT.fillScreen(TFT_BLACK);
  TFT.setCursor(0, 0, 2);
  TFT.setTextColor(TFT_WHITE,TFT_BLACK);  TFT.setTextSize(1);
  
  TFT.println("Peer-Report:");
  TFT.println();

  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (Debug) {
      Serial.println(P[PNr].Name);
      Serial.println(P[PNr].Type);
      Serial.print("MAC: "); PrintMAC(P[PNr].BroadcastAddress);
      Serial.println();
    }
    
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           P[PNr].BroadcastAddress[0], P[PNr].BroadcastAddress[1], P[PNr].BroadcastAddress[2], 
           P[PNr].BroadcastAddress[3], P[PNr].BroadcastAddress[4], P[PNr].BroadcastAddress[5]);

    TFT.print("["); TFT.print(PNr); TFT.print("]-"); TFT.print(P[PNr].Name); TFT.print(": - Type:"); TFT.print(P[PNr].Type);
    TFT.print(" - MAC:"); TFT.println(macStr);
  }
}
void RegisterPeers() {
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  memset(&peerInfo, 0, sizeof(peerInfo));

  // Register BROADCAST
  for (int b=0; b<6; b++) peerInfo.peer_addr[b] = (uint8_t) broadcastAddressAll[b];
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      PrintMAC(peerInfo.peer_addr); Serial.println(": Failed to add peer");
    }
    else {
      Serial.print (" ("); PrintMAC(peerInfo.peer_addr);  Serial.println(") added...");
    }

  // Register Peers
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (!isPeerEmpty(PNr)) {
      for (int b=0; b<6; b++) peerInfo.peer_addr[b] = (uint8_t) P[PNr].BroadcastAddress[b];
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
          PrintMAC(peerInfo.peer_addr); Serial.println(": Failed to add peer");
        }
        else {
          Serial.print("Peer: "); Serial.print(P[PNr].Name); 
          Serial.print (" ("); PrintMAC(peerInfo.peer_addr); Serial.println(") added...");
        }
    }
  }
}
void ClearPeers() {
  preferences.begin("JeepifyPeers", false);
    preferences.clear();
    Serial.println("JeepifyPeers cleared...");
  preferences.end();
}
void ClearInit() {
  preferences.begin("JeepifyInit", false);
    preferences.clear();
    Serial.println("JeepifyInit cleared...");
  preferences.end();
}
void ShowPairingScreen() {
  if (OldMode != S_PAIRING) TSScreenRefresh = millis();
  if ((millis() - TSScreenRefresh > 1000) or (Mode != OldMode)) {
    char Buf[100] = {}; 
    char macStr[18];

    OldMode = Mode;
    ScreenChanged = true;

    TFTBuffer.loadFont(AA_FONT_LARGE);
    
    TFTBuffer.setTextColor(TFT_WHITE, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);

    TFT.fillScreen(TFT_BLACK);

    TFT.drawString("Waiting for Host...", 10, 30);

    int h=20;
    for (int PNr=0; PNr<MAX_PEERS; PNr++) {
      snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            P[PNr].BroadcastAddress[0], P[PNr].BroadcastAddress[1], P[PNr].BroadcastAddress[2], 
            P[PNr].BroadcastAddress[3], P[PNr].BroadcastAddress[4], P[PNr].BroadcastAddress[5]);
      sprintf(Buf, "[%d] %s: MAC= %s, Type=%d", PNr, P[PNr].Name, macStr, P[PNr].Type);
      
      TFT.drawString(Buf, 10, 30+(PNr+1)*h);
    }
    TSScreenRefresh = millis();
  }
}
void PushTFT() {
  if (ScreenChanged) {
    Serial.print("ScreenUpdate: ");
    Serial.println(UpdateCount);
    TFTBuffer.pushSprite(0, 0);
    ScreenChanged = false;
  }
}
void SendMessage () {
  char buf[10];
  doc.clear();
  jsondata = "";

  doc["Node"] = NODE_NAME;   

  for (int SNr=0; SNr<MAX_PERIPHERALS ; SNr++) {
    if (S[SNr].Type == SENS_TYPE_SWITCH) {
      doc[S[SNr].Name] = S[SNr].Value;
    }
    if (S[SNr].Type == SENS_TYPE_AMP) {
      S[SNr].Value = ReadAmp(SNr);
      dtostrf(S[SNr].Value, 0, 2, buf);
      doc[S[SNr].Name] = buf;
    }
    if (S[SNr].Type == SENS_TYPE_VOLT) {
      S[SNr].Value = ReadVolt(SNr);
      dtostrf(S[SNr].Value, 0, 2, buf);
      doc[S[SNr].Name], buf;
    }
  }

  serializeJson(doc, jsondata);  
  
  for (int PNr=0; PNr<MAX_PEERS; PNr++) {
    if (P[PNr].Type > 9) {
      Serial.print("Sending to: "); Serial.println(P[PNr].Name); 
      Serial.print(" ("); PrintMAC(P[PNr].BroadcastAddress); Serial.println(")");
      esp_now_send(P[PNr].BroadcastAddress, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
      Serial.println(jsondata);
      Serial.println();
    }
  }

  if (Debug) { Serial.print("\nSending: "); Serial.println(jsondata); }
  AddStatus(jsondata);
}
void SendPairingRequest() {
  char Buf[10] = {};

  jsondata = "";  //clearing String after data is being sent
  doc.clear();
  
  doc["Node"]    = NODE_NAME;   
  doc["Type"]    = NODE_TYPE;
  doc["Pairing"] = "add me";
  
  for (int Si=0 ; Si<MAX_PERIPHERALS; Si++) {
    if (S[Si].Type > 0) {
      sprintf(Buf, "S%d", Si); 
      doc[Buf] = S[Si].Name;
    }
  }
  serializeJson(doc, jsondata);  

  esp_now_send(broadcastAddressAll, (uint8_t *) jsondata.c_str(), 200);  //Sending "jsondata"  
  
  if (Debug) { Serial.print("\nSending: "); Serial.println(jsondata); }
  AddStatus(jsondata);                                       
}
void SetSleepMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    SleepMode = Mode;
    if (preferences.getBool("SleepMode", false) != SleepMode) preferences.putBool("SleepMode", SleepMode);
  preferences.end();
}
void SetDebugMode(bool Mode) {
  preferences.begin("JeepifyInit", false);
    Debug = Mode;
    if (preferences.getBool("Debug", false) != Debug) preferences.putBool("Debug", Debug);
  preferences.end();
}
void ShowEichen() {
  if (OldMode != S_EICHEN) TSScreenRefresh = millis(); 
  if ((TSScreenRefresh - millis() > 1000) or (Mode != OldMode)) {
    char Buf[100] = {};
  
    OldMode = Mode;
    ScreenChanged = true;
    
    TFTBuffer.loadFont(AA_FONT_LARGE);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);

    TFT.fillScreen(TFT_BLACK);
  
    TFT.drawString("Eichen...", 10, 30);

    if (Debug) Serial.println("Eichen...");
    preferences.begin("JeepifyInit", false);
    
    int h=20;
    for(int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
      if (S[SNr].Type == SENS_TYPE_AMP) {
        float TempVal  = ads.readADC_SingleEnded(S[SNr].IOPort);
        float TempVolt = ads.computeVolts(TempVal);
        if (Debug) { 
          Serial.print("TempVal:");     Serial.println(TempVal);
          Serial.print(", TempVolt: "); Serial.println(TempVolt);
        }
        S[SNr].NullWert = TempVolt;
        sprintf(Buf, "Null-%d", SNr); 
        preferences.putFloat(Buf, S[SNr].NullWert);
        if (Debug) {
          Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(S[SNr].NullWert); 
        }

        dtostrf(TempVolt, 0, 2, BufNr);
        sprintf(Buf, "Eichen fertig: [%d] %s (Type: %d): Gemessene Spannung bei Null: %sV", SNr, S[SNr].Name, S[SNr].Type, BufNr);
        TFT.drawString(Buf, 10, 30+SNr+1*h);
        AddStatus(Buf);
      }
    }
    preferences.end();
    
    delay(5000);
    
    Mode = S_STATUS;
    
    TSScreenRefresh = millis();
  }
}
void AddStatus(String Msg) {
  for (int Si=MAX_STATUS-1; Si>0; Si--) {
    Status[Si].Msg   = Status[Si-1].Msg;
    Status[Si].TSMsg = Status[Si-1].TSMsg;
  }
  Status[0].Msg = Msg;
  Status[0].TSMsg = millis();
}
void ShowStatus() {
  if (OldMode != S_STATUS) { TSScreenRefresh = millis(); TFT.fillScreen(TFT_BLACK); }
  if ((millis() - TSScreenRefresh > 1000) or (Mode != OldMode)) {
    OldMode = Mode;
    ScreenChanged = true;
    
    TFTBuffer.loadFont(AA_FONT_LARGE);
    
    TFTBuffer.setTextColor(TFT_RUBICON, TFT_BLACK);
    TFTBuffer.setTextDatum(TC_DATUM);

    TFT.fillScreen(TFT_BLACK); 
    TFT.drawString("Status...", 10, 30);

    int h=20;
    for(int SNr=0; SNr<MAX_STATUS; SNr++) {
      char Buf[20];
      sprintf(Buf, "%d%:2d:%2d", (int)Status[SNr].TSMsg/360000%60, (int)Status[SNr].TSMsg/60000%60, (int)Status[SNr].TSMsg/1000%60);
      TFT.drawString(Buf, 10, 30+(SNr+1)*h);
      TFT.drawString(Status[SNr].Msg, 75, 30+(SNr+1)*h);
    }
    TSScreenRefresh = millis();
  }
}
void ShowVoltCalib(float V) {
  char Buf[100] = {}; char BufNr[10] = {}; 
  
  if (OldMode != S_VOLTCALIB) TSScreenRefresh = millis(); 
  if ((millis() - TSScreenRefresh > 1000) or (Mode != OldMode)) {
    OldMode = Mode;
    ScreenChanged = true;
    
    TFT.fillScreen(TFT_BLACK);
  
    TFT.drawString("Volt-Messung kalibrieren...", 10, 30);

    if (Debug) Serial.println("Volt eichen...");
    preferences.begin("JeepifyInit", false);
    
    int h=30;
    for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++){
      if (S[SNr].Type = SENS_TYPE_VOLT) {
        int TempRead = analogRead(S[SNr].IOPort);
        
        if (Debug) {
          Serial.print("S["); Serial.print(SNr); Serial.print("].Vin = ");
          Serial.println(S[SNr].Vin, 4);
          Serial.print("Volt(nachher) = ");
          Serial.println(TempRead/S[SNr].Vin, 4);
        }
        S[SNr].Vin = TempRead / V;
        
        preferences.begin("JeepifyInit", false);
        preferences.putFloat("Vin", S[SNr].Vin);
        preferences.end();

        dtostrf(TempRead, 0, 2, BufNr);
        sprintf(Buf, "[%d] %s (Type: %d): Gemessene Spannung bei Null: %sV", SNr, S[SNr].Name, S[SNr].Type, BufNr);
        TFT.drawString(Buf, 10, 30+SNr+1*h);

        break;
      }
    }
    
    delay(5000);
    
    Mode = S_MENU;
    
    TSScreenRefresh = millis();
  }
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char* buff = (char*) incomingData;        //char buffer
  jsondata = String(buff);                  //converting into STRING
  Serial.print("Recieved ");
  Serial.println(jsondata);    //Complete JSON data will be printed here
  
  DeserializationError error = deserializeJson(doc, jsondata);

  if (!error) {
    if (ReadyToPair) {
      if (doc["Pairing"] == "you are paired") { 
        //for (int b=0; b < 6; b++ ) TempBroadcast[b] = (uint8_t) mac[b];
        
        bool exists = esp_now_is_peer_exist(mac);
        if (exists) { 
          PrintMAC(mac); Serial.println(" already exists...");
        }
        else {
          bool PairingSuccess = false;
          for (int PNr=0; PNr<MAX_PEERS; PNr++) {
            Serial.print("P["); Serial.print(PNr); Serial.print("].Type = "); Serial.println(P[PNr].Type);
            if ((P[PNr].Type == 0) and (!PairingSuccess)) {
              Serial.println("leerer Slot gefunden");
              P[PNr].Type = (int) doc["Type"];
              strcpy(P[PNr].Name, doc["Node"]);
              
              for (int b = 0; b < 6; b++ ) P[PNr].BroadcastAddress[b] = mac[b];
              P[PNr].TSLastSeen = millis();
              
              PairingSuccess = true; 
              SavePeers();
              //ShowAllPreferences();
              RegisterPeers();
              
              if (Debug) {
                Serial.print("Adding in slot: "); Serial.println(PNr);
                Serial.print("Name: "); Serial.print(P[PNr].Name);
                Serial.print(" (");PrintMAC(P[PNr].BroadcastAddress); Serial.println(")\n");
                Serial.print("Saving Peers after received new one...");
                ReportPeers();
              }
            }
          }
          if (!PairingSuccess) { PrintMAC(mac); Serial.println(" adding failed..."); } 
        }
      }
    }
    else { // (!ReadyToPair)
      float TempSens;
      if (doc["Order"] == "stay alive") {
        TSLastContact = millis();
        if (Debug) { Serial.print("LastContact: "); Serial.println(TSLastContact); }
      }
      if (doc["Order"] == "SleepMode On")  { AddStatus("Sleep: on");  SetSleepMode(true); }
      if (doc["Order"] == "SleepMode Off") { AddStatus("Sleep: off"); SetSleepMode(false); }
      if (doc["Order"] == "Debug on")      { AddStatus("Debug: on");  SetDebugMode(true); }
      if (doc["Order"] == "Debug off")     { AddStatus("Debug: off"); SetDebugMode(false); }
      if (doc["Order"] == "Reset")         { AddStatus("Clear all"); ClearPeers(); ClearInit(); }
      if (doc["Order"] == "Restart")       { ESP.restart(); }
      if (doc["Order"] == "Pair")          { TSPair = millis(); ReadyToPair = true; AddStatus("Pairing beginnt"); }

      // BatterySensor
      if (NODE_TYPE == BATTERY_SENSOR) {
        if (doc["Order"] == "Eichen")      { Mode = S_EICHEN;    AddStatus("Eichen beginnt"); ShowEichen(); }
        if (doc["Order"] == "VoltCalib")   { Mode = S_VOLTCALIB; AddStatus("VoltCalib beginnt"); ShowVoltCalib((float)doc["Value"]); }
      }
      // PDC
      if ((NODE_TYPE == SWITCH_1_WAY) or (NODE_TYPE == SWITCH_2_WAY) or
          (NODE_TYPE == SWITCH_4_WAY) or (NODE_TYPE == SWITCH_8_WAY)) {
        if (doc["Order"]   == "ToggleSwitch") {
          for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++) {
            if (S[SNr].Name == doc["Value"]) S[SNr].Value = !S[SNr].Value; 
            String Nr = doc["Value"];
            AddStatus("ToggleSwitch "+Nr);
          }
          SendMessage();
        }      
      }
    }
  } // end (!error)
  else {  // error
        Serial.print(F("deserializeJson() failed: "));  //Just in case of an ERROR of ArduinoJSon
        Serial.println(error.f_str());
        return;
  }
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { 
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void setup() {
  Serial.begin(74880);
  
  tp.begin();
  tp.setRotation(ROTATION_LEFT);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
  //ClearPeers();
  TFT.init();
  TFT.setRotation(3);
  //TFT.setSwapBytes(true);
  TFT.fillScreen(TFT_BLACK);

  //Wire.begin(D5, D6);
  //ads.setGain(GAIN_TWOTHIRDS);  // 0.1875 mV/Bit .... +- 6,144V
  //ads.begin();
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("Error initializing ESP-NOW"); return; }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);    

  Serial.println("InitModule...");
  InitModule();
  Debug = true;

  Serial.println("GetPeers...");
  GetPeers();
  Serial.println("ReportPeers...");
  ReportPeers();
  Serial.println("RegisterPeers...");
  RegisterPeers();
  Serial.println("RegisterPeers fertig...");

  if (PeerCount == 0) { ReadyToPair = true; TSPair = millis(); }
  
  TSScreenRefresh = millis();

  AddStatus("Init fertig");
  ReadyToPair = true;
  TSPair = millis();
  AddStatus("Pairing beginnt"); 
  //free Pins
}
void loop() {
  int G;
  if  ((millis() - TSTouch) > TOUCH_INTERVAL) {
    G = TouchRead();
    TSTouch = millis();
    if (G == CLICK) {
      if (Mode == S_STATUS) Mode = S_PAIRING;
      else if (Mode == S_PAIRING) Mode = S_STATUS;
    } 
  }
  if  ((millis() - TSSend ) > MSG_INTERVAL  ) {
    TSSend = millis();
    if (ReadyToPair) SendPairingRequest();
    else SendMessage();
  }
  if (((millis() - TSPair ) > PAIR_INTERVAL ) and (ReadyToPair)) {
    TSPair = 0;
    ReadyToPair = false;
    AddStatus("Pairing beendet...");
    Mode = S_STATUS;
  }
  
  switch (Mode) {
    case S_PAIRING: ShowPairingScreen();
    case S_STATUS:  ShowStatus();
  }
  //PushTFT();
}
float ReadAmp (int A) {
  /*float TempVal = ads.readADC_SingleEnded(S[A].IOPort);
  float TempVolt = ads.computeVolts(TempVal); 
  float TempAmp = (TempVolt - S[A].NullWert) / S[A].VperAmp;

  if (Debug) {
    Serial.print("TempVal:  "); Serial.println(TempVal,4);
    Serial.print("TempVolt: "); Serial.println(TempVolt,4);
    Serial.print("Nullwert: "); Serial.println(S[A].NullWert,4);
    Serial.print("VperAmp:  "); Serial.println(S[A].VperAmp,4);
    Serial.print("TempAmp:  "); Serial.println(TempAmp,4);
  } 
  if (abs(TempAmp) < SENSOR_SCHWELLE) TempAmp = 0;
  */
  return (1.3); //TempAmp;
}
float ReadVolt(int V) {
  if (!S[V].Vin) { Serial.println("Vin must not be zero !!!"); return 0; }
  
  float TempVal  = analogRead(S[V].IOPort);
  float TempVolt = TempVal / S[V].Vin;
  
  if (Debug) {
    Serial.print("TempVal:  "); Serial.println(TempVal,4);
    Serial.print("Vin:      "); Serial.println(S[V].Vin);
    Serial.print("Volt: ");     Serial.println(TempVolt,4);
    
  } 
  return TempVolt;
}
bool  isPeerEmpty(int PNr) {
  return (P[PNr].Type == 0);
}
bool  isSensorEmpty(int SNr) {
  return (S[SNr].Type == 0);
}
int   TouchRead() {
  uint16_t TouchX, TouchY;
  uint8_t  Gesture;
  bool TouchContact;

  int ret = 0;

  tp.read();
  
  TouchContact = tp.isTouched;
  TouchX = tp.points[0].x;
  TouchY = tp.points[0].y;
  
  //frisch berührt
  if (TouchContact and !Touch.TSTouched) {       
    Touch.TSTouched = millis();
    Touch.x0 = TouchX;
    Touch.y0 = TouchY;
    Touch.Gesture = 0;
    Touch.TSReleased = 0;
    ret = TOUCHED;
  }
  //Finger bleibt drauf
  else if (TouchContact and Touch.TSTouched) {   
    Touch.x0 = TouchX;
    Touch.y0 = TouchY;
    Touch.Gesture = 0;
    Touch.TSReleased = 0;
    ret = HOLD;
  }
  //Release
  else if (!TouchContact and Touch.TSTouched) {  
    Touch.TSReleased = millis();
    Touch.x1 = TouchX;
    Touch.y1 = TouchY;
          
         if ((Touch.x1-Touch.x0) > 50)  { Touch.Gesture = SWIPE_LEFT;  ret = SWIPE_LEFT; }                      // swipe left
    else if ((Touch.x1-Touch.x0) < -50) { Touch.Gesture = SWIPE_RIGHT; ret = SWIPE_RIGHT; }                     // swipe right
    else if ((Touch.y1-Touch.y0) > 50)  { Touch.Gesture = SWIPE_DOWN;  ret = SWIPE_DOWN; }                      // swipe down
    else if ((Touch.y1-Touch.y0) < -50) { Touch.Gesture = SWIPE_UP;    ret = SWIPE_UP; }                        // swipe up
    else if ((Touch.TSReleased - Touch.TSTouched) > LONG_PRESS_INTERVAL) {                                      // longPress
      Touch.Gesture = LONG_PRESS;
      ret = LONG_PRESS;     
    }  
    else { Touch.Gesture = CLICK; ret = CLICK; }
  }                                                                    
  //nicht berührt
  else if (!TouchContact and !Touch.TSTouched) {  
    ret = 0;
    Touch.TSTouched  = 0;
    Touch.TSReleased = 0;
  }
  return ret;
}
void  PrintMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}