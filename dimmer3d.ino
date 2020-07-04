#define PROJECT "D I M M E R   24.10.2018 mod 04.07.2020 GD"

#define WAVELEN 20000        // Wellenlänge in µs (50 Hz = 20000µs, 60Hz->16666)
#define HWLEN (WAVELEN/2)    // Halbwellenlänge (50 Hz = 10000µs)
#define TESTLED 5            // Zusätzliche Test-LED für DEBUG3
#define TESTPIN 14           // Testausgang für DEBUG3
#define TASTER 12            // TASTER zum Dimmen ESP-12 PIN 6
#define PIN_SYNC 13          // hier liegt Rechteckflanke an, 50Hz ESP-12 PIN 7
#define PIN_OPTOKOPPLER 16   // zum MOS-Fet, Länge des Pulses gleich Helligkeit 
#define DIM_MAX HWLEN        // maximaler Wert = 100%
#define MAXSWITCHTIMEVAL 100 // Anzahl zu merkender Switch-/Sync-Zeitpunkte
#define MINTIMER 20          // Mindesten 20 µs, sonst überholen wir uns selbst
#define TOUCHSPEED 100       // Geschwindigkeit (1/x ms), Änderung
#define TOUCHWAIT 400        // Wartezeit (ms) vor Änderung
#define SOFTONOFFSPEED 10    // ms, die zwischen zwei steps vergehen
#define SOFTONOFFSTEP 2      // %-Wert der inkr./dekr. 0-100 = 50*10ms = 1/2 Sek.
#define MAXTOPICSIZE 44      // max Länge Topic
// #define MQTTOVERRIDE         Wenn gesetzt, merke sich letzten Stand von MQTT

#define MAGIC_VAL 4711 // to reset settings change to any val 0..9999

#define DEFAULT_TOPIC "Dimmer_xxx"
#define HOSTNAME "Dimmer_xxx"
/*
 * secret.h definiert zwei MACROS:
 * SECRET enthält in "" ein Passwort
 * SECRET_LEN ist die Länge des Passwortes
 */
// #if __has_include("secret.h") // unfortunately disabled in Arduino
  #include "secret.h"
// #endif
#if defined(SECRET)
  #define RESET_SECRET "eset " SECRET
  #define RESET_SECRET_LEN (5 + sizeof(SECRET))
#else
  #define RESET_SECRET "eset"
  #define RESET_SECRET_LEN 4
#endif

#define DEBUG1
//#define DEBUG2
//#define DEBUG3
//#define DEBUG2POS1
//#define DEBUG2POS2
//#define DEBUG2POS3
#ifdef DEBUG2
  #ifndef DEBUG1
    #define DEBUG1
  #endif
#endif

#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager (0.15)
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <PubSubClient.h>         // MQTT Lib
#include <math.h>
#include <Ticker.h>

boolean ta_ruhe; 

//***********************************************************************************
// Parameterstruktur für EEPROM
struct param_struct {
  uint16_t magic,   // Magic numer to detect reset
           dim_min; // wann die Lampe gerade nocht leuchtet, mit mqtt veränderbar
           
  /*
   durch die Phasenverschiebung des Sync-Kondensators und die Laufzeit des Opto-
   koppler wird der Nulldurchgang später erkannt. Dieses wird durch die Konstante 
   dim_versatz ausgeglichen. Dies ist der Wert in µs um den die Nulldurchgangs-
   erkennung verzögert ist, kann mit mqtt und vorgestelltem 'v' verändert werden 
   (darf nicht kleiner sein als die Laufzeit des TimerISR, damit sich dieser nicht 
   mit dem SyncISR in die Quere kommt):
  */
  int16_t dim_versatz;
  
  // Anfangswert bei Tastenbedienung (0 = letzter eingestellte Wert)
  uint8_t anfangswert;
  
  char mqtt_server[40], mqtt_port[5], base_mqtt_topic[MAXTOPICSIZE];
  
  /*
   Wenn das Flag "Phasenabschnitt" false ist handelt es sich um eine 
   Phasenabschnittsteuerung, ansonsten um Phasenanschnitt.
  */
  boolean Phasenabschnitt;
} param;
//***********************************************************************************

short   softon = 0,  // Counter für langsames Abblenden
        softoff = 0; // Counter für langsames Aufblenden

#ifdef DEBUG2
boolean time_measure = false;    // Sammeln MAXSWITCHTIMEVAL Syncpunkte und Ausgabe
#endif

unsigned short preservedim;      // letzter eingestellter Wert

WiFiClient espClient;
PubSubClient client(espClient);

char status_mqtt_topic[MAXTOPICSIZE], 
     cmnd_mqtt_topic[MAXTOPICSIZE]; 

//***********************************************************************************
// Kontrollstrukturen u. -routinen für Dimmersteuerung
struct dim_ctl_struct {
  unsigned short ti_dimmer; //max. 10000 µs = 100Hz 50Hz nach Gleichrichter
  short dim_prozent; //0..100% entspricht ti_dimmer, <0 ist aus, 0 der niedr. Wert
  unsigned short dim_val[101];  // Übersetzungstabelle Prozent in Werte für ti_dimmer
  unsigned short ti_dim_min, ti_dim_max; // Grenzwerte für Dimmer
} dim_ctl;

//***********************************************************************************
void inline increase_dimmer(short delta = SOFTONOFFSTEP);
//***********************************************************************************
void inline decrease_dimmer(short delta = SOFTONOFFSTEP);
//***********************************************************************************
void inline set_dimmer_max() {
  dim_ctl.ti_dimmer = dim_ctl.dim_val[100];
  dim_ctl.dim_prozent = 100;
}
//***********************************************************************************
void inline set_dimmer_min() {
  dim_ctl.ti_dimmer = dim_ctl.dim_val[0];
  dim_ctl.dim_prozent = 0;
}
//***********************************************************************************
void inline set_dimmer_aus() {
  dim_ctl.ti_dimmer = dim_ctl.dim_val[0];
  dim_ctl.dim_prozent = -1;
}
//***********************************************************************************
void inline set_dimmer_by_prozent(short dimwert) {
  if (dimwert > 100) set_dimmer_max();
  else if (dimwert < 0) set_dimmer_aus();
  else {
    dim_ctl.ti_dimmer = dim_ctl.dim_val[dimwert];
    dim_ctl.dim_prozent = dimwert;
  }
}
//***********************************************************************************
boolean inline dim_on() { return (dim_ctl.dim_prozent >= 0); }
//***********************************************************************************
short inline get_dim() { return dim_ctl.dim_prozent; }
//***********************************************************************************
void inline increase_dimmer(short delta) {
  if (!dim_on()) set_dimmer_by_prozent(preservedim);
  else set_dimmer_by_prozent(dim_ctl.dim_prozent + delta);
}
//***********************************************************************************
void inline decrease_dimmer(short delta) {
  if (!dim_on()) set_dimmer_by_prozent(preservedim);
  else set_dimmer_by_prozent(dim_ctl.dim_prozent - delta);
}
//***********************************************************************************
void calc_dimmer_tab() {
  /*
    Diese Bestimmung des Schaltpunktes zum Dimmen basiert auf dem Integral unter 
    der Sinus-Kurve und gibt für jeden Prozentwertpunkt ungefähr die gleiche 
    Energiemenge frei - unter Berücksichtigung der Blindleistung, die mit dim_min
    absolut festgelegt wird.
  */
  for (short i = 0; i <= 99; i++) 
    dim_ctl.dim_val[i] = int(acos(1.0 - i * (2.0 - (param.dim_min / (HWLEN / 2.0))) / 
                 100.0 - (param.dim_min / (HWLEN/2.0))) * DIM_MAX / PI);
  dim_ctl.dim_val[100] = DIM_MAX;
  dim_ctl.ti_dim_max = dim_ctl.dim_val[99]; // 99%
  dim_ctl.ti_dim_min = dim_ctl.dim_val[0]; // 0%
}
//***********************************************************************************
void EEPROMWrite(uint8_t *zeichen, int laenge, int pos = 0)
{
  for (int i = 0; i < laenge; i++)
  {
    EEPROM.write(pos+i, *zeichen++);
  }
}
//***********************************************************************************
void EEPROMRead(uint8_t *zeichen, int laenge, int pos = 0)
{
  for (int i = 0; i < laenge; i++)
  {
    *zeichen++ = EEPROM.read(pos+i);
  }
}
//***********************************************************************************
void setup()
{
  wl_status_t res = WL_DISCONNECTED;
 
  #ifdef DEBUG1
    Serial.begin(115200);
    Serial.printf(PROJECT " compiled %s %s (%s)\n",__DATE__,__TIME__,__FILE__);
  #endif

  WiFi.mode(WIFI_STA);
  
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);
  pinMode(TASTER, INPUT_PULLUP);
  pinMode(PIN_SYNC, INPUT_PULLUP);
  pinMode(PIN_OPTOKOPPLER, OUTPUT);
  digitalWrite(PIN_OPTOKOPPLER, LOW);
  #ifdef DEBUG3
    pinMode(TESTLED, OUTPUT);
    digitalWrite(LESTLED, LOW);
    pinMode(TESTPIN, OUTPUT);
    digitalWrite(TESTPIN, LOW);
  #endif

  EEPROM.begin(sizeof(param));

  #ifdef DEBUG1
    Serial.println(F("EEPROM-Werte lsesen..."));
  #endif

  EEPROMRead(reinterpret_cast<uint8_t*>(&param), sizeof(param));

  #ifdef DEBUG1
    Serial.println(F("Status-Values for WiFi are:"));
    Serial.println(F("WL_IDLE_STATUS      = 0,"));
    Serial.println(F("WL_NO_SSID_AVAIL    = 1,"));
    Serial.println(F("WL_SCAN_COMPLETED   = 2,"));
    Serial.println(F("WL_CONNECTED        = 3,"));
    Serial.println(F("WL_CONNECT_FAILED   = 4,"));
    Serial.println(F("WL_CONNECTION_LOST  = 5,"));
    Serial.println(F("WL_DISCONNECTED     = 6"));
  #endif

  boolean reset_detected = (param.magic != MAGIC_VAL);
  if (reset_detected) {
    #ifdef DEBUG1
       Serial.println(F("Reset: EEPROM default Werte schreiben . . ."));
    #endif
    param.magic = MAGIC_VAL;
    param.dim_min = 500; // Entspricht 1600 nach alter Logik     
    param.anfangswert = 0;
    param.dim_versatz = 2840; 
    param.Phasenabschnitt = true;
    param.mqtt_server[0] = '\0';
    strlcpy(param.mqtt_port,"1883",4);
    strlcpy(param.base_mqtt_topic,DEFAULT_TOPIC,sizeof(DEFAULT_TOPIC));
    EEPROMWrite(reinterpret_cast<uint8_t*>(&param),sizeof(param));    
    EEPROM.commit();

    /*
       Signalling reset:
    */
    delay(333);    
    digitalWrite(BUILTIN_LED, LOW);
    delay(333);    
    digitalWrite(BUILTIN_LED, HIGH);
    delay(333);    
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(PIN_OPTOKOPPLER, HIGH);
    delay(333);    
    digitalWrite(BUILTIN_LED, HIGH);
    delay(333);    
    digitalWrite(BUILTIN_LED, LOW);
    delay(333);    
    digitalWrite(BUILTIN_LED, HIGH);
    digitalWrite(PIN_OPTOKOPPLER, LOW);
  } else if (WiFi.SSID() != "") {
    for (int i = 0; ((i < 10) && (res != WL_CONNECTED)); i++) {
      res = WiFi.begin();
      for (int i = 0; ((i < 100) && (res == WL_DISCONNECTED)); i++) { 
        delay(100); 
        res = WiFi.status(); 
      } 
      #ifdef DEBUG1
        Serial.print(F("Try to connect. Result after max 10s = "));
        Serial.println(res);
      #endif
      if (res == WL_NO_SSID_AVAIL) delay(10000);
    }
  }
  
  if ((res != WL_CONNECTED) || reset_detected) {
    String mac1 = WiFi.macAddress();

    // WiFiManager
    WiFiManagerParameter 
	  custom_mqtt_server("server", "mqtt server", param.mqtt_server,40),
	  custom_mqtt_port("port", "mqtt port", param.mqtt_port, 5),
    custom_mqtt_topic("topic", "sub_mqtt topic", param.base_mqtt_topic, 
                      MAXTOPICSIZE);

    WiFiManager wifiManager;
  
    if (reset_detected) {
      // reset settings
      wifiManager.resetSettings();
    }

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_topic);
    wifiManager.setConfigPortalTimeout(180); 
    wifiManager.setConnectTimeout(100); 
    #ifndef DEBUG1
      wifiManager.setDebugOutput(false);
    #endif
    /* 
      sets timeout until configuration portal gets turned off, useful to make it all
      retry or go fetches ssid and pass and tries to connect,if it does not connect
      it starts an access point with the specified name to sleep in seconds, here  
      "AutoConnectAP" and goes into a blocking loop awaiting configuration
    */
    char AP_SSID[19] = "AutoConnectAP_";

    AP_SSID[14] = mac1[12];
    AP_SSID[15] = mac1[13];
    AP_SSID[16] = mac1[15];
    AP_SSID[17] = mac1[16];
    AP_SSID[18] = '\0';

    // qwertzui ist das Passwort zum AP
    if (!wifiManager.startConfigPortal(AP_SSID, "qwertzui"))  {
      delay(3000);
      // reset and try again
      ESP.reset();                     
    } else res = WiFi.status();

    strlcpy(param.mqtt_server, custom_mqtt_server.getValue(),40);
    strlcpy(param.mqtt_port, custom_mqtt_port.getValue(),5);
    strlcpy(param.base_mqtt_topic, custom_mqtt_topic.getValue(),MAXTOPICSIZE);

    if (param.mqtt_server[0] != '\0') {
      #ifdef DEBUG1
        Serial.println(F("MQTT-Werte ins EEPROM schreiben . . ."));
      #endif

      EEPROMWrite(reinterpret_cast<uint8_t*>(&param),sizeof(param));    
      EEPROM.commit();
    }
  }
  
  if (res != WL_CONNECTED) {
    delay(3000);
    // reset and try again
    ESP.reset();                     
  }    

  // if you get here you have connected to the WiFi
  #ifdef DEBUG1
     Serial.print(F("IP Address: "));
     Serial.println(WiFi.localIP());
  #endif

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME);

  // No authentication by default
  #ifdef SECRET
    ArduinoOTA.setPassword(SECRET);
  #endif

  #ifdef DEBUG1
    ArduinoOTA.onStart([]() {
      if (ArduinoOTA.getCommand() == U_FLASH)
        Serial.println(F("Start updating Flash"));
      else
        Serial.println(F("FS not supported"));
    });
  
    ArduinoOTA.onEnd([]() {
      Serial.println(F("\nEnd"));
    });
  
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
  
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println(F("Auth Failed"));
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println(F("Begin Failed"));
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println(F("Connect Failed"));
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println(F("Receive Failed"));
      } else if (error == OTA_END_ERROR) {
        Serial.println(F("End Failed"));
      }
    });
  #endif
  
  ArduinoOTA.begin();

  // Nur für's erste Einschalten:
  preservedim = ((param.anfangswert > 0)?param.anfangswert:100); 

  #ifdef DEBUG1
    Serial.printf("anfangswert = %d, dim_versatz = %d, dim_min = %d, %s\n",
	  param.anfangswert,param.dim_versatz,param.dim_min,
	  (param.Phasenabschnitt?"Phasenabschnitt":"Phasenanschnitt"));
  #endif

  calc_dimmer_tab();

  short port = atoi(param.mqtt_port);

  strlcpy(status_mqtt_topic, "stat/", MAXTOPICSIZE); 
  strlcat(status_mqtt_topic, param.base_mqtt_topic, MAXTOPICSIZE);
  strlcpy(cmnd_mqtt_topic, "cmnd/", MAXTOPICSIZE); 
  strlcat(cmnd_mqtt_topic, param.base_mqtt_topic, MAXTOPICSIZE);

  #ifdef DEBUG1
    Serial.printf("MQTT-Server: %s:%d\n", param.mqtt_server, port);
    Serial.print(F("MQTT-Topic Base: "));
    Serial.println(param.base_mqtt_topic);
    Serial.print(F("MQTT-Topic Status: "));
    Serial.println(status_mqtt_topic);
    Serial.print(F("MQTT-Topic Command: "));
    Serial.println(cmnd_mqtt_topic);
  #endif

  client.setServer(param.mqtt_server, port);
  client.setCallback(callback);         // MQTT Eingangsroutine festlegen

  // TASTER/Touch mit Ruhe oder Arbeitskontakt, bzw. bei Touch GND im Ruhezustand
  ta_ruhe = digitalRead(TASTER);      

  delay_OTA(10);
  set_dimmer_aus();
  calcswitchtime();
  
  attachInterrupt(digitalPinToInterrupt(PIN_SYNC),SyncISR,CHANGE);
  timer1_attachInterrupt(TimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
}
//***********************************************************************************
void delay_OTA(unsigned long msec)
{
  if (msec == 0)
  {
    ArduinoOTA.handle();
    yield();    
    return;
  }

  unsigned long stopm = millis() + msec;
  while (stopm > millis())
  {
    ArduinoOTA.handle();
    yield();    
  }
}
//***********************************************************************************
void loop()
{
  static unsigned long lastsoftchg = 0;
  
  // TASTER zum Dimmen wird abgefragt
  tasterabfrage();

  // prüfe alle SOFTONOFFSPEED ms
  if (millis() - lastsoftchg > SOFTONOFFSPEED) {
    boolean dimchg = false;
    
    if (dim_on()) {
      if (softoff > 0) {
        decrease_dimmer();
        dimchg = true;
        if (--softoff == 0) set_dimmer_aus();
      }
      if (softon > 0) {
        increase_dimmer();
        dimchg = true;
        softon--;
      }
    }
    if (dimchg) {
      calcswitchtime();
      if ((softon == 0) && (softoff == 0)) mqtt_publish();
    }
    lastsoftchg = millis();;
  }
  
  client_connected_unblocking();
  
  delay_OTA(0);
}
//***********************************************************************************
void client_connected_unblocking()
{
  static unsigned long last_reconnect = 0;
  #ifdef DEBUG1
    static unsigned long last_alive = 0;
  #endif
  unsigned long now = millis();

  if (!client.connected()) {
    if (now - last_reconnect > 50000) {
      ESP.reset();
    } else if (now - last_reconnect > 5000) {
      if (reconnect()) last_reconnect = now;
    }
  } else {
    last_reconnect = now; // to avoid overrun
    #ifdef DEBUG1
      if (now - last_alive > 10000) {
        Serial.println(F("Connection alive!"));
        last_alive = now;
      } 
    #endif
    client.loop();
  }
}
//***********************************************************************************
boolean reconnect()
{
  if (client.connect(HOSTNAME)) {
    #ifdef DEBUG1
      Serial.println(F("Reconnect succesfull!"));
    #endif
    client.subscribe(cmnd_mqtt_topic); // ... and resubscribe
  } else {
    #ifdef DEBUG1
      Serial.println(F("Reconnect failed!"));
    #endif
  }
  return client.connected();
}
//***********************************************************************************
void callback(char *topic, byte *payload, unsigned short length)     
// Subscribe (Empfange) MQTT Nachricht
{
  #ifdef DEBUG1
    Serial.printf("%d: ", length);
    for (short i = 0; i < length; i++) Serial.print((char)payload[i]);
    Serial.println();
  #endif
  
  switch (payload[0]) {
  #ifdef DEBUG2
    case 't': // toggle Zeitmessung
      if ((length == 3) && (payload[1] == 'i') && (payload[2] == 'c')) {
        time_measure = !time_measure;
        if (!time_measure) push_switchtime(0,0);
        Serial.print(F("Zeitmessung "));
        Serial.println(time_measure?"an":"aus");
      } 
      break;
  #endif
  case 'r':
    if (strncmp((const char *)(payload+1), RESET_SECRET, RESET_SECRET_LEN) != 0)
      break;
    param.magic = 0; // lösche magic cookie
    EEPROMWrite(reinterpret_cast<uint8_t*>(&param.magic),sizeof(param.magic),
              (int)offsetof(param_struct,magic));
    EEPROM.commit();

    delay_OTA(3000);
    ESP.reset();
  case 'v':                     
    // Wert in µs in dem das SYNC-Signal verzögert ist, wird für exakten 
    // Phasen-An-oder Abschnitt benötigt und im EEPROM gespeichert
    if (payload[1] == '+') param.dim_versatz += 100;
    else if (payload[1] == '-') param.dim_versatz -= 100;
    else param.dim_versatz = atoi((const char *)(payload+1));
    if (param.dim_versatz < -HWLEN) param.dim_versatz = -HWLEN;
    if (param.dim_versatz > HWLEN) param.dim_versatz = HWLEN;

    EEPROMWrite(reinterpret_cast<uint8_t*>(&param.dim_versatz),
              sizeof(param.dim_versatz),
        (int)offsetof(param_struct,dim_versatz));
    EEPROM.commit();

    mqtt_status_publish("dim_versatz", param.dim_versatz);
    break;
  case 'n':                     
    // aNschnitt: Dimmer soll im Phasen Anschnittmodus arbeiten
    // 'n' für Phasenanschnitt in EEPROM schreiben an Adresse 91
    param.Phasenabschnitt = false;
    EEPROMWrite(reinterpret_cast<uint8_t*>(&param.Phasenabschnitt),
              sizeof(param.Phasenabschnitt),
        (int)offsetof(param_struct,Phasenabschnitt));
    EEPROM.commit();
    mqtt_status_publish_txt("Phasenabschnitt","false");
    break;
  case 'b':                     
    // aBschnitt: Dimmer soll im Phasen Abschnittmodus arbeiten
    // 'b' für Phasenanschnitt in EEPROM schreiben an Adresse 91
    param.Phasenabschnitt = true;
    EEPROMWrite(reinterpret_cast<uint8_t*>(&param.Phasenabschnitt),
              sizeof(param.Phasenabschnitt),
        (int)offsetof(param_struct,Phasenabschnitt));
    EEPROM.commit();
    mqtt_status_publish_txt("Phasenabschnitt","true");
    break;
  case 'm': 
    // Minimum: Wert bei dem die zu steuernde Lampe gerade noch leuchtet
    param.dim_min = atoi((const char*)(payload+1));
  if (param.dim_min < 0) param.dim_min = 0;
  if (param.dim_min > HWLEN) param.dim_min = HWLEN; 
    calc_dimmer_tab();

    EEPROMWrite(reinterpret_cast<uint8_t*>(&param.dim_min),
              sizeof(param.dim_min),(int)offsetof(param_struct,dim_min));
    EEPROM.commit();

    mqtt_status_publish("dim_min",param.dim_min);
    break;
  case 'a': 
    // Anfangswer: Wert, der bei Handbetätigung genommen wird
    param.anfangswert = atoi((const char*)(payload+1));
    if (param.anfangswert > 100) param.anfangswert = 0;

    EEPROMWrite(reinterpret_cast<uint8_t*>(&param.anfangswert),
              sizeof(param.anfangswert),
        (int)offsetof(param_struct,anfangswert));
    EEPROM.commit();

    mqtt_status_publish("anfangswert",param.anfangswert);
    break;
  case 's':
    // Status auf MQTT ausgeben
    mqtt_status_publish_txt("File",__FILE__);
    mqtt_status_publish_txt("Date",__DATE__);
    mqtt_status_publish_txt("Time",__TIME__);
    mqtt_status_publish_txt("Phasenabschnitt",
                          (param.Phasenabschnitt?"true":"false"));
    mqtt_status_publish("dim_versatz",param.dim_versatz);
    mqtt_status_publish("dim_min",param.dim_min);
    mqtt_status_publish("dim_prozent",dim_ctl.dim_prozent);
    mqtt_status_publish("ti_dimmer",dim_ctl.ti_dimmer);
    mqtt_status_publish("anfangswert",param.anfangswert);
    break;
  case '+':
  case '-':
    if (!dim_on()) set_dimmer_by_prozent(preservedim);    
    else {
      payload[(length < 3)?length:3] = '\0';
      short mqtt_delta = atoi((const char *)payload);
      if (mqtt_delta == 0) mqtt_delta = ((*payload == '-')?-5:5);
    
      #ifdef MQTTOVERRIDE
        if ((get_dim()+mqtt_delta < 0) && dim_on()) {
          preservedim = ((param.anfangswert > 0)?
                         param.anfangswert:get_dim());
        }
      #endif
      increase_dimmer(mqtt_delta);    
    }
    #ifdef DEBUG1
      Serial.printf("Dimwert: %d = %d%\n",dim_ctl.dim_prozent,dim_ctl.ti_dimmer);
    #endif
    mqtt_publish();
    break;
  case 'o': // toggle Zeitmessung
    if ((length == 3) && (payload[1] == 'f') && (payload[2] == 'f')) {
      #ifdef MQTTOVERRIDE
        preservedim = ((param.anfangswert > 0)?
                       param.anfangswert:get_dim());
      #endif

      set_dimmer_aus();
    }
    #ifdef DEBUG1
      Serial.printf("Dimwert: %d = %d%\n",dim_ctl.dim_prozent,dim_ctl.ti_dimmer);
    #endif
    mqtt_publish();
    break;
  default: //Dimmer wird mit Zahl zwischen 0 und 100 eingestellt
    payload[(length < 3)?length:3] = '\0';
    short mqtt_dimwert = atoi((const char *)payload);
    
    set_dimmer_by_prozent(mqtt_dimwert);
    #ifdef DEBUG1
      Serial.printf("Dimwert: %d = %d%\n",mqtt_dimwert,dim_ctl.ti_dimmer);
    #endif
    mqtt_publish();
    break;
  }
  calcswitchtime();
}
//***********************************************************************************
void mqtt_status_publish(const char *status_subtopic, long statuszahl)
{
  char buff[15];
  ltoa(statuszahl, buff, 10);
  mqtt_status_publish_txt(status_subtopic, buff);
}
//***********************************************************************************
void mqtt_status_publish_txt(const char *status_subtopic, const char *value)
{
  char topic[MAXTOPICSIZE];

  strlcpy(topic, status_mqtt_topic, MAXTOPICSIZE); 
  strlcat(topic, "/", MAXTOPICSIZE);
  strlcat(topic, status_subtopic, MAXTOPICSIZE);

  client.publish(topic, value);
  #ifdef DEBUG1
    Serial.printf("Publish %s:%s\n", topic, value);
  #endif
}
//***********************************************************************************
void mqtt_publish()
{
  char dim_als_char[7];

  itoa(get_dim(), dim_als_char, 10);
  
  mqtt_status_publish_txt("Dimmer", dim_als_char); //PUBLIZIEREN
  #ifdef DEBUG1
    if ((dim_ctl.ti_dimmer < param.dim_min) || (dim_ctl.ti_dimmer > DIM_MAX))
      Serial.println(F("Dimmer out of range!"));
  #endif
}
//***********************************************************************************
void tasterabfrage()
{
  static boolean ta_war_gedr = false,
                 dim_richtung_auf = false,
                 am_dimmen = false;
  static short on_off_or_dim = TOUCHWAIT;
  static unsigned long ti_taster = 0;
  boolean ta_ist_gedr = (digitalRead(TASTER) != ta_ruhe);

  if (ta_ist_gedr) { 
    if (!ta_war_gedr) { // TASTER/Touch neu gedrückt
      ta_war_gedr = true;
      ti_taster = millis();
    } else {            // TASTER/Touch immer noch gedrückt
      unsigned long ti_taster1 = millis();
      if (ti_taster1 - ti_taster > on_off_or_dim) { //bei >400ms dimmen
        if (dim_richtung_auf) {
          increase_dimmer();
          on_off_or_dim = TOUCHSPEED;
        } else {
          decrease_dimmer();
          if (!dim_on()) {
            dim_richtung_auf = true;
            set_dimmer_min();
            on_off_or_dim = TOUCHWAIT;
          } else on_off_or_dim = TOUCHSPEED;
        }
        calcswitchtime();
        ti_taster = ti_taster1;
        am_dimmen = true;
      }
    }
  } else {
    if (ta_war_gedr) { // TASTER/Touch los gelassen
      if ((millis() - ti_taster > 10) && !am_dimmen) {     // entprellen
        if (dim_on()) {
          preservedim = ((param.anfangswert > 0)?
                         param.anfangswert:dim_ctl.dim_prozent);
          softoff = preservedim / SOFTONOFFSTEP + 1;   // Starte SoftOff
        } else {
          softon = preservedim / SOFTONOFFSTEP;
          set_dimmer_by_prozent(0); // niedrigster Wert
          calcswitchtime();
        }
      }
      ta_war_gedr = false;
      if (am_dimmen) {
        on_off_or_dim = TOUCHWAIT;
        dim_richtung_auf = !dim_richtung_auf;
        am_dimmen = false;
      }
      mqtt_publish();
    }
  }
}
//***********************************************************************************
// Interruptbehandlung
volatile unsigned long ti_null; // korrigierter letzter Nulldurchgang für sync
volatile unsigned short Next_low, Next_high; // nächste Schaltzeitpunkte darauf
//***********************************************************************************
void calcswitchtime()
/*
   Zum Dimmen werden zwei Zeitpunkte (abhängig von Phasenan-/-abschnitt
   ermittelt: Next_low und Next_high sind der jeweils nächste, zu vollziehende
   Schaltzeitpunkt; 

   Phasenanschnitt:
   |     ---         ---         ---
   |    /   \       /###\       /   \
   |   /     \     /|####\     /     \
   |  /       \   / |#####\   /       \
   | /         \ /  |######\ /         \
   -*-----------*-|---------*-----------*-
                ^ | ^       ^
          ti_null | |       Next_low = 10000
                  | |
                  | Next_high = 10000 - ti_dimmer
                  |
                  aktuelle Phase = LOW

   |     ---         ---         ---
   |    /   \       /###\       /###\
   |   /     \     /|####\     /|####\
   |  /       \   / |#####\   / |#####\
   | /         \ /  |######\ /  |######\
   -*-----------*-------|---*-----------*-
                ^       |   ^   ^
          ti_null       |   |   Next_high = 10000 - ti_dimmer
                        |   |
                        |   Next_low = 10000
                        |
                        aktuelle Phase = HIGH

   Phasenabschnitt:
   |     ---         ---         ---
   |    /   \       /###\       /###\
   |   /     \     /####|\     /####|\
   |  /       \   /#####| \   /#####| \
   | /         \ /######|  \ /######|  \
   -*-----------*-----------*-----------*-
                ^ |     ^   ^
          ti_null |     |   Next_high = 10000
                  |     |
                  |     Next_low = ti_dimmer
                  |
                  aktuelle Phase = HIGH

   |     ---         ---         ---
   |    /###\       /###\       /   \
   |   /####|\     /####|\     /     \
   |  /#####| \   /#####| \   /       \
   | /######|  \ /######|  \ /         \
   -*-----------*-----------*-----------*-
    ^         | ^       ^
    ti_null   | |       Next_low = ti_dimmer
              | |
              | Next_high = 10000
              |
              aktuelle Phase = LOW
*/
{
  if (param.Phasenabschnitt) {
    Next_low = dim_ctl.ti_dimmer;
    Next_high = HWLEN;
  } else {
    Next_low = HWLEN;
    Next_high = HWLEN-dim_ctl.ti_dimmer;
  }
  #ifdef DEBUG1
    Serial.printf("Next_low = %d, Next_high = %d\n",Next_low,Next_high);
  #endif
}
//***********************************************************************************
ICACHE_RAM_ATTR void SyncISR()
/*
  Synchronisations-Interrupt, wird vom fallenden Sync-Signal getriggert.
  Wir sind hier schon um dim_versatz über die Zeit - muss berücksichtigt werden.
  Akzeptiert wird der Trigger nur, wenn entweder der letzte Trigger min. 98%
  einer Vollwelle her ist und seit der letzten fallenden Flanke zwischen 98% und 102%
  der Zeit einer Welle vergangen ist.
*/

#define MINSTEP (WAVELEN * 49UL / 50)
#define MAXSTEP (WAVELEN * 51UL / 50)

{ 
  unsigned long now = micros()-param.dim_versatz;
  static unsigned long last_Edge = 0;
  static boolean lastLow = false;
  boolean lowLevel = (digitalRead(PIN_SYNC) == LOW);

  if (lastLow || !lowLevel) {
    lastLow = lowLevel;
    return;
  }
  lastLow = true;
  unsigned long delta = now - last_Edge;
  last_Edge = now;

  if (now - ti_null < MINSTEP) {
    #ifdef DEBUG2
      #ifdef DEBUG2POS1
        push_switchtime(10,now - ti_null);
      #endif
    #endif
  } else if ((delta < MINSTEP) || (delta > MAXSTEP)) {
    #ifdef DEBUG2
      #ifdef DEBUG2POS1
        push_switchtime(11,delta);
      #endif
    #endif
  } else {
    #ifdef DEBUG2
      #ifdef DEBUG2POS1
        push_switchtime(12,delta);
      #endif
    #endif

    ti_null = now;

    #ifdef DEBUG3
      static boolean sgn = LOW;
      digitalWrite(TESTPIN,sgn = !sgn);
    #endif
    
    if (dim_on()) {

      long wait_next;
      
      if ((param.Phasenabschnitt && (param.dim_versatz < Next_low)) || 
          (!param.Phasenabschnitt && (param.dim_versatz >= Next_high)))
      {
        pwm(HIGH);
        wait_next = Next_low-param.dim_versatz;
      } else {
        pwm(LOW);
        wait_next = Next_high-param.dim_versatz;
      }
       if (wait_next < MINTIMER) wait_next = MINTIMER; // Sicherheitshalber
       timer1_write(5UL*wait_next);
      #ifdef DEBUG2
        #ifdef DEBUG2POS2
          push_switchtime(2,wait_next);
        #endif
      #endif
    } else pwm(LOW);
  }
}
//***********************************************************************************
ICACHE_RAM_ATTR void TimerISR()
/*
   Timer-Interrupt - schaltet in den jeweila gültigen Zustand (HIGH/LOW) und
   ermittelt Wartezeit bis zum nächsten Schaltvorgang.
*/
{
  unsigned long now = micros();
  unsigned long delta = now - ti_null; // Abstand jetzt-Nulldurchgang (nur pos)
  unsigned short sdelta = delta % HWLEN; // Abstand jetzt-Nulldurchgang (pos/neg)
  long wait_next;
  
  #ifdef DEBUG3
    static boolean sgn = LOW;
    digitalWrite(TESTLED,sgn = !sgn);
  #endif

  if ((!param.Phasenabschnitt && sdelta > Next_high) || // Anschnitt
      (param.Phasenabschnitt && sdelta < Next_low)) {  // Abschnitt
    pwm(HIGH);
    wait_next = Next_low-sdelta;
  } else {
    pwm(LOW);
    wait_next = Next_high-sdelta;
  }

  if (wait_next < MINTIMER) wait_next = MINTIMER; // Sicherheitshalber

  timer1_write(5UL*wait_next);
  #ifdef DEBUG2
    #ifdef DEBUG2POS3
      push_switchtime(3,wait_next);
    #endif
  #endif
}
//***********************************************************************************
ICACHE_RAM_ATTR void pwm(boolean wert)   
// hier wird der Port zum MOS-FET geschaltet 
{
  static boolean fl_pwm = HIGH;

  if (!dim_on() || (dim_ctl.ti_dimmer < dim_ctl.ti_dim_min)) wert = LOW; // < Minimum -> aus
  else if (dim_ctl.ti_dimmer > dim_ctl.ti_dim_max) wert = HIGH; // Oberhalb Max. --> volle Helligk.
  
  if (wert == HIGH) {
    if (fl_pwm == LOW) {
      fl_pwm = HIGH; // Schalte Mosfet an
      digitalWrite(PIN_OPTOKOPPLER, HIGH);
    }
  } else {
    if (fl_pwm == HIGH) {
      fl_pwm = LOW; // Schalte Mosfet aus
      digitalWrite(PIN_OPTOKOPPLER, LOW);
    }
  }
}
//***********************************************************************************
#ifdef DEBUG2
  ICACHE_RAM_ATTR void push_switchtime(short pos,unsigned long value) 
  {
    static unsigned short ticpos[MAXSWITCHTIMEVAL];
    static unsigned long tictime[MAXSWITCHTIMEVAL];
    static unsigned long ticvalue[MAXSWITCHTIMEVAL];
    static unsigned long ticnull[MAXSWITCHTIMEVAL];
    static unsigned short ticinx = 0;
    static boolean started = false,
                   full = false;
  
    if (time_measure) {
      started = true;
      ticpos[ticinx] = pos;
      ticnull[ticinx] = ti_null;
      tictime[ticinx] = micros();
      ticvalue[ticinx++] = value;
      if (ticinx >= MAXSWITCHTIMEVAL) {
        ticinx = 0;
        full = true;
      }
    } else if (started) {
      unsigned short i;
      started = false;
      if (full) {
        i = ticinx;
        full = false;
      }
      else i = 0;
      do {
        Serial.printf("Pos %d: Time: %lu - Delta %lu Null %lu\n",
                      ticpos[i],tictime[i],ticvalue[i],ticnull[i]);
        i++;
        i %= MAXSWITCHTIMEVAL;
        yield();
      } while (i != ticinx);  
      ticinx = 0;
    }
  }
#endif
//***********************************************************************************