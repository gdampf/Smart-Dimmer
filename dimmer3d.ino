/*
  D I M M E R   24.10.2018 mod 01.06.2020 GD
*/
#define WAVELEN 20000        // Wellenlänge in µs (50 Hz = 20000µs, 60Hz->16666)
#define HWLEN (WAVELEN/2)    // Halbwellenlänge (50 Hz = 10000µs)
#define TASTER 12            // TASTER zum Dimmen ESP-12 PIN 6
#define PIN_SYNC 13          // hier liegt Rechteckflanke an, 50Hz ESP-12 PIN 7
#define PIN_OPTOKOPPLER 16   // zum MOS-Fet, Länge des Pulses gleich Helligkeit 
#define DIM_MAX HWLEN        // maximaler Wert = 100%
#define MAXSWITCHTIMEVAL 100 // Anzahl zu merkender Switch-/Sync-Zeitpunkte
#define MINTIMER 20          // Mindesten 20 µs, sonst überholen wir uns selbst
#define TOUCHSPEED 100       // Geschwindigkeit (1/x ms), Änderung
#define TOUCHWAIT 400        // Wartezeit (ms) vor Änderung
#define MAXTOPICSIZE 44
#define OFFSET_DIM_VERSATZ MAXTOPICSIZE + 45
#define OFFSET_PHASENABSCHNITT OFFSET_DIM_VERSATZ + 2
#define OFFSET_DIM_MIN OFFSET_PHASENABSCHNITT + 1
#define OFFSET_MAGIC OFFSET_DIM_MIN + 2
#define MAGIC_VAL 4711 // to reset settings change to any val 0..9999

#define DEFAULT_TOPIC "MQTT TOPIC,Bsp Wozi/ESP_7EB3_Dimmer"
#define RESET_SECRET "eset Secret123"
#define RESET_SECRET_LEN 14

//#define DEBUG1
//#define DEBUG2
//#define DEBUG3
//#define DEBUG2POS1
// #define DEBUG2POS2
// #define DEBUG2POS3
#ifdef DEBUG2
  #ifndef DEBUG1
    #define DEBUG1
  #endif
#endif

/*
  durch die Phasenverschiebung des Sync-Kondensators und die Laufzeit des 
  Optokoppler wird der Nulldurchgang später erkannt. Dieses wird durch die 
  Konstante dim_versatz ausgeglichen. Wenn das Flag "Phasenabschnitt" false
  ist handelt es sich um eine Phasenabschnittsteuerung, ansonsten um Phasen 
  Anschnitt.
*/

#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino

#include <DNSServer.h>

#include <ESP8266WebServer.h>

#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager

#include <EEPROM.h>

#include <PubSubClient.h>         // MQTT Lib

#include <math.h>

#include <Ticker.h>

boolean ta_ruhe, 
        dim_on = false, 
        Phasenabschnitt = false;

#ifdef DEBUG2
boolean time_measure = false;    // Sammeln 1000 Syncpunkte und Ausgabe
#endif

/* wert das die Lampe gerade nocht leuchtet, kann mit mqtt verändert werden */
unsigned short dim_min = 618; // Entspricht 1600 nach alter Logik     

unsigned short ti_dimmer = 0; //max. 10000 µs = 100Hz 50Hz nach Gleichrichter
unsigned short dim_prozent = 0; //0..100% entspricht ti_dimmer
unsigned short dim_val[101]; //Übersetzungstabelle Prozent in Werte für ti_dimmer
unsigned short ti_dim_min, ti_dim_max; // Grenzwerte für Dimmer

/* 
 Wert in µs um den die Nulldurchgangserkennung verzögert ist, kann mit mqtt und 
 vorgestelltem 'v' verändert werden (darf nicht kleiner sein als die Laufzeit des 
 TimerISR, damit sich dieser nicht mit dem SyncISR in die Quere kommt):
*/
short dim_versatz = 3200; 

WiFiClient espClient;
PubSubClient client(espClient);

char status_mqtt_topic[MAXTOPICSIZE], 
     cmnd_mqtt_topic[MAXTOPICSIZE]; 

volatile unsigned long ti_null;
volatile unsigned short Next_low, Next_high;

//*********************************************************************************
void setup()
{
  static char mqtt_server[40],
              mqtt_port[5] = "1883";
  char        base_mqtt_topic[MAXTOPICSIZE] = DEFAULT_TOPIC;

  #ifdef DEBUG1
    Serial.begin(115200);
    Serial.printf(" D I M M E R  24.10.2018 mod 01.06.2020 GD compiled %s %s (%s)\n",__DATE__,__TIME__,__FILE__);
  #endif

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);
  pinMode(TASTER, INPUT_PULLUP);
  pinMode(PIN_SYNC, INPUT_PULLUP);
  pinMode(PIN_OPTOKOPPLER, OUTPUT);
  digitalWrite(PIN_OPTOKOPPLER, LOW);
  #ifdef DEBUG3
    pinMode(5, OUTPUT);
    digitalWrite(5, LOW);
    pinMode(14, OUTPUT);
    digitalWrite(14, LOW);
  #endif

  String mac1 = WiFi.macAddress();

  EEPROM.begin(512);

  // WiFiManager
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server,40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_mqtt_topic("topic", "sub_mqtt topic", 
                         base_mqtt_topic, MAXTOPICSIZE);

  WiFiManager wifiManager;
  
  unsigned short magic = EEPROM.read(OFFSET_MAGIC) * 100 + 
                       EEPROM.read(OFFSET_MAGIC+1);

  if (magic != MAGIC_VAL) {
    // reset settings
    wifiManager.resetSettings();
    #ifdef DEBUG1
       Serial.println("Reset: EEPROM default Werte schreiben . . .");
    #endif
    EEPROM.write(OFFSET_DIM_VERSATZ, ((uint16_t)dim_versatz >> 8));    
    EEPROM.write(OFFSET_DIM_VERSATZ+1, ((uint16_t)dim_versatz & 0xFF));    
    EEPROM.write(OFFSET_PHASENABSCHNITT, Phasenabschnitt?'b':'n');
    EEPROM.write(OFFSET_DIM_MIN, dim_min / 100);
    EEPROM.write(OFFSET_DIM_MIN + 1, dim_min % 100);
    EEPROM.write(OFFSET_MAGIC, MAGIC_VAL / 100);
    EEPROM.write(OFFSET_MAGIC + 1, MAGIC_VAL % 100);
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
  } else {
    #ifdef DEBUG1
       Serial.println("EEPROM Werte lesen . . .");
    #endif
    // versatz des Sync-Signals zum Nulldurchgang in µs
    dim_versatz = (short)(((uint16_t) EEPROM.read(OFFSET_DIM_VERSATZ) << 8) |  
                        (uint16_t) EEPROM.read(OFFSET_DIM_VERSATZ+1));
    Phasenabschnitt = (EEPROM.read(OFFSET_PHASENABSCHNITT) == 'b');
    // Phasenabschnitt = true, sonst false
    
    // minimalen Wert für ti_dimmer holen
    dim_min = EEPROM.read(OFFSET_DIM_MIN) * 100 + EEPROM.read(OFFSET_DIM_MIN+1);
    #ifdef DEBUG1
      Serial.printf("dim_versatz = %d, dim_min = %d, %s\n",dim_versatz,dim_min,
        (Phasenabschnitt?"Phasenabschnitt":"Phasenanschnitt"));
    #endif
  }
  calc_dimmer_tab();

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_topic);
  wifiManager.setTimeout(180); 
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
  if (!wifiManager.autoConnect(AP_SSID, "qwertzui")) {  
    delay(3000);
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();                     
  }
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(base_mqtt_topic, custom_mqtt_topic.getValue());

  if (mqtt_server[0] == '\0') {
    #ifdef DEBUG1
      Serial.println("MQTT-Werte aus EEPROM holen . . .");
    #endif
    
    for (short i = 0; i < 40; i++) mqtt_server[i] = char(EEPROM.read(i));
    for (short i = 0; i < 5; i++) mqtt_port[i] = char(EEPROM.read(i + 40));
    for (short i = 0; i < MAXTOPICSIZE; i++) 
      base_mqtt_topic[i] = char(EEPROM.read(i + 45));
  } else {
    #ifdef DEBUG1
      Serial.println("MQTT-Werte ins EEPROM schreiben . . .");
    #endif

    // Adresse  0 - 39
    for (short i = 0; i < 40; i++) EEPROM.write(i, char(mqtt_server[i]));         
    // Adresse 40 - 44
    for (short i = 0; i < 5; i++) EEPROM.write(i + 40, mqtt_port[i]);           
    // Adresse 45 - 88
    for (short i = 0; i < MAXTOPICSIZE; i++) 
      EEPROM.write(i + 45, base_mqtt_topic[i]);
    EEPROM.commit();
  }

  // if you get here you have connected to the WiFi
  short port = atoi(mqtt_port);

  strlcpy(status_mqtt_topic, "stat/", MAXTOPICSIZE); 
  strlcat(status_mqtt_topic, base_mqtt_topic, MAXTOPICSIZE);
  strlcpy(cmnd_mqtt_topic, "cmnd/", MAXTOPICSIZE); 
  strlcat(cmnd_mqtt_topic, base_mqtt_topic, MAXTOPICSIZE);

  #ifdef DEBUG1
    Serial.printf("MQTT-Server: %s:%d\n", mqtt_server, port);
    Serial.print("MQTT-Topic Base: ");
    Serial.println(base_mqtt_topic);
    Serial.print("MQTT-Topic Status: ");
    Serial.println(status_mqtt_topic);
    Serial.print("MQTT-Topic Command: ");
    Serial.println(cmnd_mqtt_topic);
  #endif

  client.setServer(mqtt_server, port);
  client.setCallback(callback);         // MQTT Eingangsroutine festlegen

  // TASTER/Touch mit Ruhe oder Arbeitskontakt, bzw. bei Touch GND im Ruhezustand
  ta_ruhe = digitalRead(TASTER);      

  delay(10);
  calcswitchtime();
  attachInterrupt(digitalPinToInterrupt(PIN_SYNC),SyncISR,CHANGE);
  timer1_attachInterrupt(TimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
}
//*********************************************************************************
void loop()
{
  // TASTER zum Dimmen wird abgefragt
  tasterabfrage();

  client_connected_unblocking();
}
//*********************************************************************************
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
  if (Phasenabschnitt) {
    Next_low = ti_dimmer;
    Next_high = HWLEN;
  } else {
    Next_low = HWLEN;
    Next_high = HWLEN-ti_dimmer;
  }
  #ifdef DEBUG1
    Serial.printf("Next_low = %d, Next_high = %d\n",Next_low,Next_high);
  #endif
}
//*********************************************************************************
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
  unsigned long now = micros()-dim_versatz;
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
      digitalWrite(14,sgn = !sgn);
    #endif
    
    if (dim_on) {

      long wait_next;
      
      if ((Phasenabschnitt && (dim_versatz < Next_low)) || 
          (!Phasenabschnitt && (dim_versatz >= Next_high)))
      {
        pwm(HIGH);
        wait_next = Next_low-dim_versatz;
      } else {
        pwm(LOW);
        wait_next = Next_high-dim_versatz;
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
//*********************************************************************************
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
    digitalWrite(5,sgn = !sgn);
  #endif

  if ((!Phasenabschnitt && sdelta > Next_high) || // Anschnitt
      (Phasenabschnitt && sdelta < Next_low)) {  // Abschnitt
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
//*********************************************************************************
ICACHE_RAM_ATTR void pwm(boolean wert)   
// hier wird der Port zum MOS-FET geschaltet 
{
  static boolean fl_pwm = HIGH;

  if (!dim_on || (ti_dimmer < ti_dim_min)) wert = LOW; // Unterhalb Minimum -> aus
  else if (ti_dimmer > ti_dim_max) wert = HIGH; // Oberhalb Max. --> volle Helligk.
  
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
//*********************************************************************************
void client_connected_unblocking()
{
  static unsigned long last_reconnect = 0;
  #ifdef DEBUG1
    static unsigned long last_alive = 0;
  #endif
  unsigned long now = millis();

  if (!client.connected()) {
    if (now - last_reconnect > 5000) {
      last_reconnect = now;
      if (reconnect()) last_reconnect = 0;
    }
  } else {
    last_reconnect = 0; // to avoid overrun
    #ifdef DEBUG1
      if (now - last_alive > 10000) {
        Serial.println("Connection alive!");
        last_alive = now;
      } 
    #endif
    client.loop();
  }
}
//*********************************************************************************
boolean reconnect()
{
  if (client.connect("ESP8266Dimmer")) {
    #ifdef DEBUG1
      Serial.println("Reconnect succesfull!");
    #endif
    client.subscribe(cmnd_mqtt_topic); // ... and resubscribe
  } else {
    #ifdef DEBUG1
      Serial.println("Reconnect failed!");
    #endif
  }
  return client.connected();
}
//*********************************************************************************
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
        Serial.printf("Zeitmessung %s\n", (time_measure?"an":"aus"));
      } 
      break;
  #endif
  case 'r':
    if (strncmp((const char *)(payload+1), RESET_SECRET, RESET_SECRET_LEN) != 0)
      break;
    EEPROM.write(OFFSET_MAGIC, 0); // lösche magic cookie
    EEPROM.write(OFFSET_MAGIC + 1, 0);
    EEPROM.commit();

    delay(3000);
    ESP.reset();
  case 'v':                     
    // Wert in µs in dem das SYNC-Signal verzögert ist, wird für exakten 
    // Phasen-An-oder Abschnitt benötigt und im EEPROM gespeichert
    if (payload[1] == '+') dim_versatz += 100;
    else if (payload[1] == '-') dim_versatz -= 100;
    else dim_versatz = atoi((const char *)(payload+1));
    if (dim_versatz < -HWLEN) dim_versatz = -HWLEN;
    if (dim_versatz > HWLEN) dim_versatz = HWLEN;

    // Oberes Byte in EEPROM schreiben an Adresse 89
    EEPROM.write(OFFSET_DIM_VERSATZ, ((uint16_t)dim_versatz >> 8));    
    // Unteres Byte in EEPROM schreiben an Adresse 90
    EEPROM.write(OFFSET_DIM_VERSATZ+1, ((uint16_t)dim_versatz & 0xFF));    
    EEPROM.commit();

    mqtt_status_publish("dim_versatz", dim_versatz);
    break;
  case 'n':                     
    // aNschnitt: Dimmer soll im Phasen Anschnittmodus arbeiten
    // 'n' für Phasenanschnitt in EEPROM schreiben an Adresse 91
    EEPROM.write(OFFSET_PHASENABSCHNITT, 'n');  
    EEPROM.commit();
    Phasenabschnitt = false;
    mqtt_status_publish_txt("Phasenabschnitt","false");
    break;
  case 'b':                     
    // aBschnitt: Dimmer soll im Phasen Abschnittmodus arbeiten
    // 'b' für Phasenanschnitt in EEPROM schreiben an Adresse 91
    EEPROM.write(OFFSET_PHASENABSCHNITT, 'b');  
    EEPROM.commit();
    Phasenabschnitt = true;
    mqtt_status_publish_txt("Phasenabschnitt","true");
    break;
  case 'm': 
    // Minimum: Wert bei dem die zu steuernde Lampe gerade noch leuchtet
    dim_min = atoi((const char*)(payload+1));
    calc_dimmer_tab();

    // 1000er und 100er in EEPROM schreiben an Adresse 92
    EEPROM.write(OFFSET_DIM_MIN, dim_min / 100);
    // 10er und einer in EEPROM schreiben an Adresse 93
    EEPROM.write(OFFSET_DIM_MIN+1, dim_min % 100);
    EEPROM.commit();

    mqtt_status_publish("dim_min",dim_min);
    break;
  case 's':
    // Status auf MQTT ausgeben
    mqtt_status_publish_txt("File",__FILE__);
    mqtt_status_publish_txt("Date",__DATE__);
    mqtt_status_publish_txt("Time",__TIME__);
    mqtt_status_publish_txt("Phasenabschnitt",(Phasenabschnitt?"true":"false"));
    mqtt_status_publish("dim_versatz",dim_versatz);
    mqtt_status_publish("dim_min",dim_min);
    mqtt_status_publish_txt("dim_on",(dim_on?"true":"false"));
    mqtt_status_publish("dim_prozent",dim_prozent);
    mqtt_status_publish("ti_dimmer",ti_dimmer);
    break;
  case '+':
    if (!dim_on) dim_on = true;    
    else {
      dim_on = true;    
      dim_prozent += 5;
      if (dim_prozent >= 100) set_ti_dimmer_max();
      else set_ti_dimmer_by_prozent(dim_prozent);    
    }
    #ifdef DEBUG1
      Serial.printf("Dimwert: %d = %d%\n",dim_prozent,ti_dimmer);
    #endif
    break;
  case '-':
    if (!dim_on) dim_on = true;    
    else {
      dim_prozent -= 5;
      if (dim_prozent < 0) set_ti_dimmer_min();
      else set_ti_dimmer_by_prozent(dim_prozent);    
      dim_on = (dim_prozent > 0);
    }
    #ifdef DEBUG1
      Serial.printf("Dimwert: %d = %d%\n",dim_prozent,ti_dimmer);
    #endif
    break;
  default: //Dimmer wird mit Zahl zwischen 0 und 100 eingestellt
    payload[(length < 3)?length:3] = '\0';
    short mqtt_dimwert = atoi((const char *)payload);
    
    if (mqtt_dimwert <= 0) {
      set_ti_dimmer_min();
      dim_on = false;                      // Dimmer ausschalten
    } else if (mqtt_dimwert >= 100) {
      set_ti_dimmer_max();
      dim_on = true;                      // Dimmer einschalten
    } else { 
      set_ti_dimmer_by_prozent(mqtt_dimwert);
      dim_on = true;
    }
    #ifdef DEBUG1
      Serial.printf("Dimwert: %d = %d%\n",mqtt_dimwert,ti_dimmer);
    #endif
    break;
  }
  calcswitchtime();
}
//*********************************************************************************
void set_ti_dimmer_by_prozent(unsigned short mqtt_dimwert) {
  // dim_wert (0-100 Prozent) in ti_dimmer umrechnen
  // ti_dimmer = long(mqtt_dimwert) * long(DIM_MAX - dim_min) / 100 + dim_min;
  if (mqtt_dimwert > 100) mqtt_dimwert = 100;
  else if (mqtt_dimwert < 0) mqtt_dimwert = 0;
  ti_dimmer = dim_val[mqtt_dimwert];
  dim_prozent = mqtt_dimwert;
}
//*********************************************************************************
void set_ti_dimmer_max() {
  ti_dimmer = dim_val[100];
  dim_prozent = 100;
}
//*********************************************************************************
void set_ti_dimmer_min() {
  ti_dimmer = dim_val[0];
  dim_prozent = 0;
}
//*********************************************************************************
void increase_ti_dimmer() {
  dim_prozent += 2;
  if (dim_prozent >= 100) set_ti_dimmer_max();
  else set_ti_dimmer_by_prozent(dim_prozent);
}
//*********************************************************************************
void decrease_ti_dimmer() {
  dim_prozent -= 2;
  if (dim_prozent <= 0) set_ti_dimmer_min();
  else set_ti_dimmer_by_prozent(dim_prozent);
}
//*********************************************************************************
void calc_dimmer_tab() {
  /*
    Diese Bestimmung des Schaltpunktes zum Dimmen basiert auf dem Integral unter 
    der Sinus-Kurve und gibt für jeden Prozentwertpunkt ungefähr die gleiche 
    Energiemenge frei - unter Berücksichtigung der Blindleistung, die mit dim_min
    absolut festgelegt wird.
  */
  for (short i = 0; i <= 100; i++) 
    dim_val[i] = int(acos(1.0 - i * (2.0 - (dim_min / (HWLEN / 2.0))) / 
                 100.0 - (dim_min / (HWLEN/2.0))) * DIM_MAX / PI);
  ti_dim_max = dim_val[99]; // 99%
  ti_dim_min = dim_val[0]; // 0%
}
//*********************************************************************************
void mqtt_status_publish(const char *status_subtopic, long statuszahl)
{
  char buff[15];
  ltoa(statuszahl, buff, 10);
  mqtt_status_publish_txt(status_subtopic, buff);
}
//*********************************************************************************
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
//*********************************************************************************
void mqtt_publish()
{
  unsigned short dim_prozent = 0;
  char dim_als_char[7];

  if (dim_on) {
    if (ti_dimmer <= dim_min) dim_prozent = 1; // kleinster Dim-Wert ist nicht aus
    else if (ti_dimmer >= DIM_MAX) dim_prozent = 100;
    else dim_prozent = ((ti_dimmer - dim_min) * 100UL) / (DIM_MAX - dim_min);
  }
  itoa(dim_prozent, dim_als_char, 10);
  strlcat(dim_als_char, "%", 7);
  
  mqtt_status_publish_txt("Dimmer", dim_als_char); //PUBLIZIEREN
  #ifdef DEBUG1
    Serial.printf("Publish %s:%d\n", status_mqtt_topic, dim_prozent);
    if ((ti_dimmer < dim_min) || (ti_dimmer > DIM_MAX))
      Serial.println("Dimmer out of range!");
  #endif
}
//*********************************************************************************
void tasterabfrage()
{
  static boolean ta_war_gedr = false,
                 dim_richtung_auf = false,
                 am_dimmen = false;
  static short on_off_or_dim = TOUCHWAIT;
  static unsigned long ti_taster = 0;
  boolean ta_ist_gedr = (digitalRead(TASTER) != ta_ruhe);

  if (ta_ist_gedr) { // TASTER/Touch gedrückt
    if (!ta_war_gedr) {
      ta_war_gedr = true;
      ti_taster = millis();
    } else {
      unsigned long ti_taster1 = millis();
      if (ti_taster1 - ti_taster > on_off_or_dim) { //bei >400ms dimmen
        if (dim_richtung_auf) {
          increase_ti_dimmer();
        } else {
          decrease_ti_dimmer();
          if (ti_dimmer <= ti_dim_min) {
            dim_richtung_auf = true;
          }
        }
        calcswitchtime();
        ti_taster = ti_taster1;
         // alle x ms Wertänderung
        on_off_or_dim = (ti_dimmer <= ti_dim_min)?TOUCHWAIT:TOUCHSPEED;
        am_dimmen = true;
        dim_on = true;
      }
    }
  } else {
    if (ta_war_gedr) {
      if ((millis() - ti_taster > 10) && !am_dimmen) {     // entprellen
        if (dim_on) {
          dim_on = false;                                  // Dimmer aus
        } else {
          dim_on = true;
          if ((ti_dimmer < ti_dim_min) || (ti_dimmer > ti_dim_max)) {
            set_ti_dimmer_max();
            calcswitchtime();
          }
          dim_richtung_auf = (ti_dimmer < dim_val[50]); // 50%
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
//*********************************************************************************
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
//*********************************************************************************
