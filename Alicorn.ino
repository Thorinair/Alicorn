#include "ESP8266WiFi.h"
#include <EEPROM.h>
#include <VariPass.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremoteESP8266.h>
#include <SFE_BMP180.h>
#include <RtcDS3231.h>

#include "wifi.h"
#include "vpid.h"

extern "C" {
    #include "user_interface.h"
}

// Pins
#define PIN_RELAY_GEIG     0
#define PIN_DHT            12
#define PIN_GEIGER         13
#define PIN_IR             14
#define PIN_BUZZER         15
#define PIN_RELAY_GAS      16
#define PIN_MQ             A0

// I2C Addresses
#define I2C_LCD 0x27
#define I2C_RTC 0x57
#define I2C_BMP 0x77

// Default Settings
#define DEFAULT_LCD_BACKLIGHT    1
#define DEFAULT_SCREEN_MAIN      0
#define DEFAULT_SCREEN_SETT      0
#define DEFAULT_REMOTE_BEEPS     1
#define DEFAULT_GAS_SENSOR       1
#define DEFAULT_GEIG_CLICKS      1
#define DEFAULT_GEIG_ALARM       1
#define DEFAULT_GEIG_SENSITIVITY 1000
#define DEFAULT_INT_MEASURE      10000
#define DEFAULT_INT_PUSH         60000
#define DEFAULT_INT_PULL         1000
#define DEFAULT_INT_SYNC         1000
#define DEFAULT_WIFI             0

// MinMaxStep Settings
#define MIN_GEIG_SENSITIVITY 100
#define MIN_INT_MEASURE      2000
#define MIN_INT_PUSH         1000
#define MIN_INT_PULL         1000
#define MIN_INT_SYNC         1000

#define MAX_GEIG_SENSITIVITY 10000
#define MAX_INT_MEASURE      60000
#define MAX_INT_PUSH         60000
#define MAX_INT_PULL         60000
#define MAX_INT_SYNC         60000

#define STP_GEIG_SENSITIVITY 100
#define STP_INT_MEASURE      1000
#define STP_INT_PUSH         1000
#define STP_INT_PULL         1000
#define STP_INT_SYNC         1000

// EEPROM Addresses
#define EEPROM_SAVED            0
#define EEPROM_LCD_BACKLIGHT    1
#define EEPROM_SCREEN_MAIN      2
#define EEPROM_SCREEN_SETT      3
#define EEPROM_REMOTE_BEEPS     4
#define EEPROM_GAS_SENSOR       5
#define EEPROM_GEIG_CLICKS      6
#define EEPROM_GEIG_ALARM       7
#define EEPROM_GEIG_SENSITIVITY 8
#define EEPROM_INT_MEASURE      9
#define EEPROM_INT_PUSH         10
#define EEPROM_INT_PULL         11
#define EEPROM_INT_SYNC         12
#define EEPROM_WIFI             13

// Screens
#define SCREENS_MAIN 7
#define SCREENS_SETT 11

// Main Screens
#define SCREEN_MAIN_TIME      0
#define SCREEN_MAIN_DHT       1
#define SCREEN_MAIN_BMPMQ     2
#define SCREEN_MAIN_GEIGER    3
#define SCREEN_MAIN_CORE      4
#define SCREEN_MAIN_BULLETIN  5
#define SCREEN_MAIN_WIFI      6

// Settings Screens
#define SCREEN_SETT_REMOTE_BEEPS     0
#define SCREEN_SETT_GAS_SENSOR       1
#define SCREEN_SETT_GEIG_CLICKS      2
#define SCREEN_SETT_GEIG_ALARM       3
#define SCREEN_SETT_GEIG_SENSITIVITY 4
#define SCREEN_SETT_INT_MEASURE      5
#define SCREEN_SETT_INT_PUSH         6
#define SCREEN_SETT_INT_PULL         7
#define SCREEN_SETT_INT_SYNC         8
#define SCREEN_SETT_WIFI             9
#define SCREEN_SETT_DATETIME         10

// Intervals
#define INTERVAL_CYCLE  100
#define INTERVAL_TIMER  100
#define INTERVAL_ALARM  250
#define INTERVAL_LCD    1000
#define INTERVAL_GEIGER 60000
#define INTERVAL_CLOCK  1000

// Compact Levels
#define COMPACT_NONE   0
#define COMPACT_LONG   1
#define COMPACT_MEDIUM 2
#define COMPACT_SHORT  3

// Buzzer
#define BUZZER_REMOTE_TONE     3000
#define BUZZER_REMOTE_DURATION 100
#define BUZZER_ALARM_TONE      4000

// DHT22
DHT dht(PIN_DHT, DHT22);

// BMP180
#define ALTITUDE 123
SFE_BMP180 bmp;

// RTC
#define DATETIME_HOURS   0
#define DATETIME_MINUTES 1
#define DATETIME_SECONDS 2
#define DATETIME_YEAR    3
#define DATETIME_MONTH   4
#define DATETIME_DAY     5
RtcDS3231<TwoWire> rtc(Wire);
RtcDateTime now;

// Geiger
#define DOSE_MULTI 0.0057

// LCD
LiquidCrystal_I2C lcd(I2C_LCD, 16, 2);

// Remote
#define IR_BACKLIGHT 0xFF629D // CH
#define IR_PREV      0xFF22DD // <<
#define IR_NEXT      0xFF02FD // >>
#define IR_SETTINGS  0xFFC23D // >||
#define IR_DECREASE  0xFFE01F // -
#define IR_INCREASE  0xFFA857 // +
#define IR_DATETIME  0xFF906F // EQ
#define IR_S0        0xFF6897 // 0
#define IR_S1        0xFF30CF // 1
#define IR_S2        0xFF18E7 // 2
#define IR_S3        0xFF7A85 // 3
#define IR_S4        0xFF10EF // 4
#define IR_S5        0xFF38C7 // 5
#define IR_S6        0xFF5AA5 // 6
#define IR_S7        0xFF42BD // 7
#define IR_S8        0xFF4AB5 // 8
#define IR_S9        0xFF52AD // 9
#define IR_S10       0xFF9867 // 100+
#define IR_S11       0xFFB04F // 200+
IRrecv irrecv(PIN_IR);
decode_results results;

// WiFi
char* host;
char* ssid[COUNT_WIFI];
char* pass[COUNT_WIFI];

// Timers
os_timer_t timer;
os_timer_t timerAlarm;

// Structures
struct SETTINGS {
    bool lcdBacklight;
    int  screenMain;
    int  screenSett;
    bool remoteBeeps;
    bool gasSensor;
    bool geigerClicks;
    bool geigerAlarm;
    int  geigerSensitivity;
    int  intervalMeasure;
    long intervalGeiger;
    int  intervalPush;
    int  intervalPull;
    int  intervalSync;
    int  wifi;
} settings;

struct STATES {
    bool screenSettings;
    bool alarm;
    bool alarmOn;
    bool pushSync;
    int  datePage;
    int  wifi;
} states;

struct COUNTERS {
    int measure;
    int geiger;
    int cclock;
    int push;
    int pull;
    int sync;
    int lcd;
} counters;

struct INTERVALS {
    bool measure;
    bool geiger;
    bool cclock;
    bool push;
    bool pull;
    bool sync;
    bool lcd;
} intervals;

struct DATA {
    float  temperature;
    float  humidity;
    float  pressure;
    float  gas;
    int    cpmNow;
    int    cpm;
    float  dose;
    long   core;
    long   gain;
    String bulletin;
} data;

struct AVERAGE {
    float  temperature;
    int    temperatureCount;
    float  humidity;
    int    humidityCount;
    float  pressure;
    int    pressureCount;
    float  gas;
    int    gasCount;
} average;

// Utilities
void resetLCD();
void setRelays();
void drawScreen(String top, String bot);
bool checkInterval(int *counter, int interval);
void resetAverage();
void geigerClick();

// String Manipulation
String formatNumbers(long number, int compact, bool usePrefix, bool useSuffix, String suffix);
String splitData(String data, char separator, int index);
String boolToOnOff(bool value);

// Setups
void setupSettings();
void setupStates();
void setupCounters();
void setupWiFi();
void setupDevices();
void setupClock();
void setupTimer();

// Settings
void saveSettings();
void loadSettings();

// WiFi
void connectWiFi();

// Processes
void processTimer(void *pArg);
void processAlarm(void *pArg);
void processRemote();
void processSensors();
void processGeiger();
void processClock();
void processPush();
void processPull();
void processSync();
void processLCD();

// VariPass
void pullVariPass();
void pushVariPass();
void syncVariPass();

// Remote
void screenPrev();
void screenNext();
void screenSet(int page);
void valueDecrease();
void valueIncrease();

// Beeps
void beepRemote();

/* ===========
 *  Utilities 
 * =========== */

void resetLCD() {
    pullVariPass();
    counters.lcd = 0;
    beepRemote();
}

void setRelays() {
    if (settings.geigerClicks)
        digitalWrite(PIN_RELAY_GEIG, HIGH);
    else
        digitalWrite(PIN_RELAY_GEIG, LOW);
        
    if (settings.gasSensor)
        digitalWrite(PIN_RELAY_GAS, LOW);
    else
        digitalWrite(PIN_RELAY_GAS, HIGH);
}

void drawScreen(String top, String bot) {
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(top);
    lcd.setCursor(0,1); 
    lcd.print(bot);
}

bool checkInterval(int *counter, int interval) {
    if (*counter <= 0) {
        *counter = interval / INTERVAL_TIMER;
        return true;
    }
    else {
        (*counter)--;
        return false;
    }
}

void resetAverage() {
    average.temperature = 0;
    average.humidity    = 0;
    average.pressure    = 0;
    average.gas         = 0;
    
    average.temperatureCount = 0;
    average.humidityCount    = 0;
    average.pressureCount    = 0;
    average.gasCount         = 0;
}

void geigerClick() {
    data.cpmNow++;
}

/* =====================
 *  String Manipulation
 * ===================== */

String formatNumbers(long number, int compact, bool usePrefix, bool useSuffix, String suffix) {
    String out;
    if (compact == COMPACT_LONG) {
        if (number >= 1000000000) {
            number = number / 1000;
            if (useSuffix)
                out = " M" + suffix;
        }
        else {
            if (useSuffix)
                out = " K" + suffix;
        } 
    }
    else if (compact == COMPACT_SHORT) {
        if (number >= 1000000000) {
            number = number / 1000000000;
            if (useSuffix)
                out = " T" + suffix;
        }
        else if (number >= 1000000) {
            number = number / 1000000;
            if (useSuffix)
                out = " G" + suffix;
        }
        else if (number >= 1000) {
            number = number / 1000;
            if (useSuffix)
                out = " M" + suffix;
        }
        else {
            if (useSuffix)
                out = " K" + suffix;
        } 
    }
    String data = String(abs(number));
    
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 3; j++) {
            if ((i*3 + j) < data.length())
                out = data.charAt(data.length() - 1 - (i*3 + j)) + out;
            else
                out = " " + out;
        }
        out = " " + out;
    }
    
    out.trim();
    
    if (number < 0)
        out = "-" + out;
    else if (usePrefix)
        out = "+" + out;
    
    return out;
}

String splitData(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for(int i = 0; i <= maxIndex && found <= index; i++) {
        if(data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

String boolToOnOff(bool value) {
    if (value)
        return "ON";
    else    
        return "OFF";
}

/* ========
 *  Setups 
 * ======== */

void setupSettings() {
    EEPROM.begin(512);
    int saved = EEPROM.read(EEPROM_SAVED);
    EEPROM.end();
    
    if (saved == 1) {
        Serial.println("\nSettings already exist. Loading...");
        loadSettings();
    }
    else {       
        settings.lcdBacklight      = DEFAULT_LCD_BACKLIGHT;
        settings.screenMain        = DEFAULT_SCREEN_MAIN;
        settings.screenSett        = DEFAULT_SCREEN_SETT;
        settings.remoteBeeps       = DEFAULT_REMOTE_BEEPS;
        settings.gasSensor         = DEFAULT_GAS_SENSOR;
        settings.geigerClicks      = DEFAULT_GEIG_CLICKS;
        settings.geigerAlarm       = DEFAULT_GEIG_ALARM;
        settings.geigerSensitivity = DEFAULT_GEIG_SENSITIVITY;
        settings.intervalMeasure   = DEFAULT_INT_MEASURE;
        settings.intervalPush      = DEFAULT_INT_PUSH;
        settings.intervalPull      = DEFAULT_INT_PULL;
        settings.intervalSync      = DEFAULT_INT_SYNC;
        settings.wifi              = DEFAULT_WIFI;
        
        Serial.println("\nCreated new settings. Saving...");
        saveSettings();
    }

    lcd.setBacklight(settings.lcdBacklight);
}

void setupStates() {
    states.screenSettings = false;
    states.alarm          = false;
    states.alarmOn        = false;
    states.datePage       = 0;
    states.wifi           = settings.wifi;
}

void setupCounters() {
    counters.measure = 0;
    counters.push    = 0;
    counters.pull    = 0;
    counters.sync    = 0;
}

void setupWiFi() {
    host    = WIFI_HOST;
    ssid[0] = WIFI_SSID_0;
    ssid[1] = WIFI_SSID_1;
    pass[0] = WIFI_PASS_0;
    pass[1] = WIFI_PASS_1;
}

void setupDevices() {
    dht.begin();
    bmp.begin();
    irrecv.enableIRIn();
        
    pinMode(PIN_RELAY_GEIG, OUTPUT);
    pinMode(PIN_RELAY_GAS,  OUTPUT);
    
    pinMode(PIN_GEIGER, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_GEIGER), geigerClick, CHANGE);
    
    setRelays();
}

void setupClock() {
    rtc.Begin();
    
    if (!rtc.IsDateTimeValid()) {
        RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
        Serial.println("RTC lost confidence in the DateTime!");
        rtc.SetDateTime(compiled);
    }

    if (!rtc.GetIsRunning()) {
        Serial.println("RTC was not actively running, starting now");
        rtc.SetIsRunning(true);
    }
    
    rtc.Enable32kHzPin(false);
    rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
}

void setupTimer() {
    os_timer_setfn(&timer, processTimer, NULL);
    os_timer_arm(&timer, INTERVAL_TIMER, true);
    os_timer_setfn(&timerAlarm, processAlarm, NULL);
    os_timer_arm(&timerAlarm, INTERVAL_ALARM, true);
}

/* ==========
 *  Settings 
 * ========== */

void saveSettings() {
    EEPROM.begin(512);
    EEPROM.write(EEPROM_SAVED,            1);
    EEPROM.write(EEPROM_LCD_BACKLIGHT,    settings.lcdBacklight);
    EEPROM.write(EEPROM_SCREEN_MAIN,      settings.screenMain);
    EEPROM.write(EEPROM_SCREEN_SETT,      settings.screenSett);
    EEPROM.write(EEPROM_REMOTE_BEEPS,     settings.remoteBeeps);
    EEPROM.write(EEPROM_GAS_SENSOR,       settings.gasSensor);
    EEPROM.write(EEPROM_GEIG_CLICKS,      settings.geigerClicks);
    EEPROM.write(EEPROM_GEIG_ALARM,       settings.geigerAlarm);
    EEPROM.write(EEPROM_GEIG_SENSITIVITY, settings.geigerSensitivity / 100);
    EEPROM.write(EEPROM_INT_MEASURE,      settings.intervalMeasure   / 1000);
    EEPROM.write(EEPROM_INT_PUSH,         settings.intervalPush      / 1000);
    EEPROM.write(EEPROM_INT_PULL,         settings.intervalPull      / 1000);
    EEPROM.write(EEPROM_INT_SYNC,         settings.intervalSync      / 1000);
    EEPROM.write(EEPROM_WIFI,             settings.wifi);
    EEPROM.end();
    Serial.println("Saved settings to EEPROM.");
}

void loadSettings() {
    EEPROM.begin(512);
    settings.lcdBacklight      = (bool) EEPROM.read(EEPROM_LCD_BACKLIGHT);
    settings.screenMain        = (int)  EEPROM.read(EEPROM_SCREEN_MAIN);
    settings.screenSett        = (int)  EEPROM.read(EEPROM_SCREEN_SETT);
    settings.remoteBeeps       = (bool) EEPROM.read(EEPROM_REMOTE_BEEPS);
    settings.gasSensor         = (bool) EEPROM.read(EEPROM_GAS_SENSOR);
    settings.geigerClicks      = (bool) EEPROM.read(EEPROM_GEIG_CLICKS);
    settings.geigerAlarm       = (bool) EEPROM.read(EEPROM_GEIG_ALARM);
    settings.geigerSensitivity = (int)  EEPROM.read(EEPROM_GEIG_SENSITIVITY) * 100;
    settings.intervalMeasure   = (int)  EEPROM.read(EEPROM_INT_MEASURE)      * 1000;
    settings.intervalPush      = (int)  EEPROM.read(EEPROM_INT_PUSH)         * 1000;
    settings.intervalPull      = (int)  EEPROM.read(EEPROM_INT_PULL)         * 1000;
    settings.intervalSync      = (int)  EEPROM.read(EEPROM_INT_SYNC)         * 1000;
    settings.wifi              = (int)  EEPROM.read(EEPROM_WIFI);
    EEPROM.end();
    Serial.println("Loaded settings from EEPROM.");
}

/* ======
 *  WiFi 
 * ====== */

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.hostname(host);

    if (settings.wifi < COUNT_WIFI)
        WiFi.begin(ssid[settings.wifi], pass[settings.wifi]);
}

/* ===========
 *  Processes
 * =========== */

void processTimer(void *pArg) {
    //Serial.println("Timer tick!");
    if (checkInterval(&counters.measure, settings.intervalMeasure))
        intervals.measure = true;
        
    if (checkInterval(&counters.geiger,  INTERVAL_GEIGER))
        intervals.geiger  = true;

    if (checkInterval(&counters.cclock,  INTERVAL_CLOCK))
        intervals.cclock  = true;

    if (checkInterval(&counters.push,    settings.intervalPush))
        intervals.push    = true;
        
    if (checkInterval(&counters.pull,    settings.intervalPull))
        intervals.pull    = true;
        
    if (checkInterval(&counters.sync,    settings.intervalSync))
        intervals.sync    = true;

    if (checkInterval(&counters.lcd,     INTERVAL_LCD))
        intervals.lcd     = true;            
}

void processAlarm(void *pArg) {
    if (settings.geigerAlarm && states.alarm) {
        if (states.alarmOn) {
            noTone(PIN_BUZZER); 
            states.alarmOn = false; 
        }
        else {  
            tone(PIN_BUZZER, BUZZER_ALARM_TONE);
            states.alarmOn = true;    
        }
    }
}

void processRemote() {
    if (irrecv.decode(&results)) {
        if (irrecv.decode(&results)) {
            if (((results.value & 0xFF0000) == 0xFF0000) && (results.value != 0xFFFFFFFF)) {
                //Serial.println(results.value, HEX);

                switch(results.value) {
                    case IR_BACKLIGHT:
                        settings.lcdBacklight = !settings.lcdBacklight;
                        saveSettings();
                        
                        counters.lcd = 0;
                        lcd.setBacklight(settings.lcdBacklight);
                        if (!settings.lcdBacklight)
                            lcd.clear();
                            
                        beepRemote();
                        break;

                    case IR_SETTINGS:
                        if (settings.lcdBacklight) {
                            states.screenSettings = !states.screenSettings;

                            if (!states.screenSettings) {
                                if (states.wifi != settings.wifi)
                                    ESP.restart();
    
                                if (data.dose >= (float) settings.geigerSensitivity / 1000) {
                                    states.alarm = true;
                                }
                                else {
                                    states.alarm = false;  
                                    states.alarmOn = false;
                                    noTone(PIN_BUZZER); 
                                }                                
                            } 
                            
                            resetLCD();
                        }
                        break;

                    case IR_DATETIME:
                        if (settings.lcdBacklight && states.screenSettings && settings.screenSett == SCREEN_SETT_DATETIME) {
                            if (states.datePage >= 5)
                                states.datePage = 0;
                            else    
                                states.datePage++;      
                            resetLCD();
                        }
                        break;
                            

                    case IR_PREV:     screenPrev();    break;
                    case IR_NEXT:     screenNext();    break;
                    case IR_S0:       screenSet(0);    break;    
                    case IR_S1:       screenSet(1);    break;    
                    case IR_S2:       screenSet(2);    break;    
                    case IR_S3:       screenSet(3);    break;    
                    case IR_S4:       screenSet(4);    break;    
                    case IR_S5:       screenSet(5);    break;    
                    case IR_S6:       screenSet(6);    break;    
                    case IR_S7:       screenSet(7);    break;    
                    case IR_S8:       screenSet(8);    break;    
                    case IR_S9:       screenSet(9);    break; 
                    case IR_S10:      screenSet(10);   break;
                    //case IR_S11:      screenSet(11);   break;
                    case IR_DECREASE: valueDecrease(); break;
                    case IR_INCREASE: valueIncrease(); break;                            
                }
            }
            irrecv.resume();
        }
    }
}

void processSensors() {
    if (intervals.measure) {
        intervals.measure = false;
        
        float readValue;
        readValue = dht.readTemperature();
        if (String(readValue) != "nan") {
            data.temperature     = readValue; 
            average.temperature += readValue;         
        }
        else {
            Serial.println("Error reading Temperature.");
            average.temperature += data.temperature;   
        }
        average.temperatureCount++;  
            
        readValue = dht.readHumidity();
        if (String(readValue) != "nan") {
            data.humidity     = readValue;
            average.humidity += readValue;                 
        }
        else {
            Serial.println("Error reading Humidity.");
            average.humidity += data.humidity; 
        }
        average.humidityCount++; 

        char stat;
        double temp, pres,p0,a;        
        
        stat = bmp.startTemperature();
        if (stat != 0) {
            delay(stat);            
            stat = bmp.getTemperature(temp);
            if (stat != 0) {                            
                stat = bmp.startPressure(3);
                if (stat != 0) {
                    delay(stat);                    
                    stat = bmp.getPressure(pres, temp);
                    if (stat != 0) {                    
                        readValue = (float) bmp.sealevel(pres, ALTITUDE);
                        data.pressure     = readValue;
                        average.pressure += readValue; 
                        average.pressureCount++; 
                    }
                }
            }
        }

        if (settings.gasSensor) {
            readValue = ((1023 - (float) analogRead(PIN_MQ)) / 1023) * 100;
            data.gas     = readValue;
            average.gas += readValue; 
            average.gasCount++; 
        }
    }
}

void processGeiger() {
    if (intervals.geiger) {
        intervals.geiger = false;
        data.cpm = data.cpmNow;
        data.dose = ((float) data.cpm * DOSE_MULTI);
        data.cpmNow = 0;

        if (data.dose >= (float) settings.geigerSensitivity / 1000) {
            states.alarm = true;
        }
        else {
            states.alarm = false;  
            states.alarmOn = false;
            noTone(PIN_BUZZER); 
        }
    }
}

void processClock() {
    if (intervals.cclock) {
        intervals.cclock = false;

        if (!rtc.IsDateTimeValid())
            Serial.println("RTC lost confidence in the DateTime!");
        now = rtc.GetDateTime();  
    }
}

void processPush() {
    if (intervals.push) {
        intervals.push = false;
        pushVariPass();
    }
}

void processPull() {
    if (intervals.pull) {
        intervals.pull = false;
        pullVariPass();
    }
}

void processSync() {
    if (intervals.sync) {
        intervals.sync = false;
        syncVariPass();
    }
}

void processLCD() {
    if (intervals.lcd) {
        intervals.lcd = false;
        
        if (states.screenSettings) {
            switch (settings.screenSett) {
                case SCREEN_SETT_REMOTE_BEEPS:
                    drawScreen(
                        "> Remote Beeps", 
                        "Value: " + boolToOnOff(settings.remoteBeeps)
                    );
                    break;
                case SCREEN_SETT_GAS_SENSOR:
                    drawScreen(
                        "> Gas Sensor", 
                        "Value: " + boolToOnOff(settings.gasSensor)
                    );
                    break;
                case SCREEN_SETT_GEIG_CLICKS:
                    drawScreen(
                        "> Geiger Clicks", 
                        "Value: " + boolToOnOff(settings.geigerClicks)
                    );
                    break;
                case SCREEN_SETT_GEIG_ALARM:
                    drawScreen(
                        "> Geiger Alarm", 
                        "Value: " + boolToOnOff(settings.geigerAlarm)
                    );
                    break;
                case SCREEN_SETT_GEIG_SENSITIVITY:
                    drawScreen(
                        "> Geiger Sensit.", 
                        "Value: " + String((float) settings.geigerSensitivity / 1000, 2) + " uSv"
                    );
                    break;
                case SCREEN_SETT_INT_MEASURE:
                    drawScreen(
                        "> Measure Inter.", 
                        "Value: " + String(settings.intervalMeasure / 1000) + " s"
                    );
                    break;
                case SCREEN_SETT_INT_PUSH:
                    drawScreen(
                        "> Pushing Inter.", 
                        "Value: " + String(settings.intervalPush / 1000) + " s"
                    );
                    break;
                case SCREEN_SETT_INT_PULL:
                    drawScreen(
                        "> Pulling Inter.", 
                        "Value: " + String(settings.intervalPull / 1000) + " s"
                    );
                    break;
                case SCREEN_SETT_INT_SYNC:
                    drawScreen(
                        "> Syncing Inter.", 
                        "Value: " + String(settings.intervalSync / 1000) + " s"
                    );
                    break;
                case SCREEN_SETT_WIFI: {
                    String wifi = "0: DISABLED";
                    if (settings.wifi < COUNT_WIFI)
                        wifi = String(settings.wifi + 1) + ": " + ssid[settings.wifi];
                        
                    drawScreen(
                        "> WiFi Network", 
                        wifi
                    );  
                    break;                  
                }                
                case SCREEN_SETT_DATETIME: {
                    String title = "> Date and Time";
                    String value = "";
                    char shortString[3];
                    char longString[5];
                    switch (states.datePage) {
                        case DATETIME_HOURS:
                            snprintf_P(shortString, 3, PSTR("%02u"), now.Hour());
                            value = "Hours:   " + String(shortString);
                            break;
                        case DATETIME_MINUTES:
                            snprintf_P(shortString, 3, PSTR("%02u"), now.Minute());
                            value = "Minutes: " + String(shortString);
                            break;
                        case DATETIME_SECONDS:
                            snprintf_P(shortString, 3, PSTR("%02u"), now.Second());
                            value = "Seconds: " + String(shortString);
                            break;
                        case DATETIME_YEAR:
                            snprintf_P(longString, 5, PSTR("%02u"), now.Year());
                            value = "Year:    " + String(longString);
                            break;
                        case DATETIME_MONTH:
                            snprintf_P(shortString, 3, PSTR("%02u"), now.Month());
                            value = "Month:   " + String(shortString);
                            break;
                        case DATETIME_DAY:
                            snprintf_P(shortString, 3, PSTR("%02u"), now.Day());
                            value = "Day:     " + String(shortString);
                            break;
                    }
                    drawScreen(
                        title, 
                        value
                    );
                    break;
                }
            }
        } 
        else {
            switch (settings.screenMain) {
                case SCREEN_MAIN_TIME:
                    char timeString[6];
                    char dateString[11];
                    snprintf_P(timeString, 6, PSTR("%02u:%02u"), now.Hour(), now.Minute());
                    snprintf_P(dateString, 11, PSTR("%04u-%02u-%02u"), now.Year(), now.Month(), now.Day());
                    
                    drawScreen(
                        "     " + String(timeString), 
                        "   " + String(dateString)
                    );
                    break;
                case SCREEN_MAIN_DHT:
                    drawScreen(
                        "Temp: " + String(data.temperature, 1) + " C", 
                        "Humi: " + String(data.humidity, 1) + " %"
                    );
                    break;
                case SCREEN_MAIN_BMPMQ: {
                    String gas = "N/A";
                    if (settings.gasSensor)
                        gas = String(data.gas, 2) + " %";
                        
                    drawScreen(
                        "Prs: " + String(data.pressure, 2) + " hPa", 
                        "Air: " + gas
                    );
                    break;
                }
                case SCREEN_MAIN_GEIGER:
                    drawScreen(
                        "CPM: " + String(data.cpm) + " [" + String(data.cpmNow) + "]",
                        "Dos: " + String(data.dose, 2) + " uSv/h"
                    );
                    break;
                case SCREEN_MAIN_CORE:
                    if (WiFi.status() != WL_CONNECTED || settings.wifi == COUNT_WIFI)
                        drawScreen(
                            "WiFi Unavailable", 
                            ""
                        );
                    else   
                        drawScreen(
                            formatNumbers(data.core, COMPACT_LONG, false, true, "RF"), 
                            formatNumbers(data.gain, COMPACT_SHORT, true, true, "RF/t")
                        ); 
                    break;
                case SCREEN_MAIN_BULLETIN:
                    if (WiFi.status() != WL_CONNECTED || settings.wifi == COUNT_WIFI)
                        drawScreen(
                            "WiFi Unavailable", 
                            ""
                        );
                    else {
                        String bullA = data.bulletin;
                        String bullB = "";                    
                        if (bullA.length() > 16)
                            bullB = bullA.substring(16);
                            
                        drawScreen(
                            bullA, 
                            bullB
                        );
                    }    
                    break;  
                case SCREEN_MAIN_WIFI: {
                    String wifi = "  Connecting...";
                    if (settings.wifi == COUNT_WIFI)
                        wifi = "  Disabled";
                    else if (WiFi.status() == WL_CONNECTED)
                        wifi = "  Connected";
                        
                    drawScreen(
                        "WiFi Status:", 
                        wifi
                    ); 
                    break;
                }
            }
        } 
    }
}

/* ==========
 *  VariPass
 * ========== */

void pullVariPass() {
    if (WiFi.status() == WL_CONNECTED && !states.screenSettings)
        switch (settings.screenMain) {
            int result;
            case SCREEN_MAIN_CORE:
                long value1;
                
                value1 = varipassReadInt(KEY2, ID_CORE, &result);
                if (result == VARIPASS_RESULT_SUCCESS)
                    data.core = value1;
                else
                    Serial.println("An error has occured reading Core data! " + varipassGetResultDescription(result));
                    
                value1 = varipassReadInt(KEY2, ID_GAIN, &result);
                if (result == VARIPASS_RESULT_SUCCESS)
                    data.gain = value1;
                else
                    Serial.println("An error has occured reading Gain data! " + varipassGetResultDescription(result)); 
                     
                break;
                
            case SCREEN_MAIN_BULLETIN:
                String value2;
                
                value2 = varipassReadString(KEY1, ID_BULLETIN, &result);
                if (result == VARIPASS_RESULT_SUCCESS)
                    data.bulletin = value2;
                else
                    Serial.println("An error has occured reading Bulletin data! " + varipassGetResultDescription(result));
                     
                break;
        }
}

void pushVariPass() {
    if (WiFi.status() == WL_CONNECTED) {
        int result;
        varipassWriteFloat(KEY1, ID_TEMPERATURE, average.temperature / average.temperatureCount, &result);
        varipassWriteFloat(KEY1, ID_HUMIDITY,    average.humidity    / average.humidityCount,    &result);
        varipassWriteFloat(KEY1, ID_PRESSURE,    average.pressure    / average.pressureCount,    &result);
        if (settings.gasSensor)
            varipassWriteFloat(KEY1, ID_GAS,     average.gas         / average.gasCount,         &result);
        varipassWriteInt  (KEY1, ID_CPM,  data.cpm,  &result);
        varipassWriteFloat(KEY1, ID_DOSE, data.dose, &result);
    }
    resetAverage();
}

void syncVariPass() {
    if (WiFi.status() == WL_CONNECTED) {
        int result;

        if (states.pushSync) {
            Serial.println("Syncing data to VariPass...");
            varipassWriteBool(KEY1, ID_TGL_CLICKS, settings.geigerClicks, &result);
            if (result == VARIPASS_RESULT_SUCCESS)
                states.pushSync = false;
            else
                Serial.println("An error has occured writing geiger click settings! " + varipassGetResultDescription(result));
        }
        else {
            bool value;                
            value = varipassReadBool(KEY1, ID_TGL_CLICKS, &result);
            if (result == VARIPASS_RESULT_SUCCESS) {
                if (value != settings.geigerClicks) {
                    settings.geigerClicks = value;
                    setRelays();
                    saveSettings();
                    counters.lcd = 0;
                }                
            }
            else {
                Serial.println("An error has occured reading geiger click settings! " + varipassGetResultDescription(result));                   
            }  
        }
    }
}

/* ========
 *  Remote
 * ======== */

void screenPrev() {
    if (settings.lcdBacklight) {
        if (states.screenSettings) {
            if (settings.screenSett <= 0)
                settings.screenSett = SCREENS_SETT - 1;
            else
                settings.screenSett--;            
        }
        else {
            if (settings.screenMain <= 0)
                settings.screenMain = SCREENS_MAIN - 1;
            else
                settings.screenMain--;      
        }
        saveSettings();
        resetLCD();
    }
}

void screenNext() {
    if (settings.lcdBacklight) {
        if (states.screenSettings) {
            if (settings.screenSett >= SCREENS_SETT - 1)
                settings.screenSett = 0;
            else
                settings.screenSett++;            
        }
        else {
            if (settings.screenMain >= SCREENS_MAIN - 1)
                settings.screenMain = 0;
            else
                settings.screenMain++;  
        } 
        saveSettings();
        resetLCD();
    }
}

void screenSet(int page) {
    if (settings.lcdBacklight) {
        if (states.screenSettings) {
            if (page < SCREENS_SETT)
                settings.screenSett = page;  
        }
        else {
            if (page < SCREENS_MAIN)
                settings.screenMain = page;
        } 
        saveSettings();
        resetLCD();
    }
}

void valueDecrease() {
    if (states.screenSettings && settings.lcdBacklight) {
        bool process = false;
        switch (settings.screenSett) {
            case SCREEN_SETT_REMOTE_BEEPS:
                settings.remoteBeeps = !settings.remoteBeeps;
                process = true;
                break;
            case SCREEN_SETT_GAS_SENSOR:
                settings.gasSensor = !settings.gasSensor;
                setRelays();
                process = true;
                break;
            case SCREEN_SETT_GEIG_CLICKS:
                settings.geigerClicks = !settings.geigerClicks;
                setRelays();
                states.pushSync = true;
                process = true;
                break;
            case SCREEN_SETT_GEIG_ALARM:
                settings.geigerAlarm = !settings.geigerAlarm;
                process = true;
                break;
            case SCREEN_SETT_GEIG_SENSITIVITY:
                settings.geigerSensitivity -= STP_GEIG_SENSITIVITY;
                if (settings.geigerSensitivity < MIN_GEIG_SENSITIVITY)
                    settings.geigerSensitivity = MIN_GEIG_SENSITIVITY;
                process = true;
                break;
            case SCREEN_SETT_INT_MEASURE:
                settings.intervalMeasure -= STP_INT_MEASURE;
                if (settings.intervalMeasure < MIN_INT_MEASURE)
                    settings.intervalMeasure = MIN_INT_MEASURE;
                process = true;
                break;
            case SCREEN_SETT_INT_PUSH:
                settings.intervalPush -= STP_INT_PUSH;
                if (settings.intervalPush < MIN_INT_PUSH)
                    settings.intervalPush = MIN_INT_PUSH;
                process = true;
                break;
            case SCREEN_SETT_INT_PULL:
                settings.intervalPull -= STP_INT_PULL;
                if (settings.intervalPull < MIN_INT_PULL)
                    settings.intervalPull = MIN_INT_PULL;
                process = true;
                break;
            case SCREEN_SETT_INT_SYNC:
                settings.intervalSync -= STP_INT_SYNC;
                if (settings.intervalSync < MIN_INT_SYNC)
                    settings.intervalSync = MIN_INT_SYNC;
                process = true;
                break;
            case SCREEN_SETT_WIFI:
                if (settings.wifi <= 0)
                    settings.wifi = COUNT_WIFI;
                else    
                    settings.wifi--;
                process = true;
                break;                
            case SCREEN_SETT_DATETIME: {
                RtcDateTime newTime;
                switch (states.datePage) {
                    case DATETIME_HOURS:
                        newTime = now - 1 * 60 * 60;
                        break;
                    case DATETIME_MINUTES:
                        newTime = now - 1 * 60;
                        break;
                    case DATETIME_SECONDS:
                        newTime = now - 1;
                        break;
                    case DATETIME_YEAR:
                        if (now.Year() > 1970)
                            newTime = RtcDateTime(now.Year() - 1, now.Month(), now.Day(), now.Hour(), now.Minute(), now.Second());
                        break;
                    case DATETIME_MONTH:
                        if (now.Month() > 1)
                            newTime = RtcDateTime(now.Year(), now.Month() - 1, now.Day(), now.Hour(), now.Minute(), now.Second());
                        else    
                            newTime = RtcDateTime(now.Year() - 1, 12, now.Day(), now.Hour(), now.Minute(), now.Second());
                        break;
                    case DATETIME_DAY:
                        newTime = now - 1 * 60 * 60 * 24;
                        break;
                }
                rtc.SetDateTime(newTime);
                now = rtc.GetDateTime();
                resetLCD();
                break;
            }                    
        }
        if (process) {
            resetLCD();
            saveSettings();
        }
    }
}

void valueIncrease() {
    if (states.screenSettings && settings.lcdBacklight) {
        bool process = false;
        switch (settings.screenSett) {
            case SCREEN_SETT_REMOTE_BEEPS:
                settings.remoteBeeps = !settings.remoteBeeps;
                process = true;
                break;
            case SCREEN_SETT_GAS_SENSOR:
                settings.gasSensor = !settings.gasSensor;
                setRelays();
                process = true;
                break;
            case SCREEN_SETT_GEIG_CLICKS:
                settings.geigerClicks = !settings.geigerClicks;
                setRelays();
                states.pushSync = true;
                process = true;
                break;
            case SCREEN_SETT_GEIG_ALARM:
                settings.geigerAlarm = !settings.geigerAlarm;
                process = true;
                break;
            case SCREEN_SETT_GEIG_SENSITIVITY:
                settings.geigerSensitivity += STP_GEIG_SENSITIVITY;
                if (settings.geigerSensitivity > MAX_GEIG_SENSITIVITY)
                    settings.geigerSensitivity = MAX_GEIG_SENSITIVITY;
                process = true;
                break;
            case SCREEN_SETT_INT_MEASURE:
                settings.intervalMeasure += STP_INT_MEASURE;
                if (settings.intervalMeasure > MAX_INT_MEASURE)
                    settings.intervalMeasure = MAX_INT_MEASURE;
                process = true;
                break;
            case SCREEN_SETT_INT_PUSH:
                settings.intervalPush += STP_INT_PUSH;
                if (settings.intervalPush > MAX_INT_PUSH)
                    settings.intervalPush = MAX_INT_PUSH;
                process = true;
                break;
            case SCREEN_SETT_INT_PULL:
                settings.intervalPull += STP_INT_PULL;
                if (settings.intervalPull > MAX_INT_PULL)
                    settings.intervalPull = MAX_INT_PULL;
                process = true;
                break;
            case SCREEN_SETT_INT_SYNC:
                settings.intervalSync += STP_INT_SYNC;
                if (settings.intervalSync > MAX_INT_SYNC)
                    settings.intervalSync = MAX_INT_SYNC;
                process = true;
                break;
            case SCREEN_SETT_WIFI:
                if (settings.wifi >= COUNT_WIFI)
                    settings.wifi = 0;
                else    
                    settings.wifi++;
                process = true;
                break;      
            case SCREEN_SETT_DATETIME: {
                RtcDateTime newTime;
                switch (states.datePage) {
                    case DATETIME_HOURS:
                        newTime = now + 1 * 60 * 60;
                        break;
                    case DATETIME_MINUTES:
                        newTime = now + 1 * 60;
                        break;
                    case DATETIME_SECONDS:
                        newTime = now + 1;
                        break;
                    case DATETIME_YEAR:
                        if (now.Year() < 2100)
                            newTime = RtcDateTime(now.Year() + 1, now.Month(), now.Day(), now.Hour(), now.Minute(), now.Second());
                        break;
                    case DATETIME_MONTH:
                        if (now.Month() < 12)
                            newTime = RtcDateTime(now.Year(), now.Month() + 1, now.Day(), now.Hour(), now.Minute(), now.Second());
                        else    
                            newTime = RtcDateTime(now.Year() + 1, 1, now.Day(), now.Hour(), now.Minute(), now.Second());
                        break;
                    case DATETIME_DAY:
                        newTime = now + 1 * 60 * 60 * 24;
                        break;
                }
                rtc.SetDateTime(newTime);
                now = rtc.GetDateTime();
                resetLCD();
                break;
            }   
        }
        if (process) {
            resetLCD();
            saveSettings();           
        }
    }
}

/* =======
 *  Beeps
 * ======= */

void beepRemote() {
    if (settings.remoteBeeps) {
        tone(PIN_BUZZER, BUZZER_REMOTE_TONE, BUZZER_REMOTE_DURATION);
    }
}
  
void setup() {
    Serial.begin(115200);
    
    lcd.init();
    lcd.setBacklight(true);
    
    drawScreen(" == Alicorn ==", "  Starting...");
    
    setupSettings();
    setupStates();
    setupCounters();
    setupWiFi();
    setupDevices(); 
    setupClock();
    resetAverage();
    connectWiFi();
    
    drawScreen("", "");

    setupTimer();
}
  
void loop() {    
    
    processRemote();
    processSensors();
    processGeiger();
    processSync();
    processPush();
    if (settings.lcdBacklight) {
        processPull();
        processClock();
        processLCD();        
    }
          
    delay(INTERVAL_CYCLE);
}
