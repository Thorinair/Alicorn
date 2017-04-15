#include "ESP8266WiFi.h"
#include <EEPROM.h>
#include <VariPass.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremoteESP8266.h>
#include <SFE_BMP180.h>
//#include <MQ135.h>

#include "wifi.h"
#include "vpid.h"

extern "C" {
    #include "user_interface.h"
}

// Pins
//#define PIN_BUTTON_FACTORY 0
#define PIN_RELAY_GEIG     0
#define PIN_BUZZER_ALARM   2
#define PIN_DHT            12
#define PIN_IR             14
#define PIN_BUZZER_REMOTE  15
#define PIN_RELAY_GAS      16
#define PIN_MQ             A0
//#define PIN_LED            3

// I2C Addresses
#define I2C_LCD 0x27
#define I2C_RTC 0x57
#define I2C_BMP 0x77

// Default Settings
#define DEFAULT_LCD_BACKLIGHT    1
#define DEFAULT_REMOTE_BEEPS     1
#define DEFAULT_GAS_SENSOR       1
#define DEFAULT_GEIG_CLICKS      1
#define DEFAULT_GEIG_WARNING     1
#define DEFAULT_GEIG_SENSITIVITY 10
#define DEFAULT_INT_MEASURE      10000
#define DEFAULT_INT_GEIGER       60000
#define DEFAULT_INT_PUSH         60000
#define DEFAULT_INT_PULL         1000
#define DEFAULT_WIFI             0

// MinMaxStep Settings
#define MIN_GEIG_SENSITIVITY 10
#define MIN_INT_MEASURE      2000
#define MIN_INT_GEIGER       60000
#define MIN_INT_PUSH         1000
#define MIN_INT_PULL         1000

#define MAX_GEIG_SENSITIVITY 1000
#define MAX_INT_MEASURE      60000
#define MAX_INT_GEIGER       3600000
#define MAX_INT_PUSH         60000
#define MAX_INT_PULL         60000

#define STP_GEIG_SENSITIVITY 10
#define STP_INT_MEASURE      1000
#define STP_INT_GEIGER       60000
#define STP_INT_PUSH         1000
#define STP_INT_PULL         1000

// EEPROM Addresses
#define EEPROM_SAVED            0
#define EEPROM_LCD_BACKLIGHT    1
#define EEPROM_REMOTE_BEEPS     2
#define EEPROM_GAS_SENSOR       3
#define EEPROM_GEIG_CLICKS      4
#define EEPROM_GEIG_WARNING     5
#define EEPROM_GEIG_SENSITIVITY 6
#define EEPROM_INT_MEASURE      7
#define EEPROM_INT_GEIGER       8
#define EEPROM_INT_PUSH         9
#define EEPROM_INT_PULL         10
#define EEPROM_WIFI             11

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
#define SCREEN_SETT_GEIG_WARNING     3
#define SCREEN_SETT_GEIG_SENSITIVITY 4
#define SCREEN_SETT_INT_MEASURE      5
#define SCREEN_SETT_INT_GEIGER       6
#define SCREEN_SETT_INT_PUSH         7
#define SCREEN_SETT_INT_PULL         8
#define SCREEN_SETT_WIFI             9
#define SCREEN_SETT_TIME             10


// Intervals
#define INTERVAL_CYCLE 100
#define INTERVAL_TIMER 100
#define INTERVAL_LCD   1000
#define INTERVAL_CLOCK 1000

// Compact Levels
#define COMPACT_NONE   0
#define COMPACT_LONG   1
#define COMPACT_MEDIUM 2
#define COMPACT_SHORT  3

// Buzzer
#define BUZZER_TONE     3000
#define BUZZER_DURATION 100

// DHT22
DHT dht(PIN_DHT, DHT22);

// BMP180
#define ALTITUDE 123
SFE_BMP180 bmp;

// MQ135
//MQ135 mq(PIN_MQ);

// RTC

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

// Timer
os_timer_t timer;

// Structures
struct SETTINGS {
    bool lcdBacklight;
    bool remoteBeeps;
    bool gasSensor;
    bool geigerClicks;
    bool geigerWarning;
    int  geigerSensitivity;
    int  intervalMeasure;
    long intervalGeiger;
    int  intervalPush;
    int  intervalPull;
    int  wifi;
} settings;

struct STATES {
    bool screenSettings;
    int  screenPage;
    int  wifi;
} states;

struct COUNTERS {
    int measure;
    int cclock;
    int push;
    int pull;
    int lcd;
} counters;

struct INTERVALS {
    bool measure;
    bool cclock;
    bool push;
    bool pull;
    bool lcd;
} intervals;

struct DATA {
    float  temperature;
    float  humidity;
    float  pressure;
    float  gas;
    int    cpm;
    float  dose;
    long   core;
    long   gain;
    String bulletin;
} data;

struct AVERAGE {
    float  temperature;
    float  humidity;
    float  pressure;
    float  gas;
} average;

int averageCount;

// Utilities
//void factoryReset();
void resetLCD();
void setRelays();
void drawScreen(String top, String bot);
bool checkInterval(int *counter, int interval);
void resetAverage();

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
void processRemote();
void processSensors();
void processClock();
void processPush();
void processPull();
void processLCD();

// VariPass
void pullVariPass();
void pushVariPass();

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

/*
void factoryReset() {
    Serial.println("Factory reset initiated. Restarting...");
    EEPROM.begin(512);
    EEPROM.write(EEPROM_SAVED, 0);
    EEPROM.end();
    ESP.restart();
}
*/

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
    
    averageCount        = 0;
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
        settings.remoteBeeps       = DEFAULT_REMOTE_BEEPS;
        settings.gasSensor         = DEFAULT_GAS_SENSOR;
        settings.geigerClicks      = DEFAULT_GEIG_CLICKS;
        settings.geigerWarning     = DEFAULT_GEIG_WARNING;
        settings.geigerSensitivity = DEFAULT_GEIG_SENSITIVITY;
        settings.intervalMeasure   = DEFAULT_INT_MEASURE;
        settings.intervalGeiger    = DEFAULT_INT_GEIGER;
        settings.intervalPush      = DEFAULT_INT_PUSH;
        settings.intervalPull      = DEFAULT_INT_PULL;
        settings.wifi              = DEFAULT_WIFI;
        
        Serial.println("\nCreated new settings. Saving...");
        saveSettings();
    }

    lcd.setBacklight(settings.lcdBacklight);
}

void setupStates() {
    states.screenSettings = false;
    states.screenPage     = 0;
    states.wifi           = settings.wifi;
}

void setupCounters() {
    counters.measure = 0;
    counters.push    = 0;
    counters.pull    = 0;
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
        
    //attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_FACTORY), factoryReset, FALLING);
    pinMode(PIN_RELAY_GEIG, OUTPUT);
    pinMode(PIN_RELAY_GAS,  OUTPUT);
    
    setRelays();
}

void setupClock() {
    
}

void setupTimer() {
    os_timer_setfn(&timer, processTimer, NULL);
    os_timer_arm(&timer, INTERVAL_TIMER, true);
}

/* ==========
 *  Settings 
 * ========== */

void saveSettings() {
    EEPROM.begin(512);
    EEPROM.write(EEPROM_SAVED,            1);
    EEPROM.write(EEPROM_LCD_BACKLIGHT,    settings.lcdBacklight);
    EEPROM.write(EEPROM_REMOTE_BEEPS,     settings.remoteBeeps);
    EEPROM.write(EEPROM_GAS_SENSOR,       settings.gasSensor);
    EEPROM.write(EEPROM_GEIG_CLICKS,      settings.geigerClicks);
    EEPROM.write(EEPROM_GEIG_WARNING,     settings.geigerWarning);
    EEPROM.write(EEPROM_GEIG_SENSITIVITY, settings.geigerSensitivity);
    EEPROM.write(EEPROM_INT_MEASURE,      settings.intervalMeasure / 1000);
    EEPROM.write(EEPROM_INT_GEIGER,       settings.intervalGeiger  / 60000);
    EEPROM.write(EEPROM_INT_PUSH,         settings.intervalPush    / 1000);
    EEPROM.write(EEPROM_INT_PULL,         settings.intervalPull    / 1000);
    EEPROM.write(EEPROM_WIFI,             settings.wifi);
    EEPROM.end();
    Serial.println("Saved settings to EEPROM.");
}

void loadSettings() {
    EEPROM.begin(512);
    settings.lcdBacklight      = (bool) EEPROM.read(EEPROM_LCD_BACKLIGHT);
    settings.remoteBeeps       = (bool) EEPROM.read(EEPROM_REMOTE_BEEPS);
    settings.gasSensor         = (bool) EEPROM.read(EEPROM_GAS_SENSOR);
    settings.geigerClicks      = (bool) EEPROM.read(EEPROM_GEIG_CLICKS);
    settings.geigerWarning     = (bool) EEPROM.read(EEPROM_GEIG_WARNING);
    settings.geigerSensitivity = (int)  EEPROM.read(EEPROM_GEIG_SENSITIVITY);
    settings.intervalMeasure   = (int)  EEPROM.read(EEPROM_INT_MEASURE) * 1000;
    settings.intervalGeiger    = (long) EEPROM.read(EEPROM_INT_GEIGER)  * 60000;
    settings.intervalPush      = (int)  EEPROM.read(EEPROM_INT_PUSH)    * 1000;
    settings.intervalPull      = (int)  EEPROM.read(EEPROM_INT_PULL)    * 1000;
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
    WiFi.begin(ssid[settings.wifi], pass[settings.wifi]);
    /*
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi: '" + (String) ssid[settings.wifi] + "' With IP: " + WiFi.localIP());  
    */
}

/* ===========
 *  Processes
 * =========== */

void processTimer(void *pArg) {
    //Serial.println("Timer tick!");
    if (checkInterval(&counters.measure, settings.intervalMeasure))
        intervals.measure = true;

    if (checkInterval(&counters.cclock, INTERVAL_CLOCK))
        intervals.cclock = true;

    if (checkInterval(&counters.push, settings.intervalPush))
        intervals.push = true;
        
    if (checkInterval(&counters.pull, settings.intervalPush))
        intervals.pull = true;

    if (checkInterval(&counters.lcd, INTERVAL_LCD))
        intervals.lcd = true;    
        
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
                            states.screenPage = 0;
                            resetLCD();
                            if (states.wifi != settings.wifi)
                                ESP.restart();
                            // Serial.println("Settings pressed, now set to " + String(states.screenSettings));
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
        averageCount++;
        
        float value;
        // Serial.println("Measure interval!");
        value = dht.readTemperature();
        if (String(value) != "nan") {
            data.temperature     = value; 
            average.temperature += value;           
        }
            
        value = dht.readHumidity();
        if (String(value) != "nan") {
            data.humidity     = value;
            average.humidity += value;            
        }

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
                        value = (float) bmp.sealevel(pres, ALTITUDE);
                        if (String(value) != "nan")
                            data.pressure     = value;
                            average.pressure += value; 
                    }
                }
            }
        }

        //value = mq.getCorrectedPPM(data.temperature, data.humidity);
        value = 0;
        if (settings.gasSensor)
            value = ((1023 - (float) analogRead(PIN_MQ)) / 1023) * 100;
        data.gas     = value;
        average.gas += value; 
        
        //Serial.println("Temperature: " + String(data.temperature)); 
        //Serial.println("Humidity:    " + String(data.humidity)); 
    }
}

void processClock() {
    if (intervals.cclock) {
        intervals.cclock = false;
        
    }
}

void processPush() {
    if (intervals.push) {
        intervals.push = false;
        
        // Serial.println("Push interval!");
        pushVariPass();
    }
}

void processPull() {
    if (intervals.pull) {
        intervals.pull = false;
        
        // Serial.println("Pull interval!");
        pullVariPass();
    }
}

void processLCD() {
    if (intervals.lcd) {
        intervals.lcd = false;
        
        if (states.screenSettings) {
            switch (states.screenPage) {
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
                case SCREEN_SETT_GEIG_WARNING:
                    drawScreen(
                        "> Geiger Warning", 
                        "Value: " + boolToOnOff(settings.geigerWarning)
                    );
                    break;
                case SCREEN_SETT_GEIG_SENSITIVITY:
                    drawScreen(
                        "> Geiger Sensit.", 
                        "Value: " + String(settings.geigerSensitivity) + " uSv"
                    );
                    break;
                case SCREEN_SETT_INT_MEASURE:
                    drawScreen(
                        "> Measure Inter.", 
                        "Value: " + String(settings.intervalMeasure / 1000) + " s"
                    );
                    break;
                case SCREEN_SETT_INT_GEIGER:
                    drawScreen(
                        "> Geiger Inter.", 
                        "Value: " + String(settings.intervalGeiger / 60000) + " min"
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
                case SCREEN_SETT_WIFI:
                    drawScreen(
                        "> WiFi Network", 
                        String(settings.wifi + 1) + ": " + ssid[settings.wifi]
                    );
                    break;
                case SCREEN_SETT_TIME:
                    drawScreen(
                        "> Date and Time", 
                        ""
                    );
                    break;
            }
        } 
        else {
            switch (states.screenPage) {
                case SCREEN_MAIN_TIME:
                    drawScreen(
                        "     00:00      ", 
                        "  05 Apr 2017   "
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
                        "CPM:  " + String(data.cpm),
                        "Dose: " + String(data.dose) + " uSv"
                    );
                    break;
                case SCREEN_MAIN_CORE:
                    drawScreen(
                        formatNumbers(data.core, COMPACT_LONG, false, true, "RF"), 
                        formatNumbers(data.gain, COMPACT_SHORT, true, true, "RF/t")
                    );
                    break;
                case SCREEN_MAIN_BULLETIN: {
                    String bullA = data.bulletin;
                    String bullB = "";                    
                    if (bullA.length() > 16)
                        bullB = bullA.substring(16);
                        
                    drawScreen(
                        bullA, 
                        bullB
                    );
                    break;                    
                }
                case SCREEN_MAIN_WIFI: {
                    String wifi = "  Disconnected";
                    if (WiFi.status() == WL_CONNECTED)
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
    if (WiFi.status() == WL_CONNECTED)
        switch (states.screenPage) {
            int result;
            case SCREEN_MAIN_CORE:
                long value1;
                
                value1 = varipassReadInt(KEY2, ID_CORE, &result);
                if (result == VARIPASS_RESULT_SUCCESS)
                    data.core = value1;
                else
                    Serial.println("An error has occured! Code: " + String(result));
                    
                value1 = varipassReadInt(KEY2, ID_GAIN, &result);
                if (result == VARIPASS_RESULT_SUCCESS)
                    data.gain = value1;
                else
                    Serial.println("An error has occured! Code: " + String(result)); 
                     
                break;
                
            case SCREEN_MAIN_BULLETIN:
                String value2;
                
                value2 = varipassReadString(KEY1, ID_BULLETIN, &result);
                if (result == VARIPASS_RESULT_SUCCESS)
                    data.bulletin = value2;
                else
                    Serial.println("An error has occured! Code: " + String(result));
                     
                break;
        }
}

void pushVariPass() {
    if (WiFi.status() == WL_CONNECTED) {
        int result;
        varipassWriteFloat(KEY1, ID_TEMPERATURE, average.temperature / averageCount, &result);
        varipassWriteFloat(KEY1, ID_HUMIDITY,    average.humidity    / averageCount, &result);
        varipassWriteFloat(KEY1, ID_PRESSURE,    average.pressure    / averageCount, &result);
        if (settings.gasSensor)
            varipassWriteFloat(KEY1, ID_GAS,     average.gas         / averageCount, &result);
    }
    resetAverage();
}

/* ========
 *  Remote
 * ======== */

void screenPrev() {
    if (settings.lcdBacklight) {
        if (states.screenPage <= 0) {
            if (states.screenSettings)
                states.screenPage = SCREENS_SETT - 1;
            else    
                states.screenPage = SCREENS_MAIN - 1;
        }
        else {
            states.screenPage--;
        }
        resetLCD();
        // Serial.println("Prev, screen set to " + String(states.screenPage));
    }
}

void screenNext() {
    if (settings.lcdBacklight) {
        if (states.screenSettings) {
            if (states.screenPage >= SCREENS_SETT - 1)
                states.screenPage = 0;
            else
                states.screenPage++;            
        }
        else {
            if (states.screenPage >= SCREENS_MAIN - 1)
                states.screenPage = 0;
            else
                states.screenPage++;  
        } 
        resetLCD();
        // Serial.println("Next, screen set to " + String(states.screenPage));
    }
}

void screenSet(int page) {
    if (settings.lcdBacklight) {
        if (states.screenSettings) {
            if (page < SCREENS_SETT)
                states.screenPage = page;  
        }
        else {
            if (page < SCREENS_MAIN)
                states.screenPage = page;
        } 
        resetLCD();
        // Serial.println("Screen set to " + String(states.screenPage));
    }
}

void valueDecrease() {
    if (states.screenSettings && settings.lcdBacklight) {
        bool process = false;
        switch (states.screenPage) {
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
                process = true;
                break;
            case SCREEN_SETT_GEIG_WARNING:
                settings.geigerWarning = !settings.geigerWarning;
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
            case SCREEN_SETT_INT_GEIGER:
                settings.intervalGeiger -= STP_INT_GEIGER;
                if (settings.intervalGeiger < MIN_INT_GEIGER)
                    settings.intervalGeiger = MIN_INT_GEIGER;
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
            case SCREEN_SETT_WIFI:
                if (settings.wifi <= 0)
                    settings.wifi = COUNT_WIFI - 1;
                else    
                    settings.wifi--;
                process = true;
                break;
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
        switch (states.screenPage) {
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
                process = true;
                break;
            case SCREEN_SETT_GEIG_WARNING:
                settings.geigerWarning = !settings.geigerWarning;
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
            case SCREEN_SETT_INT_GEIGER:
                settings.intervalGeiger += STP_INT_GEIGER;
                if (settings.intervalGeiger > MAX_INT_GEIGER)
                    settings.intervalGeiger = MAX_INT_GEIGER;
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
            case SCREEN_SETT_WIFI:
                if (settings.wifi >= COUNT_WIFI - 1)
                    settings.wifi = 0;
                else    
                    settings.wifi++;
                process = true;
                break;
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
        tone(PIN_BUZZER_REMOTE, BUZZER_TONE, BUZZER_DURATION);
    }
}
  
void setup() {
    Serial.begin(115200);
    
    lcd.init();
    lcd.setBacklight(true);
    
    drawScreen(" == Alicorn ==", "  Loading...");
    setupSettings();
    setupStates();
    setupCounters();
    setupWiFi();
    setupDevices(); 
    resetAverage();
    
    drawScreen(" == Alicorn ==", "  Connecting...");
    connectWiFi();
    
    drawScreen(" == Alicorn ==", "  Ready!");

    setupTimer();
}
  
void loop() {    
    
    processRemote();
    processSensors();
    processPush();
    if (settings.lcdBacklight) {
        processPull();
        processClock();
        processLCD();        
    }

    /*
    int button = digitalRead(BUTTON_PIN);
    if (button == HIGH) {
        //Serial.println("Button: false"); 
        varipassWriteBool(KEY, ID_LAMP, false, &result);
    }
    else {
        //Serial.println("Button: true"); 
        varipassWriteBool(KEY, ID_LAMP, true, &result);
    }

    String bulletin = varipassReadString(KEY, ID_BULL, &result);
    if (result == VARIPASS_RESULT_SUCCESS) {
        //Serial.println("Read bulletin: " + bulletin);
        lcd.clear();
        lcd.home();
        lcd.setCursor(0,0); 
        lcd.print(bulletin);
        
        if (bulletin.length() > 16) { 
            lcd.setCursor(0,1); 
            lcd.print(bulletin.substring(16));
        }
    }
    else {
        Serial.println("An error has occured! Code: " + String(result)); 
    }

    long intensity = varipassReadInt(KEY, ID_INTE, &result);
    if (result == VARIPASS_RESULT_SUCCESS) {
        bool lever = varipassReadBool(KEY, ID_LEVR, &result);
        if (result == VARIPASS_RESULT_SUCCESS) {
            if (lever) {
                int output = map(intensity, 0, 100, 0, 255);
                analogWrite(PIN_LED, output);
            }
            else {
                analogWrite(PIN_LED, 0);
            }
        }
    }
    else {
        Serial.println("An error has occured! Code: " + String(result)); 
    }
    */
          
    delay(INTERVAL_CYCLE);
}
