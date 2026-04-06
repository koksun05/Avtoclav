#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_PCF8574.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>

const char* ssid = "Hata";
const char* password = "12345678";

#define BUZZER    15       //d8 esp8266
#define RELAY_PIN 16       //D0
#define MENU_BTN  14       //D5 
#define PLUS_BTN  12       //D6
#define MINUS_BTN 13       //D7

LiquidCrystal_PCF8574 lcd(0x27);
byte gradus[] = { 0b01100, 0b10010, 0b10010, 0b01100, //символ градуса
                 0b00000, 0b00000, 0b00000, 0b00000 };
Adafruit_ADS1115 ads;
const float DIVIDER_RATIO = 131.0f;    //для напруги
const float K_P = 0.4895;          // коефіціент для тиску
#define CT_RATIO 1000.0           // коефіцієнт трансформатора струму (підлаштуй)
#define R_BURDEN 106.0            // Ом (резистор вимірювання струму вторинки)
#define LSB_GAIN_ONE 0.0625e-3f   // дільник для ads1115
float voltage = 0.0f;             //  напруга
float tenCurrent = 0.0f;          //  струм
float tenPower = 0.0f;            // потужність
float total_kWh = 0.0f;          // загальна енергія
uint8_t setTemp = 120;           // встановлена температура
float t_fakt = 0.0;              // фактична температура
float P = 0.0;                   // тиск
bool U_low = false;              
bool tenErr = false;
bool  P_err = false;
bool  t_err = false;
int minU = 165; 
int menu = 0;
float  hister = 0.1;
// прапорці та змінні для біпера
bool beepActiveFlag = false;
unsigned long beepTimer = 0;
int beepCount = 0;
int beepPhase = 0;
//----------------------------
bool timerPausedByLowU = false;   // прапор паузи таймера через низьку напругу
int saved_h = 0;
int saved_m = 0;
int saved_s = 0;

unsigned long menu2_lastActivity = 0;
const unsigned long MENU2_TIMEOUT = 10000; // 10 сек

int editField = 0; // перемикач для налаштування в режимі меню2. 0 = hours, 1 = minutes
//------------------------
int startHours   = 0;   // стартовий час таймера
int startMinutes = 1;   // 
int startSeconds = 30;
  
bool timer_flag = false;     // Прапорець запуску відліку таймера
bool work_flag = false;      // Прапорець запуску відліку часу роботи
unsigned long lastUpdate = 0;
unsigned long lastWorkUpdate = 0;
unsigned long lastBlink  = 0;
bool colonVisible = true; // для миготіння двокрапки

// Поточний час таймера
int h, m, s; 
int h_w, m_w, s_w;

// ------------ Button timing params -------------
const unsigned long MENU_HOLD_MS = 1000UL;   // довге меню — 1 сек
const unsigned long REPEAT_DELAY = 400UL;    // затримка перед автоповтором (ms)
const unsigned long REPEAT_INTERVAL = 100UL; // інтервал автоповтору (ms)

bool menuLast = HIGH;
unsigned long menuPressTime = 0;
bool menuLongFired = false;

bool plusLast = HIGH;
unsigned long plusPressTime = 0;
bool plusRepeatActive = false;
unsigned long plusLastRepeat = 0;

bool minusLast = HIGH;
unsigned long minusPressTime = 0;
bool minusRepeatActive = false;
unsigned long minusLastRepeat = 0;

// EEPROM addresses
const int EEPROM_SIZE = 128;
const int ADDR_setTemp = 0;    // 1 byte
const int ADDR_tHours  = 1;    // 1 byte
const int ADDR_tMins   = 2;    // 1 byte
const int ADDR_totalKWh = 4;   // float (4 bytes) put at 4..7
const int ADDR_savedH = 8;  
const int ADDR_savedM = 9;  
const int ADDR_savedS = 10; 
const int ADDR_timerPaused = 11; 


float readTempA3(int samples = 8);
float readVoltageA2(int samples = 8);
float readCurrentA0(int samples = 8);
void readADS_andCalc();
float readPA1(int samples = 8);
void displayMain();
void display_2();
void buttons();
void loadSettingsFromEEPROM();
void saveSettingsToEEPROM();


//---------------------------------------- читання тиску з А1 - 1115
float readPA1(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += ads.readADC_SingleEnded(1);

  }
  float raw = (float)sum / (float)samples;
  float volt = raw * 0.0000625;
  float press = 2.4138 * volt - 2.3897;
    return press;
}
//----------------------------------------читання температури з А3 - 1115
float readTempA3(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += ads.readADC_SingleEnded(3);
  }
  float raw = (float)sum / (float)samples;
float volt = raw * 0.0000625;
  if (volt >= 3.29) return -1000;
  float Rpt = 250.0 * volt / (3.3 - volt);
  float temp = (Rpt - 50.0) / (50.0 * 0.00385);
  return temp;
}
//------------------------------------------Читання напруги з входу А2 - 1115
float readVoltageA2(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += ads.readADC_SingleEnded(2);
    //delay(2);
  }
  float raw = (float)sum / (float)samples;
  float voltADC = raw * LSB_GAIN_ONE; 
  float mains = voltADC * DIVIDER_RATIO;
  return mains;
}
//------------------------------------------ Читання струму з входу А0 -1115
float readCurrentA0(int samples) {
  double sumSq = 0;
  for (int i = 0; i < samples; ++i) {
    int16_t raw = ads.readADC_SingleEnded(0);
    float v = raw * LSB_GAIN_ONE;
    sumSq += (v * v);
  }
  float vRMS = sqrt(sumSq / samples);
  float I_secondary = vRMS / R_BURDEN;
  float I_primary = I_secondary * CT_RATIO;
  return I_primary;
}
// ---------------- EEPROM helpers ----------------
void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t tmpSet = EEPROM.read(ADDR_setTemp);
  if (tmpSet != 0xFF) { // if previously written
    setTemp = tmpSet;
  }
  uint8_t th = EEPROM.read(ADDR_tHours);
  if (th != 0xFF) h = th; else h = startHours;
  uint8_t tm = EEPROM.read(ADDR_tMins);
  if (tm != 0xFF) m = tm; else m = startMinutes;
  float kwh = 0.0f;
  EEPROM.get(ADDR_totalKWh, kwh);
  if (!isnan(kwh) && fabs(kwh) < 1e8f) {
    total_kWh = kwh;
  }
  // ensure sensible ranges
  if (h < 0 || h > 3) h = startHours;
  if (m < 0 || m > 59) m = startMinutes;
saved_h = EEPROM.read(ADDR_savedH);               // відновлення таймера з памяті
saved_m = EEPROM.read(ADDR_savedM);
saved_s = EEPROM.read(ADDR_savedS);
timerPausedByLowU = EEPROM.read(ADDR_timerPaused) == 1;

}

void saveSettingsToEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(ADDR_setTemp, setTemp);
  EEPROM.write(ADDR_tHours, (uint8_t)h);
  EEPROM.write(ADDR_tMins, (uint8_t)m);
  EEPROM.put(ADDR_totalKWh, total_kWh);
  EEPROM.write(ADDR_savedH, saved_h);             //збереження таймера
  EEPROM.write(ADDR_savedM, saved_m);        
  EEPROM.write(ADDR_savedS, saved_s);
  EEPROM.write(ADDR_timerPaused, timerPausedByLowU ? 1 : 0);
  EEPROM.commit();
}
//---------------------------------------------------------------------
void buttons() {
  unsigned long now = millis();
  // read pins (buttons are active LOW with INPUT_PULLUP)
  bool menuBtn = digitalRead(MENU_BTN);
  bool plusBtn = digitalRead(PLUS_BTN);
  bool minusBtn = digitalRead(MINUS_BTN);

  // ---------------- MENU button handling ----------------
  if (menuBtn == LOW && menuLast == HIGH) {
    // just pressed
    menuPressTime = now;
    menuLongFired = false;
  }
  // якщо утримувати довше ніж MENU_HOLD_MS
  if (menuBtn == LOW && !menuLongFired && (now - menuPressTime >= MENU_HOLD_MS)) {
    // спрацювало довге натискання 
    menuLongFired = true;
    work_flag = false;
    P_err = false;
    
    if (menu == 0) {
      // enter menu2
      menu = 1;
      menu2_lastActivity = now;
      editField = 0; // start with hours
      // ensure h/m in allowed ranges
      if (h < 0 || h > 3) h = h % 4;
      if (m < 0 || m > 59) m = m % 60;
    } else {
      // exit menu2 -> save and set seconds to 0
      menu = 0;
      s = 0;
      saveSettingsToEEPROM();
    }
  }

  // -------------------------------------------------------короткеМеню-пуск стоп
  if (menuBtn == HIGH && menuLast == LOW) {
    if (!menuLongFired) {
      // short press action
      if (menu == 0){
        if(P > 0.9 && P < 1.5){work_flag = !work_flag;}
           else {P_err = true;work_flag = false;}
      } else {
        // ------------------------------------------------коротке в режимі меню2 уст часи, минути.
        editField = (editField == 0) ? 1 : 0;
        menu2_lastActivity = now;
      }
    }
    // reset long fired flag on release
    menuLongFired = false;
  }
  menuLast = menuBtn;

  // ---------------- PLUS button handling ----------------
  if (plusBtn == LOW && plusLast == HIGH) {
    // just pressed
    plusPressTime = now;
    plusRepeatActive = false;
  }

  // start repeat after delay
  if (plusBtn == LOW && !plusRepeatActive && (now - plusPressTime >= REPEAT_DELAY)) {
    plusRepeatActive = true;
    plusLastRepeat = now;
    // do first repeat step immediately
    if (menu == 1) {
      // in menu2 editing
      if (editField == 0) { // hours 0..3
        h = (h + 1) % 4;
      } else { // minutes 0..59
        m = (m + 1) % 60;
      }
      menu2_lastActivity = now;
    } else {
      // main menu: change setTemp
      if (setTemp < 125) { setTemp++; }
      //saveSettingsToEEPROM();
    }
  }

  // ongoing repeat steps
  if (plusBtn == LOW && plusRepeatActive && (now - plusLastRepeat >= REPEAT_INTERVAL)) {
    plusLastRepeat = now;
    if (menu == 1) {
      if (editField == 0) h = (h + 1) % 4;
      else m = (m + 1) % 60;
      menu2_lastActivity = now;
    } else {
      if (setTemp < 125) { setTemp++; }
      //saveSettingsToEEPROM();
    }
  }

  // on release: if repeat never started -> single increment
  if (plusBtn == HIGH && plusLast == LOW) {
    if (!plusRepeatActive) {
      // single step on release
      if (menu == 1) {
        if (editField == 0) { h = (h + 1) % 4; }
        else { m = (m + 1) % 60; }
        menu2_lastActivity = now;
      } else {
        if (setTemp < 125) { setTemp++; }
        //saveSettingsToEEPROM();
      }
    }
    plusRepeatActive = false;
  }
  plusLast = plusBtn;

  // ---------------- MINUS button handling ----------------
  if (minusBtn == LOW && minusLast == HIGH) {
    minusPressTime = now;
    minusRepeatActive = false;
  }

  // start repeat after delay
  if (minusBtn == LOW && !minusRepeatActive && (now - minusPressTime >= REPEAT_DELAY)) {
    minusRepeatActive = true;
    minusLastRepeat = now;
    // do first repeat step immediately
    if (menu == 1) {
      if (editField == 0) { h = (h + 3) % 4; } // -1 mod 4
      else { m = (m + 59) % 60; }
      menu2_lastActivity = now;
    } else {
      if (setTemp > 30) { setTemp--; }
      //saveSettingsToEEPROM();
    }
  }

  // ongoing repeat steps
  if (minusBtn == LOW && minusRepeatActive && (now - minusLastRepeat >= REPEAT_INTERVAL)) {
    minusLastRepeat = now;
    if (menu == 1) {
      if (editField == 0) { h = (h + 3) % 4; }
      else { m = (m + 59) % 60; }
      menu2_lastActivity = now;
    } else {
      if (setTemp > 30) { setTemp--; }
      saveSettingsToEEPROM();
    }
  }

  // on release: if repeat never started -> single decrement
  if (minusBtn == HIGH && minusLast == LOW) {
    if (!minusRepeatActive) {
      if (menu == 1) {
        if (editField == 0) { h = (h + 3) % 4; }
        else { m = (m + 59) % 60; }
        menu2_lastActivity = now;
      } else {
        if (setTemp > 30) { setTemp--; }
        //saveSettingsToEEPROM();
      }
    }
    minusRepeatActive = false;
  }

  minusLast = minusBtn;
  // ---------------- auto-exit menu2 on inactivity ----------------
  if (menu == 1 && (now - menu2_lastActivity > MENU2_TIMEOUT)) {
    // exit to main
    menu = 0;
    s = 0;
    saveSettingsToEEPROM();
  }
}
//------------------------------------------------------------------------------

//---------------------------------------------------------------------------
void readADS_andCalc() {
   t_fakt = readTempA3(15);
  voltage = readVoltageA2(12);
  tenCurrent = readCurrentA0(15);
  if (fabs(tenCurrent) < 0.00005f) tenCurrent = 0.0f;
  tenPower = voltage * fabs(tenCurrent);
  U_low = (voltage < (float)minU);
  P= readPA1(15);
  // ---- контроль ТЕНа з антифальш захистом ----
  static byte lowCurrentCount = 0;
  if (digitalRead(RELAY_PIN) == HIGH) {
    if (tenCurrent < 0.2f) {
      if (lowCurrentCount++ > 30) {tenErr = true;work_flag = false;}  // 30 раз поспіль
    } else {
      lowCurrentCount = 0;
      tenErr = false;
    }
  } else {
    lowCurrentCount = 0;
    tenErr = false;
  }

  // ---- контроль тиску  ----
  static byte pCount = 0;
  if (work_flag) {
    if (P < 0.9 || P > 4.0) {
      if (pCount++ > 30) {P_err = true;work_flag = false;}  // 30 раз поспіль
    } else {
      pCount = 0;
      P_err = false;
    }
  } else {
    pCount = 0;
  }
}
//--------------------------------------------------------------------------------------------------

void displayMain(){
   lcd.clear();
      lcd.setCursor(0, 0);
    lcd.print("Set=");
    lcd.print (setTemp);
    lcd.print("\01C "); lcd.setCursor(11, 0); lcd.print("t\01= ");lcd.print(t_fakt,1);
    lcd.setCursor(0,1);
    lcd.print ("Time ");
   

   // Формування виводу
   // --------------------------------------------------
   lcd.setCursor(6, 1);

   lcd.print( (h < 10 ? "0" : "") );
   lcd.print(h);

   lcd.print(colonVisible ? ":" : " ");

   lcd.print( (m < 10 ? "0" : "") );
   lcd.print(m);

   lcd.print(colonVisible ? ":" : " ");

   lcd.print( (s < 10 ? "0" : "") );
   lcd.print(s);
  if (!timer_flag && h==0 && m==0 && s==0) {
    timer_flag = false; work_flag =false;
     lcd.setCursor(17, 1); lcd.print("End");}
     lcd.setCursor(0,2);
    if(work_flag && !U_low){ lcd.print ("work ");}
    else if (U_low){lcd.print ("U_low");}
    else lcd.print ("     ");

   lcd.setCursor(6, 2);
    lcd.print( (h_w < 10 ? "0" : "") );
   lcd.print(h_w);
   lcd.print(colonVisible ? ":" : " ");
   lcd.print( (m_w < 10 ? "0" : "") );
   lcd.print(m_w);
   lcd.print(colonVisible ? ":" : " ");
   lcd.print( (s_w < 10 ? "0" : "") );
   lcd.print(s_w);

   lcd.setCursor(0,3);lcd.print("P=");
   if(!P_err){lcd.print(P,2);}
   else  {lcd.print("ERR");}
   lcd.setCursor(7,3);
   if (!tenErr){lcd.print("I=");lcd.print(tenCurrent,1);}
   else lcd.print("TEN-ER");
   lcd.setCursor(14, 3); lcd.print("U=");lcd.print(voltage,0);

}
void display_2(){
   lcd.clear();
    lcd.print("Set-Time");
     lcd.setCursor(9, 0);
 if (editField == 0) lcd.print(">"); else lcd.print(" ");
   lcd.setCursor(10, 0);
   // show only hours:minutes (no seconds)
   if (h < 10) lcd.print('0');
   lcd.print(h);
   lcd.print(colonVisible ? ":" : " ");
   if (m < 10) lcd.print('0');
   lcd.print(m);
 if (editField == 1) lcd.print("<"); else lcd.print(" ");
   lcd.setCursor(0,2);
   lcd.print("Energy = ");lcd.print(total_kWh,1);
   lcd.setCursor(0,3);
  lcd.print("Long:exit  Short:fld");
}
void startBeep(int times){
    // показ лише для дебагу, можна видалити
   lcd.setCursor(16, 2); lcd.print("ALL ");
   beepActiveFlag = true;
   beepCount = times;
   beepPhase = 0;        // 0 = включити, 1 = вимкнути
   beepTimer = millis();
}

void setup() {
    Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  ArduinoOTA.begin();
  Serial.println("Ready for OTA");
  Serial.println(WiFi.localIP());
  pinMode(MENU_BTN, INPUT_PULLUP);
  pinMode(PLUS_BTN, INPUT_PULLUP);
  pinMode(MINUS_BTN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  h_w = 0;   //змінні для часу Роботи 
  m_w = 0;
  s_w = 0;
  // load saved settings
  loadSettingsFromEEPROM();
    // Перевіряємо, чи таймер був призупинений
    if (timerPausedByLowU && (saved_h > 0 || saved_m > 0 || saved_s > 0)) {
        // Відновлюємо таймер
         if (t_fakt >= setTemp - 10) {
        // температура нормальна — продовжуємо зі збереженого часу
        h = saved_h;
        m = saved_m;
        s = saved_s;
        timer_flag = true;
        work_flag = true;
            } else {
             // температура впала — починаємо таймер спочатку
             h = startHours;
             m = startMinutes;
             s = startSeconds;
             }
        timer_flag = true;
        work_flag = true;
    } else {
        // Інакше стартуємо з дефолтного часу і запуск з кнопки
        h = startHours;
        m = startMinutes;
        s = startSeconds;
        timer_flag = false;
        work_flag = false;
    }
  Wire.begin();
  ads.begin();
 ads.setGain(GAIN_TWO);
 ads.setDataRate(RATE_ADS1115_128SPS);
 delay(50);
  Wire.beginTransmission(0x27);
    lcd.begin(20, 4);
    lcd.createChar(1, gradus);
    lcd.setBacklight(255);
    lcd.home();
    lcd.clear();
    lcd.setCursor(5, 1);
    lcd.print("Hello-Chef");
    delay(2000);
    lcd.clear();
}

void loop() {
   ArduinoOTA.handle();
  buttons();
if(work_flag){
 if (P < 0.9 || P > 4.0){work_flag = false; P_err = true; }
  if(t_fakt >= setTemp - hister ){
    timer_flag = true;
    digitalWrite(RELAY_PIN,LOW);
  } else if(t_fakt <setTemp - hister) {
    timer_flag = false;lcd.setCursor(17, 1); lcd.print("TEN");
    digitalWrite(RELAY_PIN,HIGH);
  }
  if (millis() - lastWorkUpdate >= 1000) {
    lastWorkUpdate = millis();

    // Збільшуємо час роботи
    if (s_w < 59) s_w++;
    else {
      s_w = 0;
      if (m_w < 59) m_w++;
      else {
        m_w = 0;
        if (h_w < 23) h_w++;
        else { h_w = 0; }
      }
    }
  }
}
  //-----------------------------------------
  if (timer_flag && (millis() - lastUpdate >= 1000)) {
    lastUpdate = millis();

    // Зменшуємо час таймера
    if (s > 0) s--;
    else {
      s = 59;
      if (m > 0) m--;
      else {
        m = 59;
        if (h > 0) h--;
        else {
                                      // Таймер закінчився
          h = m = s = 0;
    saved_h = 0;                                       //
    saved_m = 0;                                       //
    saved_s = 0;                                       //
    timerPausedByLowU = false;                         //
    saveSettingsToEEPROM();
          timer_flag = false; // зупиняємо відлік
          work_flag = false;
        }
      }
    }
  }
  // Миготіння двокрапки тільки під час відліку
  if (timer_flag && millis() - lastBlink >= 500) {
    lastBlink = millis();
    colonVisible = !colonVisible;
  } else if (!timer_flag) {
    colonVisible = true; // коли не відлік — двокрапки не мигають
  }

  readADS_andCalc();
   if (U_low && timer_flag) {
    // Зберігаємо поточний час таймера
    saved_h = h;
    saved_m = m;
    saved_s = s;
    timerPausedByLowU = true;
    saveSettingsToEEPROM();

    timer_flag = false;
    work_flag = false;
   }
   if (timerPausedByLowU && !U_low){
       if (t_fakt >= setTemp - 10) {
        // температура нормальна — продовжуємо
        h = saved_h;
        m = saved_m;
        s = saved_s;
        timer_flag = true;
        work_flag = true;
            } else {
             // температура впала — починаємо таймер спочатку
             h = startHours;
             m = startMinutes;
             s = startSeconds;
             }
        timer_flag = true;
        work_flag = true;
        timerPausedByLowU = false;
    }
   static uint32_t disp = 0;
   if(millis() - disp >= 300){
    disp = millis();
  if (menu == 0) { displayMain(); }
  else if (menu == 1) { display_2(); }
   }
 
 //біпер
  if (tenErr && !beepActiveFlag) {
  startBeep(3);
            }
    if (P_err && !beepActiveFlag) {
  startBeep(1);
            }

// НЕ блокуючий обробник біпера — виконується завжди, поки активний
  if (beepActiveFlag) {
  unsigned long now = millis();
  if (beepPhase == 0) {                 // фаза "вмикання"
    digitalWrite(BUZZER, HIGH);
    if (now - beepTimer >= 300) {       // 300 мс ON
      beepPhase = 1;
      beepTimer = now;
    }
  }
  else if (beepPhase == 1) {            // фаза "вимикання"
    digitalWrite(BUZZER, LOW);
    if (now - beepTimer >= 200) {       // 200 мс OFF
      beepCount--;
      if (beepCount <= 0) {
        beepActiveFlag = false;         // кінець сигналів
        digitalWrite(BUZZER, LOW);      // впевнитись що вимкнено
      } else {
        beepPhase = 0;                  // наступний цикл
      }
      beepTimer = now;
    }
  }
  }
}
