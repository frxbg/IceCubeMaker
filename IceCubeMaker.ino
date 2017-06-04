/*Управление за ледогенератор Bartscher ZB-02
  От Тони Стоянов
*/


#include <avr/sleep.h>
#include <math.h>
#include <EEPROM.h>

// Дефинираме входно/изходните пинове

#define K2Led        2 // ***    2
#define K1Led        3 //        3
#define K3Led        4 // CN2    4
#define Led3Anode    5 //        5
#define Led124Anode  6 // ***    6
#define WaterLevel  A1 // CN1   (S3C9454B/F9454B) PIN 16
#define ReedSensor  A2 // CN3   (S3C9454B/F9454B) PIN 19
#define NTCPin      A3 // CN4   (S3C9454B/F9454B) PIN 18
#define Fan          7 // CN7   (S3C9454B/F9454B) PIN 9
#define Compressor  10 // COM 1 (S3C9454B/F9454B) PIN 3
#define Buzzer      11 //       (S3C9454B/F9454B) PIN 5
#define HotGas      12 // COM 2 (S3C9454B/F9454B) PIN 6
#define WaterPump   13 // COM 3 (S3C9454B/F9454B) PIN 7

// Дефинираме променливите
const int CAddress = 0;
boolean lastCicle;
byte CicleSelect;
byte RunCicle;
boolean Running;
boolean FirstStart;
boolean WaterErr;
boolean FillUp;
boolean Stage1;
boolean Stage2;
boolean Stage3;
boolean Stage4;

int OutTemp;
int Selected;

// Дефинираме времевите променливи
unsigned long Time1;
unsigned long Time2;
unsigned long Time3;
unsigned long K1Time;
unsigned long K2Time;
unsigned long K3Time;
unsigned long TimerOff;
unsigned long currentMillis;
unsigned long TimeWater;
unsigned long TimerHotGas;
unsigned long WorkingTime;
unsigned long DefrostTime;
unsigned long ProgramTime;
unsigned long WaterAlarmDelay;
unsigned long TimeIce;
unsigned long CleaningTime;
unsigned long AC;



void wakeUpNow() {
  setup();
}


// *********************************************************************
// setup
// *********************************************************************
void setup() {

  // Серийна комуникация
  while (!Serial);                   // Изчакваме докато е готова
  Serial.begin(115000);              // Стартирай
  Serial.println(F("Setup"));        // Само за дебъг
  // Задаваме постоянните изходни пинове
  pinMode(Led124Anode, OUTPUT);
  pinMode(Led3Anode, OUTPUT);
  pinMode(Fan, OUTPUT);
  pinMode(Compressor, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(HotGas, OUTPUT);
  pinMode(WaterPump, OUTPUT);
  digitalWrite(Compressor, LOW);
  digitalWrite(HotGas, LOW);
  digitalWrite(WaterPump, LOW);
  digitalWrite(Fan, LOW);
  // Задаваме постоянните входни пинове
  pinMode(WaterLevel, INPUT);
  pinMode(ReedSensor, INPUT);
  Selected = 1;
  FirstStart = 1;
  TimerOff = currentMillis;
  CicleSelect = EEPROM.read(CAddress);
}


// *********************************************************************
// sleepNow
// *********************************************************************
void sleepNow()         // Тук изключваме adrduino
{
  pinMode(K2Led, INPUT_PULLUP);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // Режима на заспиване

  sleep_enable();          // Задава заспиването в mcucr register

  attachInterrupt(0, wakeUpNow, LOW); // Използваме interrupt 0 за събуждане
  // wakeUpNow когато входа е LOW

  sleep_mode();            // Тук устройството се изключва!!
  // програмата продължава от тук след събуждане
  sleep_disable();         // първто нещо след въбуждането:
  // премахва заспиването...
  detachInterrupt(0);
  delay(1000);
}


// *********************************************************************
// loop
// *********************************************************************
void loop() {

  UpdateLeds();
  // Serial.println(F("loop"));

  // *********************************************************************
  if (FirstStart == 1) { //Ако е първо стартиране премини към изчакване
    Startup();
  }
  if (Running > 0) { // Ако е пусната програмата премини към нея
    Run();
  }
  if (WaterErr == 1) { // Аларма ако е ниско нивото на водата
    WaterAlarm();
  }

  // RUN *********************************************************************
  if (Running == 0) {
    pinMode(K1Led, INPUT_PULLUP);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, LOW);
    K1Time = millis() + 300;
    while (digitalRead (K1Led) == LOW) { // && digitalRead(WaterLevel) == HIGH && digitalRead(ReedSensor) == HIGH) {
      Serial.println(digitalRead (K1Led));
      if (millis() >= K1Time) {
        TimeWater = millis() + 2500;
        Stage1 = 0;
        Stage2 = 0;
        Stage3 = 0;
        Stage4 = 0;
        Running = 1;
        RunCicle = 0;
        lastCicle = 0;
        CicleSelect = EEPROM.read(CAddress);
        pinMode(K1Led, OUTPUT);         //
        pinMode(K2Led, OUTPUT);         //
        pinMode(K3Led, OUTPUT);         //
        digitalWrite(K1Led, LOW);       //
        digitalWrite(K2Led, LOW);       //
        digitalWrite(K3Led, LOW);       //  Обратна връзка чрез светване на всичките светодиоди
        digitalWrite(Led3Anode, HIGH);  //
        digitalWrite(Led124Anode, HIGH);//
        Serial.println(F("Run"));  //
        delay(1000);                    //
        digitalWrite(Led3Anode, LOW);
        digitalWrite(Led124Anode, LOW);
        pinMode(K1Led, INPUT_PULLUP);
        pinMode(K2Led, INPUT_PULLUP);
        pinMode(K3Led, INPUT_PULLUP);
        break;
      }
    }
  }
  UpdateLeds();

  // SelfClean *********************************************************************
  if (Running == 0) {
    pinMode(K1Led, INPUT_PULLUP);
    pinMode(K2Led, INPUT_PULLUP);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, LOW);
    K2Time = millis() + 1000;
    while (digitalRead (K1Led) == LOW && digitalRead(K2Led) == LOW) {
      if (millis() >= K2Time) {
        SelfClean();
        break;
      }
    }
  }

  // SELECT *********************************************************************
  if (Running == 0) {
    digitalWrite(Compressor, LOW);
    digitalWrite(HotGas, LOW);
    digitalWrite(WaterPump, LOW);
    digitalWrite(Fan, LOW);
    pinMode(K2Led, INPUT_PULLUP);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, LOW);
    K2Time = millis() + 300;
    while (digitalRead(K2Led) == LOW) {
      Serial.println(digitalRead (K2Led));
      if (millis() >= K2Time) {
        if (Selected == 1) {
          Selected = 0;
        }
        else if (Selected == 0) {
          Selected = 1;
        }
        Select();
        break;
      }
    }
  }
  UpdateLeds();

  // SLEEP *********************************************************************
  if (Running == 0) {
    pinMode(K3Led, INPUT_PULLUP);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, LOW);
    K3Time = millis() + 2000;
    while (digitalRead(K3Led) == LOW) {
      Serial.println(digitalRead (K3Led));
      if (millis() >= K3Time) {
        pinMode(K1Led, OUTPUT);         //
        pinMode(K2Led, OUTPUT);         //
        pinMode(K3Led, OUTPUT);         //
        digitalWrite(K1Led, LOW);       //
        digitalWrite(K2Led, LOW);       //
        digitalWrite(K3Led, LOW);       //  Обратна връзка чрез светване на всичките светодиоди
        digitalWrite(Led3Anode, HIGH);  //
        digitalWrite(Led124Anode, HIGH);//
        Serial.println(F("sleepNow"));  //
        delay(1000);                    //
        digitalWrite(Led3Anode, LOW);
        digitalWrite(Led124Anode, LOW);
        pinMode(K1Led, INPUT_PULLUP);
        pinMode(K2Led, INPUT_PULLUP);
        pinMode(K3Led, INPUT_PULLUP);
        sleepNow();
      }
    }
  }
  UpdateLeds();

  // SLEEP TIME *********************************************************************
  currentMillis = millis();
  if (currentMillis - TimerOff > 600000) {
    TimerOff = currentMillis;
    Serial.println(F("sleepNow"));
    pinMode(K1Led, OUTPUT);
    pinMode(K2Led, OUTPUT);
    pinMode(K3Led, OUTPUT);
    digitalWrite(K1Led, LOW);
    digitalWrite(K2Led, LOW);
    digitalWrite(K3Led, LOW);
    digitalWrite(Led3Anode, HIGH);
    digitalWrite(Led124Anode, HIGH);
    delay(1000);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, LOW);
    pinMode(K1Led, INPUT_PULLUP);
    pinMode(K2Led, INPUT_PULLUP);
    pinMode(K3Led, INPUT_PULLUP);
    sleepNow();
  }
  // ALARMS *********************************************************************
  //pinMode(WaterLevel, INPUT);
  if (digitalRead(WaterLevel) == HIGH) {
    TimeWater = millis() + 3000;
    if (millis() < TimeWater) {
      WaterErr = 1;
    }
  }
  UpdateLeds();
}
// *********************************************************************
// Run
// *********************************************************************
void Run() {
  // Serial.println(F("Run"));
  // SelfClean *********************************************************************
  pinMode(K1Led, INPUT_PULLUP);
  digitalWrite(Led124Anode, LOW);
  K1Time = millis() + 1000;
  while (digitalRead (K1Led) == LOW) {
    if (millis() >= K1Time) {
      pinMode(K1Led, OUTPUT);         //
      pinMode(K2Led, OUTPUT);         //
      pinMode(K3Led, OUTPUT);         //
      digitalWrite(K1Led, LOW);       //
      digitalWrite(K2Led, LOW);       //
      digitalWrite(K3Led, LOW);       //  Обратна връзка чрез светване на всичките светодиоди
      digitalWrite(Led3Anode, HIGH);  //
      digitalWrite(Led124Anode, HIGH);//
      delay(1000);
      CleaningTime = millis() + 1200000;
      SelfClean();
      if (Running == 1) {
        Running = 0;
      }
      break;
    }
  }
  UpdateLeds();
  currentMillis = millis();
  if (CicleSelect == 5) {
    CicleSelect = 7;
  }

  // Stage1 *********************************************************************
  if (Stage1 == 0 && Running == 1) { //Стъпка 1
    digitalWrite(WaterPump, HIGH); //Включваме водната помпа
    Serial.println((TimeWater + 60000) - currentMillis); //
    if (currentMillis >= TimeWater + 60000) {
      Stage1 = 1;
      Serial.println(F("TimeWater finish")); //
      delay(1000);
      TimerHotGas = millis();
    }
  }

  // Stage2 *********************************************************************
  if (Stage2 == 0 && Stage1 == 1 && Running == 1) { //Стъпка 2
    digitalWrite(Compressor, HIGH); //Включваме компресора
    digitalWrite(HotGas, HIGH); //Включваме горещите пари
    Serial.println((TimerHotGas + 60000) - currentMillis); //
    if (currentMillis >= TimerHotGas + 60000) {
      digitalWrite(HotGas, LOW); //Изключваме горещите пари след изминалото време
      Stage2 = 1;
      Serial.println(F("TimerHotGas finish")); //
      delay(1000);
      WorkingTime = millis();
    }
  }

  // Stage3 *********************************************************************
  if (Stage3 == 0 && Stage2 == 1 && Stage1 == 1 && Running == 1) { //Стъпка 3
    digitalWrite(Fan, HIGH); //Всключваме вентилатора на кондензатора
    GetTemp();
    if (Selected == 1) {
      if (OutTemp >= 23) {
        ProgramTime = 5400000;
      }
      if (OutTemp < 23) {
        ProgramTime = 4500000;
      }
    }
    else if (Selected == 0) {
      if (OutTemp >= 23) {
        ProgramTime = 4500000;
      }
      if (OutTemp < 23) {
        ProgramTime = 3600000;
      }
    }
    Serial.println((WorkingTime + ProgramTime) - currentMillis); //
    if (currentMillis >= WorkingTime + ProgramTime) {
      Stage3 = 1;
      Serial.println(F("WorkingTime finish")); //
      delay(1000);
      DefrostTime = millis();
      digitalWrite(WaterPump, LOW); //След изминалото време изключваме водната помпа
    }
  }

  // Stage4 *********************************************************************
  if (Stage4 == 0 && Stage3 == 1 && Stage2 == 1 && Stage1 == 1 && Running == 1 && lastCicle == 1) {
    RunCicle = RunCicle + 1;
  }
  if (Stage4 == 0 && Stage3 == 1 && Stage2 == 1 && Stage1 == 1 && Running == 1 && lastCicle == 0) { //Стъпка 4
    digitalWrite(HotGas, HIGH); //Включваме горещите пари за да отделим леда
    Serial.println((DefrostTime + 120000) - currentMillis); //
    if (currentMillis >= DefrostTime + 120000) {
      Stage4 = 1;
      Serial.println(F("DefrostTime finish")); //
      // Край на програмата изключваме всички компоненти
      digitalWrite(Compressor, LOW);
      digitalWrite(HotGas, LOW);
      digitalWrite(Fan, LOW);
      if (RunCicle < CicleSelect) {
        RunCicle = RunCicle + 1;
      }
      TimerOff = currentMillis;
      TimeIce = millis() + 10000;
      while (digitalRead(ReedSensor) == LOW) {
        if (CicleSelect <= 4 && lastCicle == 0) {
          if (millis() >= TimeIce) {
            Serial.print ("Ice ");
            RunCicle = CicleSelect;
            break;
          }
        }
        if (CicleSelect >= 5 && lastCicle == 0 && Stage4 == 1) {
          if (millis() >= TimeIce) {
            Serial.print ("Full ");
            RunCicle = CicleSelect - 1;
            lastCicle = 1;
            break;
          }
        }
      }
    }
  }
  //pinMode(WaterLevel, INPUT);
  if (digitalRead(WaterLevel) == HIGH) {
    TimeWater = millis() + 3000;
    if (millis() < TimeWater) {
      WaterErr = 1;
      WaterAlarm();
    }
  }

  // Cancel *********************************************************************
  if (Running > 0) {
    pinMode(K3Led, INPUT_PULLUP);
    digitalWrite(Led124Anode, LOW);
    K3Time = millis() + 2000;
    while (digitalRead(K3Led) == LOW) {
      Serial.println(digitalRead (K3Led));
      if (millis() >= K3Time) {
        pinMode(K1Led, OUTPUT);
        pinMode(K2Led, OUTPUT);
        pinMode(K3Led, OUTPUT);
        digitalWrite(K1Led, LOW);
        digitalWrite(K2Led, LOW);
        digitalWrite(K3Led, LOW);
        digitalWrite(Led3Anode, HIGH);
        digitalWrite(Led124Anode, HIGH);
        delay(1000);
        digitalWrite(Led3Anode, LOW);
        digitalWrite(Led124Anode, LOW);
        pinMode(K1Led, INPUT_PULLUP);
        pinMode(K2Led, INPUT_PULLUP);
        pinMode(K3Led, INPUT_PULLUP);
        Running = 0;
        digitalWrite(Compressor, LOW);
        digitalWrite(HotGas, LOW);
        digitalWrite(WaterPump, LOW);
        digitalWrite(Fan, LOW);
        Running = 0;
        break;
      }
    }
  }

  Serial.print(RunCicle);
  Serial.print(CicleSelect);
  Serial.print(" ");
  if (digitalRead(ReedSensor) == LOW && Stage4 == 0 && Stage3 == 0 && Stage2 == 0 && Stage1 == 0 && lastCicle == 0) {
    TimeIce = millis() + 1000;
    IceFull();
  }
  if (Stage4 == 1) {
    Stage1 = 0;
    Stage2 = 0;
    Stage3 = 0;
    Stage4 = 0;
    AC = millis() + 120000;
    AntiShortCicle();
  }
  if (RunCicle >= CicleSelect && Running == 1) {
    Running = 0;
    CicleSelect = EEPROM.read(CAddress);
  }
  if (Running == 1) {
    Run();
  }
}

// *********************************************************************
// AntiShortCicle
// *********************************************************************
void AntiShortCicle() {
  while (AC >= millis()) {
    UpdateLeds();
    Serial.print("AntiShortCicle");
    digitalWrite(WaterPump, HIGH);
    pinMode(K3Led, INPUT_PULLUP);
    digitalWrite(Led124Anode, LOW);
    if (digitalRead(K3Led) == LOW) {
      break;
    }
    if (millis() > AC) {
      break;
    }
  }
}
// *********************************************************************
// SelfClean
// *********************************************************************
void SelfClean() {
  digitalWrite(Compressor, LOW);
  digitalWrite(HotGas, LOW);
  digitalWrite(Fan, LOW);
  digitalWrite(WaterPump, HIGH); //Включваме водната помпа
  while (CleaningTime >= millis()) {
    currentMillis = millis();
    Serial.println(CleaningTime - currentMillis);
    pinMode(K1Led, OUTPUT);
    pinMode(K2Led, OUTPUT);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, HIGH);
    digitalWrite(K1Led, LOW);
    digitalWrite(K2Led, HIGH);
    delay(50);
    digitalWrite(Led124Anode, LOW);
    digitalWrite(Led3Anode, HIGH);
    digitalWrite(K2Led, LOW);
    digitalWrite(K1Led, HIGH);
    delay(50);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, LOW);

    if (millis() > CleaningTime) {
      Serial.println(F("Cleaning finish")); //
      break;
      delay(1000);
    }
    pinMode(K3Led, INPUT_PULLUP);
    K3Time = millis() + 2000;
    if (digitalRead(K3Led) == LOW) {
      while (digitalRead(K3Led) == LOW) {
        Serial.println(digitalRead (K3Led));
        if (millis() >= K3Time) {
          pinMode(K1Led, OUTPUT);
          pinMode(K2Led, OUTPUT);
          pinMode(K3Led, OUTPUT);
          digitalWrite(K1Led, LOW);
          digitalWrite(K2Led, LOW);
          digitalWrite(K3Led, LOW);
          digitalWrite(Led3Anode, HIGH);
          digitalWrite(Led124Anode, HIGH);
          delay(1000);
          CleaningTime = millis();
          break;
        }
      }
    }
  }
}

// *********************************************************************
// Startup
// *********************************************************************
void Startup() {
  Serial.println(F("Startup"));
  Time1 = millis() + 6000;
  int Startup = LOW;
  while (millis() < Time1) {
    pinMode(K1Led, OUTPUT);
    pinMode(K2Led, OUTPUT);
    pinMode(K3Led, OUTPUT);
    digitalWrite(Led3Anode, HIGH);
    digitalWrite(Led124Anode, HIGH);

    unsigned long previousMillis;
    currentMillis = millis();

    if (currentMillis - previousMillis >= 500) {
      // Запази последното време когато диода е мигал
      previousMillis = currentMillis;

      // Ако диода е изключен светни го и обратното
      if (Startup == LOW) {
        Startup = HIGH;
      } else {
        Startup = LOW;
      }

      // Включи изходните пинове със статуса на диода

      digitalWrite(K1Led, Startup);
      digitalWrite(K2Led, Startup);
      digitalWrite(K3Led, Startup);
      if (millis() >= Time1) {
        Running = 0;
        FirstStart = 0;
        delay(1000);
        TimerOff = currentMillis;
        Select();
      }
    }
  }
}

// *********************************************************************
// Select
// *********************************************************************
void Select() {
  GetTemp();
  Serial.println(F("Select")); //
  // Избираме режима на работа
  pinMode(K2Led, INPUT_PULLUP);
  digitalWrite(Led3Anode, LOW);
  digitalWrite(Led124Anode, LOW);
  if (digitalRead(K2Led) == LOW) {
    K2Time = millis() + 2000;
  }
  while (digitalRead (K2Led) == LOW) {
    Serial.println(digitalRead (K2Led));
    if (millis() >= K2Time) {
      pinMode(K1Led, OUTPUT);         //
      pinMode(K2Led, OUTPUT);         //
      pinMode(K3Led, OUTPUT);         //
      digitalWrite(K1Led, LOW);       //
      digitalWrite(K2Led, LOW);       //
      digitalWrite(K3Led, LOW);       //  Обратна връзка чрез светване на всичките светодиоди
      digitalWrite(Led3Anode, HIGH);  //
      digitalWrite(Led124Anode, HIGH);//
      Serial.println(F("Selecting")); //
      delay(100);                    //
      Selecting();
      //break;
    }
    if (digitalRead (K2Led) == HIGH) {
      break;
    }
  }
  Serial.print(F("K2Led  LOW"));
  TimerOff = currentMillis;
  UpdateLeds();
  delay(1000);
}
// *********************************************************************
// Selecting
// *********************************************************************
void Selecting() {
  Serial.println(F("CicleSelect")); //
  CicleSelect = EEPROM.read(CAddress);
  unsigned long TimeSelecting;
  TimeSelecting = millis() + 10000;
  while (millis() < TimeSelecting) {
    if (CicleSelect == 1) {
      digitalWrite(Led3Anode, LOW);
      digitalWrite(Led124Anode, HIGH);
      pinMode(K1Led, OUTPUT);
      pinMode(K2Led, OUTPUT);
      pinMode(K3Led, OUTPUT);
      digitalWrite(K2Led, LOW);
      digitalWrite(K1Led, HIGH);
      digitalWrite(K3Led, HIGH);
    }
    else if (CicleSelect == 2) {
      digitalWrite(Led3Anode, HIGH);
      digitalWrite(Led124Anode, LOW);
      pinMode(K1Led, OUTPUT);
      pinMode(K2Led, OUTPUT);
      pinMode(K3Led, OUTPUT);
      digitalWrite(K2Led, LOW);
      digitalWrite(K1Led, HIGH);
      digitalWrite(K3Led, HIGH);
    }
    else if (CicleSelect == 3) {
      digitalWrite(Led3Anode, LOW);
      digitalWrite(Led124Anode, HIGH);
      pinMode(K1Led, OUTPUT);
      pinMode(K2Led, OUTPUT);
      pinMode(K3Led, OUTPUT);
      digitalWrite(K1Led, LOW);
      digitalWrite(K2Led, HIGH);
      digitalWrite(K3Led, HIGH);
    }
    else if (CicleSelect == 4) {
      digitalWrite(Led3Anode, LOW);
      digitalWrite(Led124Anode, HIGH);
      pinMode(K1Led, OUTPUT);
      pinMode(K2Led, OUTPUT);
      pinMode(K3Led, OUTPUT);
      digitalWrite(K3Led, LOW);
      digitalWrite(K1Led, HIGH);
      digitalWrite(K2Led, HIGH);
    }
    else if (CicleSelect == 5) {
      digitalWrite(Led3Anode, HIGH);
      digitalWrite(Led124Anode, HIGH);
      pinMode(K1Led, OUTPUT);
      pinMode(K2Led, OUTPUT);
      pinMode(K3Led, OUTPUT);
      digitalWrite(K3Led, LOW);
      digitalWrite(K1Led, LOW);
      digitalWrite(K2Led, LOW);
    }
    pinMode(K2Led, INPUT_PULLUP);
    digitalWrite(Led3Anode, LOW);
    digitalWrite(Led124Anode, LOW);
    if (digitalRead(K2Led) == LOW) {
      unsigned long PreviousMillis;
      currentMillis = millis();
      if (currentMillis - PreviousMillis >= 300) {
        PreviousMillis = currentMillis;

        if (CicleSelect < 5) {
          CicleSelect = CicleSelect + 1;
        } else if (CicleSelect >= 5) {
          CicleSelect = 1;
        }
        TimeSelecting = millis() + 10000;
        Serial.println(CicleSelect);
      }
    }
    if (millis() >= TimeSelecting) {
      if (CicleSelect != EEPROM.read(CAddress))
      {
        EEPROM.write(CAddress, CicleSelect);
      }
      break;
    }
  }
}
// *********************************************************************
// GetTemp
// *********************************************************************
void GetTemp() {
  // Получаване на температура от външния датчик
  int RawADC;
  RawADC = analogRead(NTCPin);
  double Temp;
  Temp = log(10000.0 * ((1024.0 / RawADC - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  Temp = Temp - 273.15;
  OutTemp = Temp;
  TimerOff = currentMillis;
}
// *********************************************************************
// WaterAlarm
// *********************************************************************
void WaterAlarm() {
  // Serial.println(F("WaterAlarm"));
  int WAlarm = LOW;
  digitalWrite(Led124Anode, HIGH);
  pinMode(K1Led, OUTPUT);
  pinMode(K2Led, OUTPUT);
  pinMode(K3Led, OUTPUT);
  digitalWrite(K1Led, HIGH);
  digitalWrite(K2Led, HIGH);
  digitalWrite(K3Led, HIGH);
  Time2 = millis() + 600000;
  while (millis() < Time2) {

    unsigned long previousMillis;
    currentMillis = millis();

    if (currentMillis - previousMillis >= 1000) {
      // Запази последното време когато диода е мигал
      previousMillis = currentMillis;

      if (WAlarm == LOW) {
        WAlarm = HIGH;
        analogWrite(Buzzer, 220);
      } else {
        WAlarm = LOW;
        analogWrite(Buzzer, 0);
      }
      Running = 0;
      pinMode(K3Led, OUTPUT);
      digitalWrite(K3Led, WAlarm);
      if (millis() > Time2) {
        sleepNow();
      }
      if (digitalRead(WaterLevel) == LOW) {
        analogWrite(Buzzer, 0);
        delay(2000);
        WaterErr = 0;
        FillUp = 0;
        break;
      }
    }
  }
}

// *********************************************************************
// IceFull
// *********************************************************************

void IceFull() {
  Serial.print ("IceFull ");
  while (digitalRead(ReedSensor) == LOW) {
    if (millis() > TimeIce) {
      int IceAlarm = LOW;
      digitalWrite(Led124Anode, HIGH);
      pinMode(K1Led, OUTPUT);
      pinMode(K2Led, OUTPUT);
      pinMode(K3Led, OUTPUT);
      digitalWrite(K1Led, HIGH);
      digitalWrite(K2Led, HIGH);
      digitalWrite(K3Led, HIGH);
      Time3 = millis() + 10000;
      analogWrite(Buzzer, 250);
      delay(1000);
      while (millis() < Time3) {
        analogWrite(Buzzer, 0);

        unsigned long previousMillis;
        currentMillis = millis();

        if (currentMillis - previousMillis >= 750) {
          // Запази последното време когато диода е мигал
          previousMillis = currentMillis;

          if (IceAlarm == LOW) {
            IceAlarm = HIGH;
          } else {
            IceAlarm = LOW;
          }
          Running = 0;
          pinMode(K2Led, OUTPUT);
          digitalWrite(K2Led, IceAlarm);
          if (millis() > Time3) {
            Running = 0;
            break;
            loop();
          }
          if (digitalRead(ReedSensor) == HIGH) {
            analogWrite(Buzzer, 0);
            delay(2000);
            break;
          }
        }
      }
    }
  }
}
// *********************************************************************
// UpdateLeds
// *********************************************************************
void UpdateLeds() {
  // Serial.println(F("UpdateLeds"));
  if (Selected == 1) {
    digitalWrite(Led124Anode, HIGH);
    pinMode(K1Led, OUTPUT);
    digitalWrite(K1Led, LOW);
  }
  else if (Selected == 0) {
    digitalWrite(Led3Anode, HIGH);
    pinMode(K2Led, OUTPUT);
    digitalWrite(K2Led, LOW);
  }
}
