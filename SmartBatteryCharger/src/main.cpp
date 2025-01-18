/*==============================================
    LATEST EDIT : 21-June-2024
  ==============================================  */

// constants won't change. They're used here to set pin numbers:
#include <LiquidCrystal.h>
#include <Arduino.h>
#include <Wire.h> 


LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
struct pair getMinMax(byte arr[], int low, int high);
struct pair getsMinMax(byte arr[], int n);

#define DEBOUNCE 10           // how many ms to debounce, 5+ ms is usually plenty
#define NUMBUTTONS sizeof(5)  //determine how big the array up above is, by checking the size

int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
//Define the buttons that we'll use.
const int numOfInputs = 4;
byte buttons[] = { 9, 10 };

double currentTemp = 0;  //Variable to hold the current Temperature

int SensVal = 0;
int potentiometer = A0;  //From the main potentiometer
int PWM = 11;
int count = 120;

int Analogpin0_value = 0;
int VsupPin = A0;
int BATPin = A1;
//int TPin = 8;
//int BtnRun = 9;
//int BtnMode = 10;

// int BtnRun = A3;
// int BtnMode = A4;
// int BtnEXIT = A5;

int ThermistorPin = A0;
const int SAMPLE_NUMBER = 50;             //ADC Sample Size
const double BALANCE_RESISTOR = 99800.0;  // Resistor Divider Value
const double MAX_ADC = 1023.0;            //ADC Value
const double BETA = 3974.0;               //NTC Beta Value

const double ROOM_TEMP = 298.15;  // room temperature in Kelvin
const double RESISTOR_ROOM_TEMP = 98600.0;

//track if a button is just pressed, just released, or 'currently pressed'
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
byte previous_keystate[NUMBUTTONS], current_keystate[NUMBUTTONS];


//===================================
// Function Prototype
//===================================
void CheckPins(int menu);
void check_switches();
void VAMode();
void TemperatureMode();
void FlickerMode();
void CounterMode();
void PMWMode();
void TemperatureMode();
int adc_sampling(int adcPin);
float FlickPercentage(float max, float min) ;
float vmeas(uint8_t pins);
void CounterMode();
void InitSensorReading();
byte SensorReading();
double readThermistor();
void DisplayFlickerVal(float val);


//Battery Monitor
//+++++++++++++++++++++++++
float voltage, current;
float vout = 0.0;
float vin1 = 0.0;
float vin2 = 0.0;
float vin = 0.0;
float Vshunt = 0.0;
float R1 = 100000;
float R2 = 10000;
int vol = A0;
int val = 0;
int charging = 11;
int charged = 12;
int BATLOW = 8;

int sensorValue = 0;    // value read from the pot
float outputValue = 0;  // value output to the PWM (analog out)

//int PWR = A2;
//int IOUT = A3;
String unit = " A ";
int state = 0;
int ACState = 10;
//+++++++++++++++++++++++++

float FlickerVal = 0;
int J = 200;
byte data[600];

struct pair {
  float min;
  float max;
};

class Flasher {
  // Class Member Variables
  // These are initialized at startup
  int ledPin;    // the number of the LED pin
  long OnTime;   // milliseconds of on-time
  long OffTime;  // milliseconds of off-time

  // These maintain the current state
  int ledState;                  // ledState used to set the LED
  unsigned long previousMillis;  // will store last time LED was updated

  // Constructor - creates a Flasher
  // and initializes the member variables and state
public:
  Flasher(int pin, long on, long off) {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);

    OnTime = on;
    OffTime = off;

    ledState = LOW;
    previousMillis = 0;
  }

  void Update() {
    //check to see if it's time to change the state of the LED
    unsigned long currentMillis = millis();

    if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime)) {
      ledState = LOW;                  // Turn it off
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(ledPin, ledState);  // Update the actual LED
    } else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime)) {
      ledState = HIGH;                 // turn it on
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(ledPin, ledState);  // Update the actual LED
    }
  }
};

int MySelection = 0;
Flasher changeDir(11, 400, 400);

//=============================================================================
void CheckPins(int menu) {
  switch (menu) {
    case 1:
      MySelection = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("VOLT-AMP");
      lcd.setCursor(0, 1);
      lcd.print("MODE");
      delay(1500);
      break;
    case 2:
      MySelection = 2;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TEMP");
      lcd.setCursor(0, 1);
      lcd.print("MODE");
      delay(1500);
      break;
    case 3:
      MySelection = 3;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FLICKER");
      lcd.setCursor(0, 1);
      lcd.print("MODE");
      delay(1500);
      break;
    case 4:
      MySelection = 4;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("COUNTER");
      lcd.setCursor(0, 1);
      lcd.print("MODE");
      delay(1500);
      break;
    case 5:
      MySelection = 5;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PWM");
      lcd.setCursor(0, 1);
      lcd.print("MODE");
      delay(1500);
      break;
    default:
      break;
  }
  //Serial.println(MySelection);
}


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  lcd.begin(8, 2);
  lcd.setCursor(0, 0);
  lcd.print("SELECT");
  lcd.setCursor(0, 1);
  lcd.print("MENU");
  byte i;
  // Make input & enable pull-up resistors on switch pins
  for (i = 0; i < NUMBUTTONS; i++) {
    pinMode(buttons[i], INPUT);
  }
  pinMode(potentiometer, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(charging, OUTPUT);
  pinMode(charged, OUTPUT);
  pinMode(BATLOW, OUTPUT);
  pinMode(BATPin, INPUT);
  //pinMode(PWR, INPUT);
  //pinMode(IOUT, INPUT);
  pinMode(ACState, INPUT_PULLUP);
  pinMode(VsupPin, INPUT);
  digitalWrite(charging, LOW);
  digitalWrite(charged, LOW);
  digitalWrite(BATLOW, LOW);
  TCCR2B = TCCR2B & B11111000 | B00000001;  // pin 3 and 11 PWM frequency of 31372.55 Hz
  // int Screen = 0;
  // do
  // {
  //   byte thisSwitch = thisSwitch_justPressed();
  //   switch (thisSwitch)
  //   {
  //     case 0:
  //       Screen++;
  //       if (Screen == 1) {
  //         lcd.setCursor(0, 1);
  //         lcd.print("1.VOLT-AMP  ");
  //       } else if (Screen == 2) {
  //         lcd.setCursor(0, 1);
  //         lcd.print("2.TEMP  ");
  //       } else if (Screen == 3) {
  //         lcd.setCursor(0, 1);
  //         lcd.print("3.FLICK%  ");
  //       } else if (Screen == 4) {
  //         lcd.setCursor(0, 1);
  //         lcd.print("4.COUNT ");
  //       } else if (Screen == 5) {
  //         lcd.setCursor(0, 1);
  //         lcd.print("5.PWM   ");
  //       }

  //       if (Screen > 5)
  //         Screen = 0;
  //       break;

  //     case 1:
  //       if (Screen == 1) {
  //         CheckPins(1);
  //       } else if (Screen == 2) {
  //         CheckPins(2);
  //       } else if (Screen == 3) {
  //         CheckPins(3);
  //       } else if (Screen == 4) {
  //         CheckPins(4);
  //       } else if (Screen == 5) {
  //         CheckPins(5);
  //       }
  //       break;

  //     case 2:
  //       lcd.setCursor(0, 1);
  //       lcd.print("2");
  //       break;
  //   }
  // } while (MySelection == 0);

  CheckPins(1);
}



void check_switches() {
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static long lasttime;
  byte index;
  if (millis() < lasttime) {
    // we wrapped around, lets just try again
    lasttime = millis();
  }
  if ((lasttime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return;
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();
  for (index = 0; index < NUMBUTTONS; index++) {
    justpressed[index] = 0;  //when we start, we clear out the "just" indicators
    justreleased[index] = 0;
    currentstate[index] = digitalRead(buttons[index]);  //read the button
    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
        // just pressed
        justpressed[index] = 1;
      } else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
        justreleased[index] = 1;  // just released
      }
      pressed[index] = !currentstate[index];  //remember, digital HIGH means NOT pressed
    }
    previousstate[index] = currentstate[index];  //keep a running tally of the buttons
  }
}

byte thisSwitch_justPressed() {
  byte thisSwitch = 255;
  check_switches();  //check the switches &amp; get the current state
  for (byte i = 0; i < NUMBUTTONS; i++) {
    current_keystate[i] = justpressed[i];
    if (current_keystate[i] != previous_keystate[i]) {
      if (current_keystate[i]) thisSwitch = i;
    }
    previous_keystate[i] = current_keystate[i];
  }
  return thisSwitch;
}


/*==============================================
    L O O P
  ==============================================  */

void loop() {
  switch (MySelection) {
    case 1:
      state = digitalRead(ACState); 
      delay(50);
      VAMode();
      break;
    case 2:
      TemperatureMode();
      break;
    case 3:
      FlickerMode();
      break;
    case 4:
      CounterMode();
      break;
    case 5:
      PMWMode();
      break;
    default:
      break;
  }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//     L C D   --   C O N T R O L

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void VAMode() {
  while(state == 0)
  {
      vin1 = vmeas(VsupPin);
      vin2 = vmeas(BATPin);
      Vshunt = abs(vin1-vin2);
      float Ival =  Vshunt/0.23;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BAT VOLTS");
      lcd.setCursor(0, 1);
      lcd.print("V=" + String(vin1) + "V");
      delay(500);

      //Current OUT
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DISCHARG");
      lcd.setCursor(0, 1);
      lcd.println("I=" + String(-Ival) + "A ");
      delay(500);

    //Power Drawn OUT   
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PWR OUT");
      lcd.setCursor(0, 1);
      lcd.println("P=" + String(vin1*Ival) + "W ");
      delay(500);  
      Serial.println(String(vin1) + "," + String(-Ival) + "," + state + "\n");

      if(state == 0 && vin2 < 12.10){  //Cutt off when battery drop to 12.10V
         lcd.clear();
         lcd.setCursor(0, 0);
         lcd.print("BAT VOLTS");
         lcd.setCursor(0, 1);
         lcd.print("LOW  " + String(count));
         delay(500);

         lcd.setCursor(0, 1);
         lcd.print("V2=" + String(vin2) + "V ");
         delay(500);

         //count -= 1;
         // if (count <= 0){
         //   lcd.clear();
         //   lcd.setCursor(0, 0);
         //   lcd.print("SHUTTING");
         //   lcd.setCursor(0, 1);
         //   lcd.print("DOWN");
         //   delay(5000);
         //   digitalWrite(BATLOW, HIGH);
         //}
        }

       state = digitalRead(ACState); 
   }
   while(state == 1)
   {
      vin1 = vmeas(VsupPin);
      vin2 = vmeas(BATPin);
      Vshunt = abs(vin1-vin2);
      float Ival =  Vshunt/0.13;

      lcd.clear();    
      lcd.setCursor(0, 0);
      lcd.print("BAT VOLTS");
      lcd.setCursor(0, 1);
      lcd.print("V=" + String(vin2) + "V");
      delay(500);

     //Current IN
      lcd.clear();  
      lcd.setCursor(0, 0);
      lcd.print("CHARGING");
      lcd.setCursor(0, 1);
      lcd.print("I=" + String(Ival) + "A ");
      delay(500);

      //Power Drawn IN 
      lcd.setCursor(0, 0);
      lcd.print("PWR IN  ");
      lcd.setCursor(0, 1);
      lcd.println("P=" + String(vin2*Ival) + "W ");
      delay(500);
      Serial.println(String(vin2) + "," + String(+Ival) +  "," + state + "\n");

      state = digitalRead(ACState);
   }

  //delay(500); 
  //lcd.clear();
  // lcd.setCursor(0, 1);
  // lcd.print("V=" + String(vin1) + "V");

  //Serial.println("Vsup=" + String(vin1) + "V");  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //Serial.println("VBAT=" + String(vin2) + "V");  //=>>>>>>>>>>>>>>>>>>>>>>>>>
  //Serial.println("Vsh=" + String(Vshunt) + "V");  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>

  //Serial.println("Pout=" + String(vin1*Vshunt) + "W"); //>>>>>>>>>>>>>>>>>>>>
  //Serial.println("  "); 
}

//*++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Pass Analog Pin measureming ADC and Return Voltage
//+++++++++++++++++++++++++++++++++++++++++++++++++++++
float vmeas(uint8_t pins) {
  //val = analogRead(LightPin);
  val = adc_sampling(pins);
  //  Serial.print("ADC = ");
  //  Serial.println(val);
  vout = (val * 5.1) / 1024;
  float volts = vout / (R2 / (R1 + R2));  

  return volts;
}

int adc_sampling(int adcPin) {
  double adcAverage = 0;          //Holds the average voltage measurement
  int adcSamples[SAMPLE_NUMBER];  // Array to hold each voltage measurement

  //Calculate thermistor's average resistance:
  for (int i = 0; i < SAMPLE_NUMBER; i++) {
    adcSamples[i] = analogRead(adcPin);  // read from pin and store
    delay(10);
  }
  //Average value from sample
  for (int i = 0; i < SAMPLE_NUMBER; i++) {
    adcAverage += adcSamples[i];  // add all samples up . . .
  }
  adcAverage /= SAMPLE_NUMBER;  // . . . average it w/ divide

  return adcAverage;
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//     L C D   --   C O N T R O L

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void TemperatureMode() {
  currentTemp = readThermistor();
  lcd.setCursor(0, 0);
  lcd.print("TEMP");
  lcd.setCursor(0, 1);
  lcd.print(currentTemp);
  lcd.print((char)223);
  lcd.print("C");
  delay(2000);
}

void FlickerMode() {
  DisplayFlickerVal(FlickerVal);
  int a, b, i;
  for (i = 0; i < J; i++) {
    //data[i] = map(analogRead(LightPin), 0, 1023, 0, 255);  // or lightPin0
    byte SensorVal = SensorReading();
    data[i] = map(SensorVal, 0, 1023, 0, 255);
  }
  //struct pair minmax = getMinMax(data, 0, (J - 1));
  struct pair minmax = getsMinMax(data, J);

  FlickerVal = FlickPercentage(minmax.max, minmax.min);
  //DisplayFlickerVal(FlickerVal);
  delay(20);
}

void CounterMode() {
  DisplayFlickerVal(FlickerVal);
  lcd.setCursor(0, 0);
  lcd.print("COUNTER");
  lcd.setCursor(0, 1);
  lcd.print("METER");
  delay(2000);

  changeDir.Update();
}

void PMWMode() {
  float voltage = analogRead(potentiometer);
  int VALUE = map(voltage, 0, 1024, 0, 254);
  analogWrite(PWM, VALUE);
  lcd.setCursor(0, 0);
  lcd.print("PWM");
  lcd.setCursor(0, 1);
  lcd.print(VALUE);
  delay(2000);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//     L C D   --   C O N T R O L

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void DisplayFlickerVal(float val) {
  lcd.setCursor(0, 0);
  lcd.print("FLICKER");
  lcd.setCursor(0, 1);
  lcd.print(val + String("%  "));
}


//+++++++++++++++++++++++++++++

//  Flicker Index CALCULATIONS

//+++++++++++++++++++++++++++++

float FlickPercentage(float max, float min) {
  float flickVal = 0.00;

  if (max == 0.00) {
    flickVal = 0.00;
  } else if (max > 0.00) {
    flickVal = ((max - min) / (max + min)) * 100;
  }

  return flickVal;
}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//SMOOTHING

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Do Initial Reading on the Sensor
void InitSensorReading() {
  Analogpin0_value = analogRead(VsupPin);
  delay(5);
}

// Take a second Reading and do Filtering
byte SensorReading() {
  // Analog Filtering
  Analogpin0_value = Analogpin0_value * (1.0 - 0.125) + (analogRead(VsupPin) * 0.125);
  return Analogpin0_value;
}




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//FUNCTION STRUCTURES

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct pair getMinMax(byte arr[], int low, int high) {
  struct pair minmax, mml, mmr;
  int mid;

  //If There is only one elements
  if (low == high) {
    minmax.max = arr[low];
    minmax.min = arr[low];
    return minmax;
  }

  //If there are two elements
  if (high == low + 1) {
    if (arr[low] > arr[high]) {
      minmax.max = arr[low];
      minmax.min = arr[high];
    } else {
      minmax.max = arr[high];
      minmax.min = arr[low];
    }
    return minmax;
  }

  //If there are more than 2 elements
  mid = (low + high) / 2;
  mml = getMinMax(arr, low, mid);
  mmr = getMinMax(arr, mid + 1, high);

  //Compare minimum of two parts
  if (mml.min < mmr.min)
    minmax.min = mml.min;
  else
    minmax.min = mmr.min;

  //Compare maximums of two parts
  if (mml.max > mmr.max)
    minmax.max = mml.max;
  else
    minmax.max = mmr.max;


  return minmax;
};


struct pair getsMinMax(byte arr[], int n) {
  struct pair minmax;
  int i;

  if (n == 1) {
    minmax.max = arr[0];
    minmax.min = arr[0];
    return minmax;
  }

  if (arr[0] > arr[1]) {
    minmax.max = arr[0];
    minmax.min = arr[1];
    return minmax;
  } else {
    minmax.max = arr[1];
    minmax.min = arr[0];
  }

  for (i = 2; i < n; i++) {
    if (arr[i] > minmax.max)
      minmax.max = arr[i];
    else if (arr[i] < minmax.min)
      minmax.min = arr[i];
  }
  return minmax;
};

double readThermistor() {
  double rThermistor = 0;         //thermistor resistance value
  double tKelvin = 0;             //calculated temperature
  double tCelsius = 0;            //temperature in celsius
  double adcAverage = 0;          //Holds the average voltage measurement
  int adcSamples[SAMPLE_NUMBER];  // Array to hold each voltage measurement

  //Calculate thermistor's average resistance:
  for (int i = 0; i < SAMPLE_NUMBER; i++) {
    adcSamples[i] = analogRead(ThermistorPin);  // read from pin and store
    delay(10);
  }

  //Average value from sample
  for (int i = 0; i < SAMPLE_NUMBER; i++) {
    adcAverage += adcSamples[i];  // add all samples up . . .
  }
  adcAverage /= SAMPLE_NUMBER;  // . . . average it w/ divide

  //Thermistor resistor equation
  rThermistor = BALANCE_RESISTOR * ((MAX_ADC / adcAverage) - 1);

  //Beta Equations
  tKelvin = (BETA * ROOM_TEMP) / (BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));
  tCelsius = tKelvin - 273.15;  // convert kelvin to celsius

  return tCelsius;
}
