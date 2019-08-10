#include <EEPROM.h>
#include <SimpleDHT.h>
#include <PID_v1.h>

// ************************************************
// Pin definitions
// ************************************************

// Output pins
#define FanPin 2
#define HeatingOffPin 3
#define HeatingOnPin 4

// Input pins
#define DHTPin 5
#define SwitchPin1 6
#define SwitchPin2 7
#define SwitchPin3 8


// ************************************************
// Switch
// ************************************************

#define SwitchModeInvalid 4 
#define SwitchMode1 1
#define SwitchMode2 2
#define SwitchMode3 3
int switchMode = SwitchModeInvalid;
int switchModePrevious = -1;

bool IsHeatingAllowed;
// Min temperature dictated by Setpoint

bool IsFanAllowed;
bool IsFanOn;
int fanMinutesPerHour;
unsigned long fanStartTime;

const unsigned long millisPerHour = 3600000;
//const unsigned long millisPerHour = 60000;
const unsigned long millisPerMinute = 60000;
//const unsigned long millisPerMinute = 1000;


// ************************************************
// DHT11 Temperature and humidity sensor
// ************************************************
SimpleDHT11 dht11;
byte temperature = 0;
byte humidity = 0;
byte data[40] = {0};

// ************************************************
// PID Variables and constants
// ************************************************

// Output window size (Active for WS * DC milliseconds every WS milliseconds)
long WindowSize = 10000; 

// Variables associated with the PID
double Setpoint;
double Input;
double Output;
volatile long onTime = 0;
unsigned long windowStartTime;
double DutyCycle;
bool IsInActiveWindow;
bool IsHeatingActive;

// PID tuning parameters
double Kp = 200;
double Ki = 0;
double Kd = 0;
 
// The PID object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600);
  
  // Initialise pins
  pinMode(FanPin, OUTPUT);
  pinMode(HeatingOffPin, OUTPUT);
  pinMode(HeatingOnPin, OUTPUT);
  
  digitalWrite(FanPin, LOW);  // make sure the relays are off to start with
  digitalWrite(HeatingOnPin, LOW);
  digitalWrite(HeatingOffPin, HIGH);

  pinMode(DHTPin, INPUT);
  pinMode(SwitchPin1, INPUT_PULLUP);
  pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP);

  // Initialise fan variables
  fanStartTime = millis();

  // Initialise the PID variables
  windowStartTime = millis();

  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize / 10);
  
  // Turn the PID on
  myPID.SetMode(AUTOMATIC);


  Serial.println("Starting");
}


void loop() {
  if (ReadSwitchSetting()) {
    RefreshStateAccordingToMode();  
  }

  UpdateFanStatus();
  
  // Update Input temperature
  CheckTemperatureSensor();
  
  RecalculateInputVariables();

  // Pass execution off to the PID to recalculate Output
  myPID.Compute();

  UpdateWindow();
  RecalculateDutyCycle();  

  UpdateOutPins();

  PrintSerial();
  delay(500);
}

void RefreshStateAccordingToMode() {
  switch (switchMode) {
  case SwitchMode1:
    IsFanAllowed = false;
    fanMinutesPerHour = 0;
    
    IsHeatingAllowed = true;
    Setpoint = 24;
    
    break;
  case SwitchMode2:
    IsFanAllowed = true;
    fanMinutesPerHour = 1;
    
    IsHeatingAllowed = true;
    Setpoint = 10;
    
    break;
  case SwitchMode3:
    IsFanAllowed = true;
    fanMinutesPerHour = 10;

    IsHeatingAllowed = true;
    Setpoint = 20;
    
    break;
  }
}

void UpdateFanStatus() {
  if (IsFanAllowed) {
    int minutesPassed = ((millis() - fanStartTime) % millisPerHour) / millisPerMinute;
    if (IsFanOn && minutesPassed > fanMinutesPerHour) {
      IsFanOn = false;
    } else if (!IsFanOn && minutesPassed < fanMinutesPerHour) {
      IsFanOn = true;
    }
  } else {
    IsFanOn = false;
  }
}

// SP, In, AbsIn, Out, DC
void PrintSerial() {
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.print(Input);
  Serial.print(",");
  Serial.print(Output * 0.1);
  Serial.print(",");
  Serial.print(DutyCycle * 100);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(IsHeatingAllowed);
  Serial.print(",");
  Serial.print(IsHeatingActive);
  Serial.print(",");
  Serial.print(IsFanAllowed);
  Serial.print(",");
  Serial.print(IsFanOn);
  Serial.print(",");
  Serial.println(switchMode);
}

void RecalculateInputVariables() {
  if (IsHeatingAllowed) {
    IsHeatingActive = Setpoint > Input;
  } else {
    IsHeatingActive = false;
  }
}

void RecalculateDutyCycle() {
  if (IsHeatingAllowed) {
    // Recalculate the duty cycle as a fraction
    DutyCycle = ((10*Output) / WindowSize);
    IsInActiveWindow = DutyCycle * WindowSize > millis() - windowStartTime;
  } else {
    DutyCycle = 0;
    IsInActiveWindow = false;
  }
}

void UpdateWindow() {
  // If we have passed the end of the window, start the next one
  if (millis() - windowStartTime > WindowSize) {
    windowStartTime += WindowSize;
  }
}

void UpdateOutPins() {
  // Check if we are in the active part of the window and heating is active
  if (IsHeatingAllowed && IsInActiveWindow && IsHeatingActive) {
    digitalWrite(HeatingOffPin, LOW);
    digitalWrite(HeatingOnPin, HIGH);
  } 
  else 
  {
    // If we are in the inactive part of the window, disable the heating
    digitalWrite(HeatingOnPin, LOW);
    digitalWrite(HeatingOffPin, HIGH);
  }

  // Regardless of heating, turn the fan on when needed
  if (IsFanOn) {
    digitalWrite(FanPin, HIGH);
  } else {
    digitalWrite(FanPin, LOW);
  }
}

void CheckTemperatureSensor() {
  if (dht11.read(DHTPin, &temperature, &humidity, data)) {
    return;
  }
  Input = (int)temperature;
}

bool ReadSwitchSetting() {
  int previous = switchMode;
  
  if (digitalRead(SwitchPin1) == LOW) {
    switchMode = SwitchMode1;
  } else if (digitalRead(SwitchPin2) == LOW) {
    switchMode = SwitchMode2;
  } else if (digitalRead(SwitchPin3) == LOW) {
    switchMode = SwitchMode3;
  }

  return previous != switchMode;
}
