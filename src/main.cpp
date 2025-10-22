/**
 * @author Chitoan
 * @date June 2024
 * @brief AC Detect and Relay Control System
 * @version v2.0.0
 */

#include <Arduino.h>

// Define counts of components
#define buttonCount 5
#define acCount 4
#define relayCount 4
#define ledCount 8
#define ledCtrlCount 4

// define hardware pin
const int ButtonPin[buttonCount] = {PB0, PC15, PC7, PC6, PB11};        // BT1, BT2, BT3, BT4, BT5
const int ACPin[acCount] = {PA0, PA1, PC0, PC1};                       // ADC0, ADC1, ADC2, ADC3
const int RelayPin[relayCount] = {PC11, PC10, PA15, PC12};             // RLY1, RLY2, RLY3, RLY4
const int LEDPin[ledCount] = {PB3, PB4, PC3, PC4, PC5, PB9, PB8, PD2}; // a,b,c,d,e,f,g,dp
const int BuzzerPin = PA8;
const int LEDCtrl[ledCtrlCount] = {PC14, PC13, PD0, PD1};

// --------------------------- GLOBAL VARIABLES ---------------------------
volatile unsigned long lastPulseTime[acCount] = {0};
bool electricalDetect[acCount] = {false};
int relayActive = -1;

// Variables for timing
unsigned long previousMillis = 0;
const unsigned long interval = 1000;
unsigned long blinkInterval = 500;

// Variables for display status
unsigned long lastDisplay = 0;
const unsigned long displayInterval = 1000;

// Variables for buzzer
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 500;
int toneFreq = 2000;

// Variables for button debounce
bool lastButtonState[5];
unsigned long lastDebounceTime[5];
const unsigned long debounceDelay = 100;

// relay state
bool relay1State = false;
bool relay2State = false;
bool relay3State = false;
bool relay4State = false;

// --------------------------- DEFINE FUNCTION ---------------------------
void buzzerBeep(unsigned long duration);
void updateBuzzer();
/** 
 * @brief Initial setup function.
 * @param None
 */
// --------------------------- SETUP ---------------------------
void setup()
{
  // config button pins
  for (int i = 0; i < buttonCount; i++)
  {
    pinMode(ButtonPin[i], INPUT_PULLUP);
    lastButtonState[i] = digitalRead(ButtonPin[i]);
    lastDebounceTime[i] = 0;
  }
  // config ADC pins
  for (int i = 0; i < acCount; i++)
  {
    pinMode(ACPin[i], INPUT);
  }
  // congfig relay pins
  for (int i = 0; i < relayCount; i++)
  {
    pinMode(RelayPin[i], OUTPUT);
    digitalWrite(RelayPin[i], LOW);
  }
  // config 7 segment pins
  for (int i = 0; i < ledCount; i++)
  {
    pinMode(LEDPin[i], OUTPUT);
    digitalWrite(LEDPin[i], LOW);
  }
  // config control LED pins
  for (int i = 0; i < ledCtrlCount; i++)
  {
    pinMode(LEDCtrl[i], OUTPUT);
    digitalWrite(LEDCtrl[i], LOW);
  }
  pinMode(BuzzerPin, OUTPUT);
  Serial.begin(115200);
}
/**
 * @brief Main program loop.
 * @param None
 */
// --------------------------- LOOP ---------------------------
void loop()
{
  unsigned long now = millis();

  // ====== Main channel ======
  for (int i = 0; i < 4; i++)
  {
    int currentState = digitalRead(ACPin[i]);

    // Rising edge detected
    if (currentState == HIGH)
    {
      lastPulseTime[i] = now;
      electricalDetect[i] = true;
    }

    // if not detected pulse in >100ms is unpowered
    if (electricalDetect[i] && (now - lastPulseTime[i] > 100))
    {
      electricalDetect[i] = false;
      if (relayActive == i)
      {
        digitalWrite(RelayPin[i], LOW);
        relayActive = -1;
        Serial.printf("Channel %d: Powered -> relay off\r\n", i + 1);
        buzzerBeep(100);
      }
    }
  }
  relay1State = digitalRead(RelayPin[0]);
  relay2State = digitalRead(RelayPin[1]);
  relay3State = digitalRead(RelayPin[2]);
  relay4State = digitalRead(RelayPin[3]);

  // Select first powered channel
  if (relayActive == -1)
  {
    for (int i = 0; i < 3; i++)
    {
      if (electricalDetect[i])
      {
        relayActive = i;
        digitalWrite(RelayPin[i], HIGH);
        Serial.printf("Channel %d: Powered -> relay on\r\n", i + 1);
        buzzerBeep(100);
        break;
      }
    }
  }

  // if all channel is powered -> priority to channel 1
  if (electricalDetect[0] && electricalDetect[1] && electricalDetect[2] && relayActive != 0)
  {
    for (int i = 0; i < 3; i++)
      digitalWrite(RelayPin[i], LOW);

    relayActive = 0;
    digitalWrite(RelayPin[0], HIGH);
    Serial.println("All is powered -> priority to channel 1");
    buzzerBeep(200);
  }

  // ====== four independent channels  ======
  int currentState4 = digitalRead(ACPin[3]);
  static unsigned long lastPulse4 = 0;
  static bool Powered = false;

  if (currentState4 == HIGH)
  {
    lastPulse4 = now;
    Powered = true;
  }
  if (Powered && (now - lastPulse4 > 100))
    Powered = false;

  digitalWrite(RelayPin[3], Powered ? HIGH : LOW);

  // ====== DISPLAY STATUS EVERY SECOND ======
  if (now - lastDisplay >= displayInterval)
  {
    lastDisplay = now;
    Serial.println("\n================= SYSTEM STATUS =================");
    for (int i = 0; i < 4; i++)
    {
      Serial.printf("Channel %d | IN: %s | RELAY: %s\r\n",
                    i + 1,
                    (i < 3 ? (electricalDetect[i] ? "POWERED " : "UNPOWERED") : (Powered ? "POWERED " : "UNPOWERED")),
                    (digitalRead(RelayPin[i]) == HIGH) ? "ON" : "OFF");
    }
    Serial.println("========================================================");
  }
  // ====== BUTTONS CHECK ======
  if (digitalRead(ButtonPin[0]) == LOW)
  {
    buzzerBeep(200);
  }
  if (digitalRead(ButtonPin[1]) == LOW)
  {
    buzzerBeep(200);
  }
  if (digitalRead(ButtonPin[2]) == LOW)
  {
    buzzerBeep(200);
  }
  if (digitalRead(ButtonPin[3]) == LOW)
  {
    buzzerBeep(200);
  }
  if (digitalRead(ButtonPin[4]) == LOW)
  {
    buzzerBeep(200);
  }

  updateBuzzer();
}

// --------------------------- BUZZER FEATURE ---------------------------

/**
 * @brief Statrt buzzer beep for a duration.
 * @param duration Time counting by second.
 */
void buzzerBeep(unsigned long duration)
{
  if (!buzzerActive)
  {
    buzzerActive = true;
    buzzerStartTime = millis();
    buzzerDuration = duration;
    tone(BuzzerPin, toneFreq);
  }
}

/**
 * @brief check and turn off buzzer.
 */
void updateBuzzer()
{
  if (buzzerActive && millis() - buzzerStartTime >= buzzerDuration)
  {
    noTone(BuzzerPin);
    buzzerActive = false;
  }
}