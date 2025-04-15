#include "esp_timer.h"

// === 4-bit Sine Wave Table (0 to 15) ===
const uint8_t SineWave[16] = { 6, 9, 11, 13, 15, 14, 13, 11, 9, 7, 5, 3, 1, 0, 2, 4 };
volatile uint8_t Index = 0;
volatile bool waveformActive = false;

// === DAC output pins (simulate 4-bit DAC on GPIO 0, 1, 2, 4) ===
const int DAC_PINS[4] = {0, 1, 2, 3};

// === Piano key input pins ===
const int KEY_PINS[4] = {6, 7, 8, 9};

// === Timer periods for different notes (in microseconds) ===
const uint16_t timerPeriods[4] = {120, 106, 95, 80};  // ~Freqs: A4, B4, C5, E5

// === Timer handle ===
esp_timer_handle_t periodic_timer;

// === Initialize DAC Pins ===
void DAC_Init() {
  for (int i = 0; i < 4; i++) {
    pinMode(DAC_PINS[i], OUTPUT);
    digitalWrite(DAC_PINS[i], LOW);
  }
}

// === Output 4-bit Value to DAC ===
void DAC_Out(uint8_t value) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(DAC_PINS[i], (value >> i) & 0x01);
  }
}

// === Timer callback function ===
void onTimer(void* arg) {
  if (waveformActive) {
    Index = (Index + 1) & 0x0F;
    DAC_Out(SineWave[Index]);
  }
}

// === Setup Function ===
void setup() {
  Serial.begin(115200);
  DAC_Init();

  for (int i = 0; i < 4; i++) {
    pinMode(KEY_PINS[i], INPUT);
  }

  const esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "sine_timer"
  };

  esp_timer_create(&timer_args, &periodic_timer);
  esp_timer_start_periodic(periodic_timer, timerPeriods[0]); // Start with default
}

// === Main Loop ===
void loop() {
  bool keyPressed = false;

  for (int i = 0; i < 4; i++) {
    if (digitalRead(KEY_PINS[i]) == HIGH) {
      waveformActive = true;
      esp_timer_stop(periodic_timer);  // Must stop before changing period
      esp_timer_start_periodic(periodic_timer, timerPeriods[i]);
      keyPressed = true;
      break;  // Play only one tone at a time
    }
  }

  if (!keyPressed) {
    waveformActive = false;
    DAC_Out(0); // Silence
  }

  delay(1);  // Light debounce
}