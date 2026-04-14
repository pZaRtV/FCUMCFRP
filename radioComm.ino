//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 4/14/2026
//Version: B-1.4.8

// Modified by: Patrick Andrasena T.
// Fixes applied (B-1.4.5 -> B-1.4.8):
//   FIX 1: Removed channel_N_raw = 0 wipe on sync detection
//   FIX 2: Corrected sync detection threshold to >2500 µs
//   FIX 3: Removed all Serial.print() from ISR
//   FIX 4: channel_N_raw and ppm_isr_count declared volatile
//   FIX 5: togglePPMDebug() removed from main loop
//   FIX 6: Removed no-op map() call in ISR
//   FIX 7: ppm_counter reset on first sync as well as subsequent ones
//   FIX 8: PPM pin set to plain INPUT (not INPUT_PULLUP)
//   FIX 9: ppm_counter declared volatile
//  FIX 10: getCh1-6 guard changed to USE_PWM_RX only (removed USE_PWM_PASSTHROUGH_CH1)
//  FIX 11: Added IBUS receiver support with proper extern declarations
//  FIX 12: Fixed debugPPMSignal() function scope issues for multi-receiver support

//========================================================================================================================//

#include "quad.h"

#if defined USE_IBUS_RX
  IBUS ibus(IBUS_SERIAL);
  uint16_t ibusChannels[14];
  bool ibusFailSafe  = false;
  bool ibusLostFrame = false;
#endif

// Function declarations
void getPPM();
unsigned long getRadioPWM(int channel);
void togglePPMDebug();

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
unsigned long rising_edge_start;

// All variables written in the ISR must be volatile.
volatile unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
volatile int ppm_counter = 0;

unsigned long time_ms = 0;
volatile unsigned long ppm_isr_count = 0;
volatile bool ppm_debug_enabled = false;

void radioSetup() {
  #if defined USE_PPM_RX
    // FIX 8: Plain INPUT — not INPUT_PULLUP.
    // INPUT_PULLUP holds the pin HIGH via a ~47k resistor, fighting the receiver's
    // LOW pulses and preventing edges from being detected on weak receivers.
    // Standard PPM idles HIGH and pulses LOW → plain INPUT is correct.
    // If your RX uses inverted PPM (idles LOW, pulses HIGH) use INPUT_PULLDOWN.
    pinMode(PPM_Pin, INPUT);
    delay(20);

    Serial.print("PPM Pin: ");
    Serial.println(PPM_Pin);
    Serial.print("Interrupt capable: ");
    Serial.println(digitalPinToInterrupt(PPM_Pin));

    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);
    Serial.println("PPM interrupt attached successfully");

  #elif defined USE_PWM_RX
    pinMode(ch1Pin, INPUT_PULLUP);
    pinMode(ch2Pin, INPUT_PULLUP);
    pinMode(ch3Pin, INPUT_PULLUP);
    pinMode(ch4Pin, INPUT_PULLUP);
    pinMode(ch5Pin, INPUT_PULLUP);
    pinMode(ch6Pin, INPUT_PULLUP);
    delay(20);
    attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
    delay(20);

  #elif defined USE_SBUS_RX
    sbus.begin();

  #elif defined USE_ELRS_RX
    elrs_rx.begin();

  #elif defined USE_DSM_RX
    Serial3.begin(115000);
  
  #elif defined USE_IBUS_RX
  ibus.begin();
    
  #else
    #error No RX type defined...
  #endif
}

unsigned long getRadioPWM(int ch_num) {
  #if defined USE_ELRS_RX
    elrs_rx.update();
    uint16_t pwm = elrs_rx.getChannel(ch_num);
    return constrain(pwm, 1000, 2000);

  #elif defined USE_IBUS_RX
    // i-BUS returns 1000-2000 directly from the IBUS library
    if (ch_num == 1) return ibusChannels[IBUS_MAP_CH1 - 1];
    else if (ch_num == 2) return ibusChannels[IBUS_MAP_CH2 - 1];
    else if (ch_num == 3) return ibusChannels[IBUS_MAP_CH3 - 1];
    else if (ch_num == 4) return ibusChannels[IBUS_MAP_CH4 - 1];
    else if (ch_num == 5) return ibusChannels[IBUS_MAP_CH5 - 1];
    else if (ch_num == 6) return ibusChannels[IBUS_MAP_CH6 - 1];
    else return 1500;

  #elif defined USE_PPM_RX || defined USE_PWM_RX || defined USE_SBUS_RX || defined USE_DSM_RX
    if (ch_num == 1) return channel_1_raw;
    else if (ch_num == 2) return channel_2_raw;
    else if (ch_num == 3) return channel_3_raw;
    else if (ch_num == 4) return channel_4_raw;
    else if (ch_num == 5) return channel_5_raw;
    else if (ch_num == 6) return channel_6_raw;
    else return 1500;

  #else
    return 1500;
  #endif
}

// Call once manually (e.g. from setup) for bench debugging.
// Do NOT call inside loop() — it will oscillate the flag every 500 ms.
void togglePPMDebug() {
  static unsigned long last_toggle_time = 0;
  if (micros() - last_toggle_time > 500000) {
    ppm_debug_enabled = !ppm_debug_enabled;
    Serial.print("PPM ISR debug ");
    Serial.println(ppm_debug_enabled ? "ENABLED" : "DISABLED");
    last_toggle_time = micros();
  }
}

void serialEvent3(void) {
  #if defined USE_DSM_RX
    while (Serial3.available()) {
      DSM.handleSerialEvent(Serial3.read(), micros());
    }
  #endif
}

//========================================================================================================================//
// INTERRUPT SERVICE ROUTINES

#if defined USE_PPM_RX || defined USE_HYBRID_RX
void getPPM() {
  ppm_isr_count++;

  static unsigned long last_rising_edge = 0;
  static int last_pin_state = LOW;

  unsigned long current_time = micros();
  int current_pin_state = digitalRead(PPM_Pin);

  if (current_pin_state == HIGH && last_pin_state == LOW) {
    unsigned long pulse_position = current_time - last_rising_edge;

    // Skip first edge after boot
    if (last_rising_edge == 0) {
      last_rising_edge = current_time;
      last_pin_state = current_pin_state;
      return;
    }

    #ifndef PPM_SYNC_MIN_US
      #define PPM_SYNC_MIN_US 2500UL
    #endif
    #ifndef PPM_CH_MIN_US
      #define PPM_CH_MIN_US   700UL
    #endif
    #ifndef PPM_CH_MAX_US
      #define PPM_CH_MAX_US   2300UL
    #endif

    if (pulse_position >= PPM_SYNC_MIN_US) {
      // New frame — reset counter, keep existing channel values intact
      ppm_counter = 0;

    } else if (pulse_position >= PPM_CH_MIN_US && pulse_position <= PPM_CH_MAX_US) {
      ppm_counter++;
      unsigned long ch_value = constrain(pulse_position, 1000UL, 2000UL);

      if      (ppm_counter == 1) channel_1_raw = ch_value;
      else if (ppm_counter == 2) channel_2_raw = ch_value;
      else if (ppm_counter == 3) channel_3_raw = ch_value;
      else if (ppm_counter == 4) channel_4_raw = ch_value;
      else if (ppm_counter == 5) channel_5_raw = ch_value;
      else if (ppm_counter == 6) channel_6_raw = ch_value;
    }

    last_rising_edge = current_time;
  }

  last_pin_state = current_pin_state;
}
#endif // USE_PPM_RX || USE_HYBRID_RX

// FIX 10: Guard is USE_PWM_RX only — USE_PWM_PASSTHROUGH_CH1 removed.
// When USE_PPM_RX is active, getCh1-6 must NOT be compiled or they will
// conflict with PPM writes to channel_N_raw.
#if defined USE_PWM_RX
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) rising_edge_start_1 = micros();
  else if(trigger == 0) channel_1_raw = micros() - rising_edge_start_1;
}
void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) rising_edge_start_2 = micros();
  else if(trigger == 0) channel_2_raw = micros() - rising_edge_start_2;
}
void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) rising_edge_start_3 = micros();
  else if(trigger == 0) channel_3_raw = micros() - rising_edge_start_3;
}
void getCh4() {
  int trigger = digitalRead(ch4Pin);
  if(trigger == 1) rising_edge_start_4 = micros();
  else if(trigger == 0) channel_4_raw = micros() - rising_edge_start_4;
}
void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) rising_edge_start_5 = micros();
  else if(trigger == 0) channel_5_raw = micros() - rising_edge_start_5;
}
void getCh6() {
  int trigger = digitalRead(ch6Pin);
  if(trigger == 1) rising_edge_start_6 = micros();
  else if(trigger == 0) channel_6_raw = micros() - rising_edge_start_6;
}
#endif // USE_PWM_RX
