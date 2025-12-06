#include <M5Unified.h>
#include <FastLED.h>

const int DISPLAY_SWITCH = 39;
const int LED_DATA_PIN = 27;
const int NUM_LEDS = 25;
const int CENTER_LED = 12;
const unsigned long DEBOUNCE_MS = 50;
const unsigned long SEND_INTERVAL_MS = 100;

CRGB leds[NUM_LEDS];

volatile bool button_state_changed = false;
volatile bool current_button_state = false;
unsigned long last_button_time = 0;
unsigned long last_send_time = 0;

void IRAM_ATTR button_isr() {
    unsigned long now = millis();
    if (now - last_button_time > DEBOUNCE_MS) {
        button_state_changed = true;
        current_button_state = (digitalRead(DISPLAY_SWITCH) == LOW);
        last_button_time = now;
    }
}

void send_button_state(bool pressed) {
    Serial.print("BTN:");
    Serial.print(pressed ? "1" : "0");
    Serial.print("\n");
    Serial.flush();
}

void update_led(bool pressed) {
    leds[CENTER_LED] = pressed ? CRGB::Blue : CRGB::Green;
    FastLED.show();
}

void setup() {
    M5.begin();

    Serial.begin(115200);

    pinMode(DISPLAY_SWITCH, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(DISPLAY_SWITCH), button_isr, CHANGE);

    FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    FastLED.clear();
    update_led(false);
}

void loop() {
    if (button_state_changed) {
        button_state_changed = false;
        send_button_state(current_button_state);
        update_led(current_button_state);
    }

    unsigned long now = millis();
    if (now - last_send_time >= SEND_INTERVAL_MS) {
        send_button_state(current_button_state);
        last_send_time = now;
    }

    delay(10);
}
