#include "M5Dial.h"

const unsigned long SEND_INTERVAL_MS = 100;
const int BEEP_FREQ = 4000;        // ブザー周波数 (Hz)
const int BEEP_DURATION_MS = 50;   // 鳴動時間 (ms)

bool current_button_state = false;
unsigned long last_send_time = 0;

void send_button_state(bool pressed) {
    Serial.print("BTN:");
    Serial.print(pressed ? "1" : "0");
    Serial.print("\n");
    Serial.flush();
}

void update_display(bool pressed) {
    if (pressed) {
        M5Dial.Display.fillScreen(TFT_BLUE);
        M5Dial.Display.setTextColor(TFT_WHITE, TFT_BLUE);
        M5Dial.Display.setTextDatum(middle_center);
        M5Dial.Display.setTextSize(3);
        M5Dial.Display.drawString("PRESSED", M5Dial.Display.width() / 2, M5Dial.Display.height() / 2 - 20);
        M5Dial.Display.setTextSize(2);
        M5Dial.Display.drawString("BTN: 1", M5Dial.Display.width() / 2, M5Dial.Display.height() / 2 + 25);
    } else {
        M5Dial.Display.fillScreen(TFT_BLACK);
        M5Dial.Display.setTextColor(TFT_GREEN, TFT_BLACK);
        M5Dial.Display.setTextDatum(middle_center);
        M5Dial.Display.setTextSize(3);
        M5Dial.Display.drawString("READY", M5Dial.Display.width() / 2, M5Dial.Display.height() / 2 - 20);
        M5Dial.Display.setTextSize(2);
        M5Dial.Display.drawString("BTN: 0", M5Dial.Display.width() / 2, M5Dial.Display.height() / 2 + 25);
    }
}

void setup() {
    auto cfg = M5.config();
    M5Dial.begin(cfg, false, false);

    Serial.begin(115200);

    M5Dial.Display.setRotation(0);
    update_display(false);
}

void loop() {
    M5Dial.update();

    // 物理ボタン(中央プッシュボタン)またはタッチパネルの押下を判定
    auto t = M5Dial.Touch.getDetail();
    bool is_pressed = M5Dial.BtnA.isPressed() || t.isPressed();

    if (is_pressed != current_button_state) {
        current_button_state = is_pressed;
        send_button_state(current_button_state);
        update_display(current_button_state);

        if (current_button_state) {
            M5Dial.Speaker.tone(BEEP_FREQ, BEEP_DURATION_MS);
        }
    }

    unsigned long now = millis();
    if (now - last_send_time >= SEND_INTERVAL_MS) {
        send_button_state(current_button_state);
        last_send_time = now;
    }

    delay(10);
}
