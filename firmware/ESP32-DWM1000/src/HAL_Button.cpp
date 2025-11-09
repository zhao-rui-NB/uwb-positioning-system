#include "HAL_Button.h"
#include "Arduino.h"

// === 可調參數 ===
#define DEBOUNCE_TIME    50      // 消抖時間 (ms)
#define LONG_PRESS_TIME  500     // 長按啟動時間 (ms)
#define REPEAT_INTERVAL  10     // 長按後重複觸發間隔 (ms)

// === 回傳鍵值 ===
// 0 = 無按鍵, 1 = UP, 2 = DOWN, 3 = OK

typedef struct {
    uint8_t pin;
    uint8_t state;               // 當前讀值
    uint8_t lastStableState;     // 穩定狀態
    unsigned long lastChangeTime;
    unsigned long pressStartTime;
    unsigned long lastRepeatTime;
    bool pressed;
    bool repeating;
} Button;

Button buttons[] = {
    {BTN_UP, HIGH, HIGH, 0, 0, 0, false, false},
    {BTN_DOWN, HIGH, HIGH, 0, 0, 0, false, false},
    {BTN_OK, HIGH, HIGH, 0, 0, 0, false, false}
};

uint8_t key_scan(void)
{
    unsigned long now = millis();

    for (int i = 0; i < 3; i++) {
        uint8_t reading = digitalRead(buttons[i].pin);

        // 狀態變化 → 重設消抖時間
        if (reading != buttons[i].state) {
            buttons[i].state = reading;
            buttons[i].lastChangeTime = now;
        }

        // 穩定超過消抖時間
        if ((now - buttons[i].lastChangeTime) > DEBOUNCE_TIME) {
            // 狀態變化
            if (buttons[i].lastStableState != buttons[i].state) {
                buttons[i].lastStableState = buttons[i].state;

                if (buttons[i].state == LOW) {
                    // 按下
                    buttons[i].pressStartTime = now;
                    buttons[i].lastRepeatTime = now;
                    buttons[i].pressed = true;
                    buttons[i].repeating = false;
                    return i + 1;  // 立刻回報一次
                } else {
                    // 放開
                    buttons[i].pressed = false;
                    buttons[i].repeating = false;
                }
            }

            // 若持續按住
            if (buttons[i].pressed) {
                // 到達長按啟動時間
                if (!buttons[i].repeating && (now - buttons[i].pressStartTime > LONG_PRESS_TIME)) {
                    buttons[i].repeating = true;
                    buttons[i].lastRepeatTime = now;
                    return i + 1; // 長按第一次重複發送
                }

                // 若已進入重複階段
                if (buttons[i].repeating && (now - buttons[i].lastRepeatTime > REPEAT_INTERVAL)) {
                    buttons[i].lastRepeatTime = now;
                    return i + 1; // 定期再發一次
                }
            }
        }
    }

    return 0; // 無按鍵
}


unsigned long get_current_millis(){
    return millis();
}
