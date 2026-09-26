#include "key.h"

#define Is_Key_Pressed()  (HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin) == GPIO_PIN_RESET)

uint8_t Is_Key_Clicked(void) {
    static uint8_t count = 0;   // 消抖计数
    static uint8_t pressed = 0; // 用于每次按键只返回一次点击事件
    if (Is_Key_Pressed() && !pressed) {
        count++;
        if (count >= 3 && Is_Key_Pressed()) {
            pressed = 1;
            return 1; // 按键被点击
        }
    }
    if (!Is_Key_Pressed()) {
        pressed = 0; // 按键释放，重置状态
        count = 0;   // 按键释放，重置消抖计数
    }
    return 0;
}


