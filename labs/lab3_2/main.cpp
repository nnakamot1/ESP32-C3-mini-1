#include "DFRobot_RGBLCD1602.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void)
{
    // RGB = 0x62, LCD = 0x3E, 16x2 display, I2C0
    DFRobot_RGBLCD1602 lcd(0x2D, 16, 2, I2C_NUM_0, (0x7c >> 1));

    lcd.init();
    lcd.setBacklight(true);     // turn on backlight (white)
    lcd.setRGB(0, 255, 0);      // or set a color, e.g., green

    lcd.clear();

    // First line
    lcd.setCursor(0, 0);
    lcd.printstr("Hello CSE121!");

    // Second line
    lcd.setCursor(0, 1);
    lcd.printstr("Nakamoto");

    // Keep task alive
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
