#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "driver/i2c.h"
#include "esp_err.h"
#ifdef __cplusplus
}
#endif

// Same I2C LCD address macro as original
#define LCD_ADDRESS     (0x7c >> 1)

// Color IDs
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

// RGB controller registers
#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

// HD44780 command set
#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

// Flags for entry mode
#define LCD_ENTRYRIGHT           0x00
#define LCD_ENTRYLEFT            0x02
#define LCD_ENTRYSHIFTINCREMENT  0x01
#define LCD_ENTRYSHIFTDECREMENT  0x00

// Flags for display on/off control
#define LCD_DISPLAYON   0x04
#define LCD_DISPLAYOFF  0x00
#define LCD_CURSORON    0x02
#define LCD_CURSOROFF   0x00
#define LCD_BLINKON     0x01
#define LCD_BLINKOFF    0x00

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE  0x00
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00

// Flags for function set
#define LCD_8BITMODE    0x10
#define LCD_4BITMODE    0x00
#define LCD_2LINE       0x08
#define LCD_1LINE       0x00
#define LCD_5x10DOTS    0x04
#define LCD_5x8DOTS     0x00

class DFRobot_RGBLCD1602 {
public:
    // Constructor — same idea as original, but explicitly takes i2c_port
    DFRobot_RGBLCD1602(uint8_t RGBAddr,
                       uint8_t lcdCols = 16,
                       uint8_t lcdRows = 2,
                       i2c_port_t i2c_port = I2C_NUM_0,
                       uint8_t lcdAddr = LCD_ADDRESS);

    // Initialize LCD + RGB and I2C bus
    void init();

    void clear();
    void home();

    void noDisplay();
    void display();

    void stopBlink();
    void blink();

    void noCursor();
    void cursor();

    void scrollDisplayLeft();
    void scrollDisplayRight();

    void leftToRight();
    void rightToLeft();

    void noAutoscroll();
    void autoscroll();

    void customSymbol(uint8_t location, uint8_t charmap[]);

    // col: 0-15, row: 0-(rows-1)
    void setCursor(uint8_t col, uint8_t row);

    // Backlight RGB (0-255 each)
    void setRGB(uint8_t r, uint8_t g, uint8_t b);

    // Backlight PWM single channel (REG_RED / REG_GREEN / REG_BLUE)
    void setPWM(uint8_t color, uint8_t pwm) {
        setReg(color, pwm);
        if (_RGBAddr == 0x6B) {
            setReg(0x07, pwm);
        }
    }

    // Backlight color presets (WHITE, RED, GREEN, BLUE)
    void setColor(uint8_t color);

    void closeBacklight() { setRGB(0, 0, 0); }
    void setColorWhite() { setRGB(255, 255, 255); }

    // Single character write (used by printstr)
    size_t write(uint8_t data);

    // Send a command byte
    void command(uint8_t data);

    // Backlight on/off (white / off)
    void setBacklight(bool mode);

    // Simple string print helper (non-Arduino)
    void printstr(const char *s);

private:
    void begin(uint8_t rows, uint8_t charSize = LCD_5x8DOTS);

    // Send data bytes to LCD (_lcdAddr)
    void send(uint8_t *data, uint8_t len);

    // Write register on RGB controller (_RGBAddr)
    void setReg(uint8_t addr, uint8_t data);

    // Low-level I2C transmit helper
    esp_err_t transmit(uint8_t addr, const uint8_t *data, size_t len);

    // State
    uint8_t    _showFunction;
    uint8_t    _showControl;
    uint8_t    _showMode;
    uint8_t    _initialized;
    uint8_t    _numLines;
    uint8_t    _currLine;
    uint8_t    _lcdAddr;
    uint8_t    _RGBAddr;
    uint8_t    _cols;
    uint8_t    _rows;
    i2c_port_t _i2c_port;

public:
    // Same public members as original for register mapping
    uint8_t REG_RED   = 0;
    uint8_t REG_GREEN = 0;
    uint8_t REG_BLUE  = 0;
    uint8_t REG_ONLY  = 0;
};
