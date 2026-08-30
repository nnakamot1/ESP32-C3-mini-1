#include "DFRobot_RGBLCD1602.h"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
}

static const char *TAG = "RGBLCD1602";

// I2C config (SCL=8, SDA=10)
#define I2C_MASTER_SCL_IO           GPIO_NUM_8
#define I2C_MASTER_SDA_IO           GPIO_NUM_10
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// Color lookup (same as original)
static const uint8_t color_define[4][3] =
{
    {255, 255, 255}, // white
    {255, 0,   0},   // red
    {0,   255, 0},   // green
    {0,   0,   255}, // blue
};

static bool s_i2c_initialized = false;

static esp_err_t i2c_master_init_if_needed(i2c_port_t port) {
    if (s_i2c_initialized) {
        return ESP_OK;
    }

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(port, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C master initialized (port=%d, SDA=%d, SCL=%d)",
             port, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return ESP_OK;
}

/* --------------------------- Constructor --------------------------- */

DFRobot_RGBLCD1602::DFRobot_RGBLCD1602(uint8_t RGBAddr,
                                       uint8_t lcdCols,
                                       uint8_t lcdRows,
                                       i2c_port_t i2c_port,
                                       uint8_t lcdAddr)
: _showFunction(0),
  _showControl(0),
  _showMode(0),
  _initialized(0),
  _numLines(0),
  _currLine(0),
  _lcdAddr(lcdAddr),
  _RGBAddr(RGBAddr),
  _cols(lcdCols),
  _rows(lcdRows),
  _i2c_port(i2c_port)
{
}

/* ----------------------------- Init ----------------------------- */

void DFRobot_RGBLCD1602::init()
{
    // I2C init
    if (i2c_master_init_if_needed(_i2c_port) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C, LCD won't work");
        return;
    }

    // Map RGB registers depending on RGBAddr (same logic as original)
    if (_RGBAddr == 0x60) {
        REG_RED   = 0x04;
        REG_GREEN = 0x03;
        REG_BLUE  = 0x02;
        REG_ONLY  = 0x02;
    } else if (_RGBAddr == (0x60 >> 1)) {
        REG_RED   = 0x06; // pwm2
        REG_GREEN = 0x07; // pwm1
        REG_BLUE  = 0x08; // pwm0
        REG_ONLY  = 0x08;
    } else if (_RGBAddr == 0x6B) {
        REG_RED   = 0x06;
        REG_GREEN = 0x05;
        REG_BLUE  = 0x04;
        REG_ONLY  = 0x04;
    } else if (_RGBAddr == 0x2D) {
        REG_RED   = 0x01;
        REG_GREEN = 0x02;
        REG_BLUE  = 0x03;
        REG_ONLY  = 0x01;
    }

    _showFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    begin(_rows);
}

/* ---------------------- High-level LCD API ---------------------- */

void DFRobot_RGBLCD1602::clear()
{
    command(LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));   // clear/home need >1.52ms
}

void DFRobot_RGBLCD1602::home()
{
    command(LCD_RETURNHOME);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void DFRobot_RGBLCD1602::noDisplay()
{
    _showControl &= ~LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _showControl);
}

void DFRobot_RGBLCD1602::display()
{
    _showControl |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _showControl);
}

void DFRobot_RGBLCD1602::stopBlink()
{
    _showControl &= ~LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _showControl);
}

void DFRobot_RGBLCD1602::blink()
{
    _showControl |= LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _showControl);
}

void DFRobot_RGBLCD1602::noCursor()
{
    _showControl &= ~LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _showControl);
}

void DFRobot_RGBLCD1602::cursor()
{
    _showControl |= LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _showControl);
}

void DFRobot_RGBLCD1602::scrollDisplayLeft()
{
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void DFRobot_RGBLCD1602::scrollDisplayRight()
{
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void DFRobot_RGBLCD1602::leftToRight()
{
    _showMode |= LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _showMode);
}

void DFRobot_RGBLCD1602::rightToLeft()
{
    _showMode &= ~LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _showMode);
}

void DFRobot_RGBLCD1602::noAutoscroll()
{
    _showMode &= ~LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _showMode);
}

void DFRobot_RGBLCD1602::autoscroll()
{
    _showMode |= LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _showMode);
}

void DFRobot_RGBLCD1602::customSymbol(uint8_t location, uint8_t charmap[])
{
    location &= 0x7; // 0-7
    command(LCD_SETCGRAMADDR | (location << 3));

    uint8_t data[9];
    data[0] = 0x40;  // data register
    for (int i = 0; i < 8; ++i) {
        data[i + 1] = charmap[i];
    }
    send(data, 9);
}

void DFRobot_RGBLCD1602::setCursor(uint8_t col, uint8_t row)
{
    if (row >= _rows) row = _rows - 1;
    uint8_t addr = (row == 0) ? (0x80 | col) : (0xC0 | col);

    uint8_t data[2] = { 0x80, addr }; // control byte + address
    send(data, 2);
}

void DFRobot_RGBLCD1602::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t temp_r, temp_g, temp_b;

    if (_RGBAddr == (0x60 >> 1)) {
        temp_r = (uint16_t)r * 192 / 255;
        temp_g = (uint16_t)g * 192 / 255;
        temp_b = (uint16_t)b * 192 / 255;
        setReg(REG_RED,   temp_r);
        setReg(REG_GREEN, temp_g);
        setReg(REG_BLUE,  temp_b);
    } else {
        setReg(REG_RED,   r);
        setReg(REG_GREEN, g);
        setReg(REG_BLUE,  b);
        if (_RGBAddr == 0x6B) {
            setReg(0x07, 0xFF);
        }
    }
}

void DFRobot_RGBLCD1602::setColor(uint8_t color)
{
    if (color > 3) return;
    setRGB(color_define[color][0],
           color_define[color][1],
           color_define[color][2]);
}

size_t DFRobot_RGBLCD1602::write(uint8_t value)
{
    uint8_t data[2] = { 0x40, value }; // data register
    send(data, 2);
    return 1;
}

void DFRobot_RGBLCD1602::command(uint8_t value)
{
    uint8_t data[2] = { 0x80, value }; // command register
    send(data, 2);
}

void DFRobot_RGBLCD1602::setBacklight(bool mode)
{
    if (mode) {
        setColorWhite();
    } else {
        closeBacklight();
    }
}

void DFRobot_RGBLCD1602::printstr(const char *s)
{
    if (!s) return;
    while (*s) {
        write((uint8_t)(*s++));
    }
}

/* ---------------------- Private helpers ---------------------- */

void DFRobot_RGBLCD1602::begin(uint8_t rows, uint8_t charSize)
{
    if (rows > 1) {
        _showFunction |= LCD_2LINE;
    }
    _numLines = rows;
    _currLine = 0;

    if ((charSize != 0) && (rows == 1)) {
        _showFunction |= LCD_5x10DOTS;
    }

    // Wait for power-up (like original ~50 ms)
    vTaskDelay(pdMS_TO_TICKS(50));

    // Function set sequence (3 times)
    command(LCD_FUNCTIONSET | _showFunction);
    vTaskDelay(pdMS_TO_TICKS(5));

    command(LCD_FUNCTIONSET | _showFunction);
    vTaskDelay(pdMS_TO_TICKS(5));

    command(LCD_FUNCTIONSET | _showFunction);

    // Display on, cursor off, blink off
    _showControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    display();

    // Clear display
    clear();

    // Entry mode: left to right, no shift
    _showMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    command(LCD_ENTRYMODESET | _showMode);

    // RGB controller init (same logic as original)
    if (_RGBAddr == (0xC0 >> 1)) {
        setReg(REG_MODE1, 0x00);
        setReg(REG_OUTPUT, 0xFF);
        setReg(REG_MODE2, 0x20);
    } else if (_RGBAddr == (0x60 >> 1)) {
        setReg(0x01, 0x00);
        setReg(0x02, 0xFF);
        setReg(0x04, 0x15);
    } else if (_RGBAddr == 0x6B) {
        setReg(0x2F, 0x00);
        setReg(0x00, 0x20);
        setReg(0x01, 0x00);
        setReg(0x02, 0x01);
        setReg(0x03, 0x04);
    }

    setColorWhite();
}

void DFRobot_RGBLCD1602::send(uint8_t *data, uint8_t len)
{
    if (!data || len == 0) return;
    esp_err_t ret = transmit(_lcdAddr, data, len);
    if (ret != ESP_OK) {
//        ESP_LOGE(TAG, "send() LCD I2C error: %s", esp_err_to_name(ret));
    }
}

void DFRobot_RGBLCD1602::setReg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2] = { addr, data };
    esp_err_t ret = transmit(_RGBAddr, buf, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setReg(0x%02X) I2C error: %s",
                 addr, esp_err_to_name(ret));
    }
}

esp_err_t DFRobot_RGBLCD1602::transmit(uint8_t addr, const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_master_write_to_device(_i2c_port,
                                               addr,
                                               data,
                                               len,
                                               pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
 //       ESP_LOGW(TAG, "I2C transmit to 0x%02X failed: %s",
  //               addr, esp_err_to_name(ret));
    }
    // Small delay to avoid hammering the bus
    esp_rom_delay_us(100);
    return ret;
}
