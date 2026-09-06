// lab5_loopback — self-contained Morse code test: no Raspberry Pi needed.
//
// Runs BOTH sides of lab5_2's optical Morse link on a single ESP32-C3:
//   - A transmitter task blinks an LED (GPIO4 by default) in Morse timing,
//     repeating TX_MESSAGE forever.
//   - The same photoresistor receiver from lab5_2 (ADC1 channel 3 / GPIO3)
//     decodes whatever light it sees and prints recognized letters.
//   - An LCD1602 (from lab3) mirrors what the serial monitor shows: the
//     rolling decoded text on line 1, and the live raw ADC reading /
//     light state on line 2 — so you can watch the receiver work without
//     needing to open a serial monitor at all.
//
// Wiring (same breadboard components as lab5_2/lab5_3, no Pi required):
//   - Photoresistor divider: 3.3V -> photoresistor -> [node -> GPIO3] ->
//     resistor -> GND. More light must raise the ADC reading at the node;
//     if your resistor is on the other leg, the light/dark logic inverts.
//   - LED: 3.3V-capable GPIO (GPIO4 by default) -> LED anode (long leg);
//     LED cathode (short leg) -> resistor -> GND.
//   - Point the LED directly at the photoresistor, close together, and
//     shield both from ambient light (e.g. under a cup or dark cloth) so
//     the sensor only sees the LED.
//   - LCD1602 (DFRobot RGB backlight): I2C_NUM_0, SCL=GPIO0, SDA=GPIO1 —
//     same wiring as lab3_2/lab3_3, no conflict with GPIO3/GPIO4 above.
//
// Set via build_flags in platformio.ini (both optional):
//   -DTX_MESSAGE=\"SOS\"
//   -DLED_GPIO_NUM=4

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
}

#include "DFRobot_RGBLCD1602.h"

static const char *TAG = "MORSE_LOOPBACK";

#ifndef TX_MESSAGE
#define TX_MESSAGE "SOS"
#endif
#ifndef LED_GPIO_NUM
#define LED_GPIO_NUM 4
#endif

/* -------------------------------------------------------------------------- */
/*                        Hardware / ADC configuration                        */
/* -------------------------------------------------------------------------- */

#define MORSE_ADC_UNIT       ADC_UNIT_1
#define MORSE_ADC_CHANNEL    ADC_CHANNEL_3      // GPIO3 on ESP32-C3
#define MORSE_ADC_ATTEN      ADC_ATTEN_DB_12
#define MORSE_ADC_BITWIDTH   ADC_BITWIDTH_DEFAULT

/* -------------------------------------------------------------------------- */
/*                             Timing configuration                            */
/* -------------------------------------------------------------------------- */

#define MORSE_UNIT_MS        55    // length of a "dot" — shared by TX and RX.
                                    // With the LCD sharing the breadboard,
                                    // 20/50/52ms all decode as garbage but
                                    // 55ms+ is clean (52 fails, 55 works —
                                    // the boundary is right there). lab5_2/
                                    // lab5_3 (no LCD) run cleanly at their
                                    // original 20ms/18ms spec once the same
                                    // THRESHOLD_MARGIN fix is applied; here
                                    // the LCD's presence still costs some
                                    // margin, so this stays at 55ms plus the
                                    // first-pulse compensation below.
#define SAMPLE_PERIOD_MS     2     // RX sampling period — 10 samples per dot
#define CALIBRATION_SAMPLES  200

// Classification thresholds (in units of MORSE_UNIT_MS)
#define DOT_MAX_UNITS        2     // <= 2 units -> dot
#define DASH_MIN_UNITS       3     // >= 3 units -> dash
#define MAX_DASH_UNITS       5     // > 5 units ON is considered noise / invalid

#define LETTER_GAP_UNITS     3     // >= 3 units of OFF -> end of letter
#define WORD_GAP_UNITS       7     // >= 7 units of OFF -> space between words
#define MESSAGE_GAP_UNITS    9     // >= 9 units -> end of phrase (newline)

// ADC thresholding
#define THRESHOLD_MARGIN     20    // counts above dark baseline

// Ignore very short pulses/gaps (units < MIN_VALID_UNITS)
#define MIN_VALID_UNITS      1

// Cap pattern length to avoid runaway patterns
#define MAX_SYMBOLS_PER_CHAR 5

// Require light level to be stable for a few samples before toggling.
// In *samples*, not ms — 4 samples = 8ms at the new 2ms rate, close to the
// old 20ms-rate debounce's 2x10ms=20ms in relative terms would be too slow
// now, so this is tuned down to stay a small fraction of one dot (20ms).
#define LIGHT_STABLE_SAMPLES 3

// Gap between repeats of TX_MESSAGE, and between words within it
#define TX_REPEAT_GAP_UNITS  7
#define TX_WORD_GAP_UNITS    7
#define TX_LETTER_GAP_UNITS  3
#define TX_SYMBOL_GAP_UNITS  1

/* -------------------------------------------------------------------------- */
/*                        LCD — mirrors the serial monitor                    */
/* -------------------------------------------------------------------------- */

static DFRobot_RGBLCD1602 *g_lcd = nullptr;

// Rolling window of the last 32 decoded characters (letters, word/phrase
// gaps), same content the serial monitor shows via putchar() — split across
// both LCD lines (16 chars each) instead of a fast-changing status readout,
// since that updates far too quickly to actually read on a 2-line display.
#define DECODED_CAPACITY 32
static char g_decoded[DECODED_CAPACITY + 1] = {0};
static int  g_decoded_len = 0;

static void lcd_show_line(int row, const char *text)
{
    if (!g_lcd) return;
    char padded[17];
    snprintf(padded, sizeof(padded), "%-16s", text);  // pad/truncate to 16 chars
    g_lcd->setCursor(0, row);
    g_lcd->printstr(padded);
}

static void lcd_render_decoded(void)
{
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "%-16.16s", g_decoded);
    snprintf(line1, sizeof(line1), "%-16.16s", g_decoded + 16);
    lcd_show_line(0, line0);
    lcd_show_line(1, line1);
}

// Appends a decoded character to the rolling 32-char buffer, scrolling left
// once it's full — mirrors the serial monitor's running transcript.
static void lcd_push_decoded_char(char c)
{
    if (g_decoded_len < DECODED_CAPACITY) {
        g_decoded[g_decoded_len++] = c;
        g_decoded[g_decoded_len] = '\0';
    } else {
        memmove(g_decoded, g_decoded + 1, DECODED_CAPACITY - 1);
        g_decoded[DECODED_CAPACITY - 1] = c;
        g_decoded[DECODED_CAPACITY] = '\0';
    }
    lcd_render_decoded();
}

static void lcd_reset_decoded(void)
{
    g_decoded_len = 0;
    g_decoded[0] = '\0';
    lcd_render_decoded();
}

/* -------------------------------------------------------------------------- */
/*                              Morse lookup table                            */
/* -------------------------------------------------------------------------- */

typedef struct {
    const char *pattern;
    char        ch;
} morse_entry_t;

static const morse_entry_t MORSE_TABLE[] = {
    // Letters
    {".-",   'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..",  'D'}, {".",    'E'},
    {"..-.", 'F'}, {"--.",  'G'}, {"....", 'H'}, {"..",   'I'}, {".---", 'J'},
    {"-.-",  'K'}, {".-..", 'L'}, {"--",   'M'}, {"-.",   'N'}, {"---",  'O'},
    {".--.", 'P'}, {"--.-", 'Q'}, {".-.",  'R'}, {"...",  'S'}, {"-",    'T'},
    {"..-",  'U'}, {"...-", 'V'}, {".--",  'W'}, {"-..-", 'X'}, {"-.--", 'Y'},
    {"--..", 'Z'},

    // Digits
    {"-----",'0'}, {".----",'1'}, {"..---",'2'}, {"...--",'3'}, {"....-",'4'},
    {".....",'5'}, {"-....",'6'}, {"--...",'7'}, {"---..",'8'}, {"----.",'9'},

    {NULL,   0}
};

static char morse_lookup(const char *pattern)
{
    for (int i = 0; MORSE_TABLE[i].pattern != NULL; i++) {
        if (strcmp(MORSE_TABLE[i].pattern, pattern) == 0) {
            return MORSE_TABLE[i].ch;
        }
    }
    return 0;
}

static const char *morse_encode(char c)
{
    c = (char) toupper((unsigned char) c);
    for (int i = 0; MORSE_TABLE[i].pattern != NULL; i++) {
        if (MORSE_TABLE[i].ch == c) {
            return MORSE_TABLE[i].pattern;
        }
    }
    return NULL;
}

/* -------------------------------------------------------------------------- */
/*                         Transmitter (drives the LED)                       */
/* -------------------------------------------------------------------------- */

static const gpio_num_t kLedGpio = (gpio_num_t) LED_GPIO_NUM;

static void led_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << kLedGpio);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(kLedGpio, 0);
}

static void tx_blink_symbol(char sym)
{
    if (sym == '.') {
        gpio_set_level(kLedGpio, 1);
        vTaskDelay(pdMS_TO_TICKS(MORSE_UNIT_MS));
        gpio_set_level(kLedGpio, 0);
    } else if (sym == '-') {
        gpio_set_level(kLedGpio, 1);
        vTaskDelay(pdMS_TO_TICKS(3 * MORSE_UNIT_MS));
        gpio_set_level(kLedGpio, 0);
    }
}

// Blinks message once (words separated by spaces), using the standard Morse
// timing ratios: 1u between symbols, 3u between letters, 7u between words.
static void tx_send_message(const char *message)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s", message);

    char *saveptr = NULL;
    char *word = strtok_r(buf, " ", &saveptr);
    bool first_word = true;

    while (word) {
        if (!first_word) {
            vTaskDelay(pdMS_TO_TICKS(TX_WORD_GAP_UNITS * MORSE_UNIT_MS));
        }
        first_word = false;

        for (int li = 0; word[li] != '\0'; li++) {
            if (li > 0) {
                vTaskDelay(pdMS_TO_TICKS(TX_LETTER_GAP_UNITS * MORSE_UNIT_MS));
            }
            const char *pattern = morse_encode(word[li]);
            if (!pattern) {
                continue;
            }
            for (int si = 0; pattern[si] != '\0'; si++) {
                if (si > 0) {
                    vTaskDelay(pdMS_TO_TICKS(TX_SYMBOL_GAP_UNITS * MORSE_UNIT_MS));
                }
                tx_blink_symbol(pattern[si]);
            }
        }

        word = strtok_r(NULL, " ", &saveptr);
    }
}

static void morse_tx_task(void *arg)
{
    (void) arg;
    led_init();

    ESP_LOGI(TAG, "Transmitting \"%s\" on GPIO%d, unit=%dms", TX_MESSAGE, (int) kLedGpio, MORSE_UNIT_MS);

    // Give the receiver task time to finish its dark-baseline calibration
    // before the LED starts blinking.
    vTaskDelay(pdMS_TO_TICKS(CALIBRATION_SAMPLES * SAMPLE_PERIOD_MS + 500));

    while (1) {
        tx_send_message(TX_MESSAGE);
        vTaskDelay(pdMS_TO_TICKS(TX_REPEAT_GAP_UNITS * MORSE_UNIT_MS));
    }
}

/* -------------------------------------------------------------------------- */
/*                       Receiver (reads the photoresistor)                   */
/* -------------------------------------------------------------------------- */

static adc_oneshot_unit_handle_t adc_handle;
static int dark_baseline = 0;
static int light_threshold = 0;

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = MORSE_ADC_UNIT;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = MORSE_ADC_ATTEN;
    chan_cfg.bitwidth = MORSE_ADC_BITWIDTH;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MORSE_ADC_CHANNEL, &chan_cfg));
}

static void adc_calibrate_dark(void)
{
    ESP_LOGI(TAG, "Calibrating dark baseline (%d samples). Keep LED OFF.", CALIBRATION_SAMPLES);
    lcd_show_line(0, "Calibrating...");
    lcd_show_line(1, "Keep LED off");
    int sum = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, MORSE_ADC_CHANNEL, &raw) == ESP_OK) {
            sum += raw;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    dark_baseline   = sum / CALIBRATION_SAMPLES;
    light_threshold = dark_baseline + THRESHOLD_MARGIN;

    ESP_LOGI(TAG, "Dark baseline=%d, threshold=%d", dark_baseline, light_threshold);
}

// Logs the raw ADC reading to the serial monitor every ~500ms for debugging.
// This does NOT go to the LCD — it updates far too fast to read on a 2-line
// display; the LCD instead shows the decoded text transcript (see
// lcd_push_decoded_char / lcd_render_decoded above).
#define RAW_LOG_PERIOD_SAMPLES (500 / SAMPLE_PERIOD_MS)

static bool read_light_state(void)
{
    static bool logical_light   = false;
    static int  above_count     = 0;
    static int  below_count     = 0;
    static int  log_counter     = 0;

    int raw = 0;
    if (adc_oneshot_read(adc_handle, MORSE_ADC_CHANNEL, &raw) != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed");
        return logical_light;
    }

    if (++log_counter >= RAW_LOG_PERIOD_SAMPLES) {
        log_counter = 0;
        ESP_LOGI(TAG, "raw=%d threshold=%d light=%s", raw, light_threshold, logical_light ? "ON" : "off");
    }

    if (raw > light_threshold) {
        above_count++;
        below_count = 0;

        if (!logical_light && above_count >= LIGHT_STABLE_SAMPLES) {
            logical_light = true;
            ESP_LOGD(TAG, "Light state -> ON (raw=%d)", raw);
        }
    } else {
        below_count++;
        above_count = 0;

        if (logical_light && below_count >= LIGHT_STABLE_SAMPLES) {
            logical_light = false;
            ESP_LOGD(TAG, "Light state -> OFF (raw=%d)", raw);
        }
    }

    return logical_light;
}

static void flush_letter(char *symbol_buf, int *len)
{
    if (*len == 0) {
        return;
    }

    symbol_buf[*len] = '\0';

    int dot_count  = 0;
    int dash_count = 0;
    for (int i = 0; i < *len; i++) {
        if (symbol_buf[i] == '.') {
            dot_count++;
        } else if (symbol_buf[i] == '-') {
            dash_count++;
        }
    }

    char c = morse_lookup(symbol_buf);

    if (*len == 5 && dash_count <= 1) {
        c = 0; // treat as unknown / noise
    }

    if (c != 0) {
        putchar(c);
        fflush(stdout);
        lcd_push_decoded_char(c);
    }

    ESP_LOGI(TAG, "LETTER: \"%s\" -> '%c'", symbol_buf, c ? c : '?');

    *len = 0;
}

// Set whenever the preceding OFF gap was a LETTER_GAP or longer. The very
// first ON pulse after such a gap tends to measure a bit short, enough to
// occasionally drop a dot — add a small compensation before classification.
static int g_pending_compensation_ms = 0;

static void handle_on_time(int on_time_ms, char *symbol_buf, int *len)
{
    if (on_time_ms <= 0) {
        return;
    }

    if (g_pending_compensation_ms > 0) {
        on_time_ms += g_pending_compensation_ms;
        g_pending_compensation_ms = 0;
    }

    int units = (on_time_ms + MORSE_UNIT_MS / 2) / MORSE_UNIT_MS;

    if (units < MIN_VALID_UNITS) {
        ESP_LOGD(TAG, "Ignoring short ON pulse: %d ms (~%d units)", on_time_ms, units);
        return;
    }

    if (units > MAX_DASH_UNITS) {
        ESP_LOGW(TAG,
                 "Very long ON pulse: %d ms (~%d units) - treating as noise, flushing \"%s\"",
                 on_time_ms, units, symbol_buf);
        flush_letter(symbol_buf, len);
        return;
    }

    char sym = (units <= DOT_MAX_UNITS) ? '.' : '-';

    if (*len >= MAX_SYMBOLS_PER_CHAR) {
        ESP_LOGW(TAG,
                 "Symbol buffer full (%d, \"%s\"), flushing before adding '%c'",
                 *len, symbol_buf, sym);
        flush_letter(symbol_buf, len);
    }

    if (*len < MAX_SYMBOLS_PER_CHAR) {
        symbol_buf[*len] = sym;
        (*len)++;
        symbol_buf[*len] = '\0';
    }

    ESP_LOGD(TAG, "PULSE: %d ms (~%d units) -> '%c', pattern=\"%s\"",
             on_time_ms, units, sym, symbol_buf);
}

static void handle_off_time(int off_time_ms, char *symbol_buf, int *len)
{
    if (off_time_ms <= 0) {
        return;
    }

    int units = (off_time_ms + MORSE_UNIT_MS / 2) / MORSE_UNIT_MS;

    if (units < MIN_VALID_UNITS) {
        ESP_LOGD(TAG, "Ignoring short OFF gap: %d ms (~%d units)", off_time_ms, units);
        return;
    }

    if (units >= LETTER_GAP_UNITS) {
        g_pending_compensation_ms = 50;
    }

    if (units >= MESSAGE_GAP_UNITS) {
        if (*len > 0) {
            flush_letter(symbol_buf, len);
        }
        putchar('\n');
        fflush(stdout);
        lcd_reset_decoded();  // mirrors the monitor starting a fresh line
        ESP_LOGI(TAG,
                 "PHRASE GAP CONFIRMED: %d ms (~%d units) -> end of phrase",
                 off_time_ms, units);
    } else if (units >= WORD_GAP_UNITS) {
        if (*len > 0) {
            flush_letter(symbol_buf, len);
        }
        putchar(' ');
        fflush(stdout);
        lcd_push_decoded_char(' ');
        ESP_LOGI(TAG,
                 "WORD GAP CONFIRMED: %d ms (~%d units) -> new word",
                 off_time_ms, units);
    } else if (units >= LETTER_GAP_UNITS) {
        if (*len > 0) {
            flush_letter(symbol_buf, len);
            ESP_LOGI(TAG,
                     "LETTER GAP CONFIRMED: %d ms (~%d units) -> next letter",
                     off_time_ms, units);
        } else {
            ESP_LOGD(TAG,
                     "LETTER GAP (no pending letter): %d ms (~%d units)",
                     off_time_ms, units);
        }
    } else {
        if (*len > 0) {
            ESP_LOGD(TAG, "SYMBOL GAP: %d ms (~%d units)", off_time_ms, units);
        } else {
            ESP_LOGD(TAG, "IDLE GAP: %d ms (~%d units) with empty buffer", off_time_ms, units);
        }
    }
}

static void morse_rx_task(void *arg)
{
    (void) arg;

    adc_init();
    adc_calibrate_dark();
    lcd_reset_decoded();

    bool prev_light        = false;
    int  on_time_ms        = 0;
    int  off_time_ms       = 0;
    bool phrase_gap_emitted = false;

    char symbol_buf[16]    = {0};
    int  symbol_len        = 0;

    ESP_LOGI(TAG, "Starting Morse decoder (time-based).");

    while (1) {
        bool light = read_light_state();

        if (light) {
            on_time_ms += SAMPLE_PERIOD_MS;

            if (!prev_light && off_time_ms > 0) {
                handle_off_time(off_time_ms, symbol_buf, &symbol_len);
                off_time_ms       = 0;
                phrase_gap_emitted = false;
            }
        } else {
            off_time_ms += SAMPLE_PERIOD_MS;

            if (prev_light && on_time_ms > 0) {
                handle_on_time(on_time_ms, symbol_buf, &symbol_len);
                on_time_ms = 0;
            }

            int units = (off_time_ms + MORSE_UNIT_MS / 2) / MORSE_UNIT_MS;
            if (!phrase_gap_emitted && units >= MESSAGE_GAP_UNITS) {
                handle_off_time(off_time_ms, symbol_buf, &symbol_len);
                phrase_gap_emitted = true;
                off_time_ms        = 0;
            }
        }

        prev_light = light;

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* -------------------------------------------------------------------------- */
/*                                  app_main                                  */
/* -------------------------------------------------------------------------- */

extern "C" void app_main(void)
{
    // RGBAddr = 0x2D, LCD on I2C_NUM_0 (SDA=1, SCL=0) — same wiring as lab3_3.
    static DFRobot_RGBLCD1602 lcd(/*RGBAddr*/ 0x2D, /*cols*/ 16, /*rows*/ 2,
                                  I2C_NUM_0, LCD_ADDRESS);
    lcd.init();
    lcd.setBacklight(true);
    lcd.setColorWhite();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printstr("Morse Loopback");
    lcd.setCursor(0, 1);
    lcd.printstr("Starting...");
    g_lcd = &lcd;

    // RX gets a higher priority than TX: both need tight sub-20ms timing on
    // this single-core chip, and RX missing a sample is worse (corrupts a
    // pulse-width measurement) than TX being delayed a tick or two.
    xTaskCreate(morse_rx_task, "morse_rx_task", 4096, NULL, 6, NULL);
    xTaskCreate(morse_tx_task, "morse_tx_task", 4096, NULL, 5, NULL);
}
