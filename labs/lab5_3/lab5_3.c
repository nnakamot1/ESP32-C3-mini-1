#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_adc/adc_oneshot.h"

static const char *TAG = "MORSE_RX";

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

// Keep timing as-is
#define MORSE_UNIT_MS        18    // length of a "dot"
#define SAMPLE_PERIOD_MS     9    // sampling period
#define CALIBRATION_SAMPLES  200

// Classification thresholds (in units of MORSE_UNIT_MS)
#define DOT_MAX_UNITS        2     // <= 2 units -> dot
#define DASH_MIN_UNITS       3     // >= 3 units -> dash
#define MAX_DASH_UNITS       5     // > 5 units ON is considered noise / invalid

#define LETTER_GAP_UNITS     3     // >= 3 units of OFF -> end of letter
#define WORD_GAP_UNITS       7     // >= 7 units of OFF -> space between words
// Very long OFF gap = end-of-phrase
#define MESSAGE_GAP_UNITS    9    // >= 20 units -> end of phrase (newline)

// ADC thresholding
#define THRESHOLD_MARGIN     50    // counts above dark baseline

// Ignore very short pulses/gaps (units < MIN_VALID_UNITS)
#define MIN_VALID_UNITS      1

// Cap pattern length to avoid runaway patterns
#define MAX_SYMBOLS_PER_CHAR 5

// Require light level to be stable for a few samples before toggling
#define LIGHT_STABLE_SAMPLES 2

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

/* -------------------------------------------------------------------------- */
/*                               ADC handling                                 */
/* -------------------------------------------------------------------------- */

static adc_oneshot_unit_handle_t adc_handle;
static int dark_baseline = 0;
static int light_threshold = 0;

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = MORSE_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = MORSE_ADC_ATTEN,
        .bitwidth = MORSE_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MORSE_ADC_CHANNEL, &chan_cfg));
}

static void adc_calibrate_dark(void)
{
    ESP_LOGI(TAG, "Calibrating dark baseline (%d samples). Keep LED OFF.", CALIBRATION_SAMPLES);
    int sum = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, MORSE_ADC_CHANNEL, &raw) == ESP_OK) {
            sum += raw;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    dark_baseline   = sum / CALIBRATION_SAMPLES;
    light_threshold = dark_baseline + THRESHOLD_MARGIN;

    ESP_LOGI(TAG, "Dark baseline=%d, threshold=%d", dark_baseline, light_threshold);
}

/*
 * More robust light-state detection:
 * - Uses a boolean "logical" light state.
 * - Requires LIGHT_STABLE_SAMPLES consecutive samples above/below threshold
 *   before toggling the logical state, to avoid flicker on noise.
 */
static bool read_light_state(void)
{
    static bool logical_light   = false;
    static int  above_count     = 0;
    static int  below_count     = 0;

    int raw = 0;
    if (adc_oneshot_read(adc_handle, MORSE_ADC_CHANNEL, &raw) != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed");
        return logical_light;
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

/* -------------------------------------------------------------------------- */
/*                     Helpers for decoding based on time                     */
/* -------------------------------------------------------------------------- */

static void flush_letter(char *symbol_buf, int *len)
{
    if (*len == 0) {
        return;
    }

    symbol_buf[*len] = '\0';

    // Count dots and dashes for simple noise heuristics
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

    // Heuristic: 5-symbol patterns with 0 or 1 dash (".....", "....-")
    // are often noise in this setup. We still allow patterns like
    // "...--" (3) and "..---" (2), which have >= 2 dashes.
    if (*len == 5 && dash_count <= 1) {
        c = 0; // treat as unknown / noise
    }

    // Only print recognized characters; don't print '?' anymore
    if (c != 0) {
        putchar(c);
        fflush(stdout);
    }

    ESP_LOGI(TAG, "LETTER: \"%s\" -> '%c'", symbol_buf, c ? c : '?');

    *len = 0;
}

/*
 * ON-time checker:
 * - Ignores very short pulses (< MIN_VALID_UNITS).
 * - Distinguishes dot vs dash based on units.
 * - Rejects very long ON pulses (> MAX_DASH_UNITS) as noise and flushes
 *   the current symbol buffer (helps recover from messed-up dashes).
 * - Caps pattern length (MAX_SYMBOLS_PER_CHAR) to keep letters sane.
 */
static void handle_on_time(int on_time_ms, char *symbol_buf, int *len)
{
    if (on_time_ms <= 0) {
        return;
    }

    int units = (on_time_ms + MORSE_UNIT_MS / 2) / MORSE_UNIT_MS; // round

    // Ignore tiny pulses that round to < 1 unit
    if (units < MIN_VALID_UNITS) {
        ESP_LOGD(TAG, "Ignoring short ON pulse: %d ms (~%d units)", on_time_ms, units);
        return;
    }

    // If this pulse is way too long for a valid dash, treat as noise,
    // flush any partial letter, and don't add a symbol from this pulse.
    if (units > MAX_DASH_UNITS) {
        ESP_LOGW(TAG,
                 "Very long ON pulse: %d ms (~%d units) - treating as noise, flushing \"%s\"",
                 on_time_ms, units, symbol_buf);
        flush_letter(symbol_buf, len);
        return;
    }

    char sym = (units <= DOT_MAX_UNITS) ? '.' : '-';

    // Cap pattern length: if full, flush as a letter before adding more
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

    // Debug-only to avoid clutter
    ESP_LOGD(TAG, "PULSE: %d ms (~%d units) -> '%c', pattern=\"%s\"",
             on_time_ms, units, sym, symbol_buf);
}

/*
 * OFF-time checker:
 * - Called with the *total* OFF time between pulses
 *   (usually on OFF->ON transition, or for very long OFF as phrase gap).
 * - Ignores very short gaps (< MIN_VALID_UNITS).
 * - Distinguishes symbol gap vs letter gap vs word gap vs phrase gap.
 * - Prints:
 *      - nothing for symbol gap,
 *      - space ' ' for word gap,
 *      - newline '\n' for phrase gap.
 * - Logs are worded as *confirmations* that the time has elongated enough.
 */
static void handle_off_time(int off_time_ms, char *symbol_buf, int *len)
{
    if (off_time_ms <= 0) {
        return;
    }

    int units = (off_time_ms + MORSE_UNIT_MS / 2) / MORSE_UNIT_MS;

    // Ignore tiny gaps that round to < 1 unit
    if (units < MIN_VALID_UNITS) {
        ESP_LOGD(TAG, "Ignoring short OFF gap: %d ms (~%d units)", off_time_ms, units);
        return;
    }

    if (units >= MESSAGE_GAP_UNITS) {
        // End of phrase: flush last letter (if any), then newline.
        if (*len > 0) {
            flush_letter(symbol_buf, len);
        }
        putchar('\n');
        fflush(stdout);
        ESP_LOGI(TAG,
                 "PHRASE GAP CONFIRMED: %d ms (~%d units) -> end of phrase",
                 off_time_ms, units);
    } else if (units >= WORD_GAP_UNITS) {
        // Word gap: flush letter and add a visible space between words
        if (*len > 0) {
            flush_letter(symbol_buf, len);
        }
        putchar(' ');
        fflush(stdout);
        ESP_LOGI(TAG,
                 "WORD GAP CONFIRMED: %d ms (~%d units) -> new word",
                 off_time_ms, units);
    } else if (units >= LETTER_GAP_UNITS) {
        // Letter gap (between letters in the same word)
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
        // Gap between symbols inside a letter
        if (*len > 0) {
            ESP_LOGD(TAG, "SYMBOL GAP: %d ms (~%d units)", off_time_ms, units);
        } else {
            ESP_LOGD(TAG, "IDLE GAP: %d ms (~%d units) with empty buffer", off_time_ms, units);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                          Main decoder: time-based                          */
/* -------------------------------------------------------------------------- */

static void morse_decoder_task(void *arg)
{
    (void)arg;

    adc_init();
    adc_calibrate_dark();

    bool prev_light        = false;
    int  on_time_ms        = 0;
    int  off_time_ms       = 0;
    bool phrase_gap_emitted = false;  // ensures phrase gap is handled once per long OFF

    char symbol_buf[16]    = {0};
    int  symbol_len        = 0;

    ESP_LOGI(TAG, "Starting Morse decoder (time-based).");

    while (1) {
        bool light = read_light_state();

        if (light) {
            // Light is ON: count ON time
            on_time_ms += SAMPLE_PERIOD_MS;

            // OFF -> ON: we have just ended an OFF gap; classify it
            if (!prev_light && off_time_ms > 0) {
                handle_off_time(off_time_ms, symbol_buf, &symbol_len);
                off_time_ms       = 0;
                phrase_gap_emitted = false;  // new activity, reset phrase detector
            }
        } else {
            // Light is OFF: count OFF time
            off_time_ms += SAMPLE_PERIOD_MS;

            // ON -> OFF: we have just ended an ON pulse; classify dot/dash
            if (prev_light && on_time_ms > 0) {
                handle_on_time(on_time_ms, symbol_buf, &symbol_len);
                on_time_ms = 0;
            }

            // While staying OFF, detect a very long OFF gap as phrase end,
            // even if the LED never turns back on.
            int units = (off_time_ms + MORSE_UNIT_MS / 2) / MORSE_UNIT_MS;
            if (!phrase_gap_emitted && units >= MESSAGE_GAP_UNITS) {
                handle_off_time(off_time_ms, symbol_buf, &symbol_len);
                phrase_gap_emitted = true;
                off_time_ms        = 0;  // reset after phrase separation
            }
        }

        prev_light = light;

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* -------------------------------------------------------------------------- */
/*                                  app_main                                  */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    xTaskCreate(morse_decoder_task, "morse_decoder_task", 4096, NULL, 5, NULL);
}
