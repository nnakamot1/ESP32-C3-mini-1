#!/usr/bin/env python3
"""
Morse code LED transmitter for Raspberry Pi (gpiozero).

Blinks an LED wired to a GPIO pin, encoding a message in Morse code.
Point the ESP32's photoresistor at this LED to test the lab5_2/lab5_3
optical Morse receiver firmware.

Usage:
    python3 rpi_morse_tx.py <repeat> "<message>" [--pin N] [--unit-ms N] [--active-low] [--verbose]

Requires: a Raspberry Pi with gpiozero installed (not runnable on a plain PC).
"""
import argparse
import time
import sys
from gpiozero import LED

MORSE = {
    'A': ".-",  'B': "-...", 'C': "-.-.", 'D': "-..",  'E': ".",
    'F': "..-.",'G': "--.",  'H': "....", 'I': "..",   'J': ".---",
    'K': "-.-", 'L': ".-..", 'M': "--",   'N': "-.",   'O': "---",
    'P': ".--.",'Q': "--.-", 'R': ".-.",  'S': "...",  'T': "-",
    'U': "..-", 'V': "...-", 'W': ".--",  'X': "-..-", 'Y': "-.--",
    'Z': "--..",
    '0': "-----", '1': ".----", '2': "..---", '3': "...--", '4': "....-",
    '5': ".....", '6': "-....", '7': "--...", '8': "---..", '9': "----.",
    '.': ".-.-.-", ',': "--..--", '?': "..--..", '/': "-..-.", '-': "-....-",
    '@': ".--.-.", '!': "-.-.--", ':': "---...", ';': "-.-.-.", '(': "-.--.",
    ')': "-.--.-", '=': "-...-",  '+': ".-.-.",  '\'': ".----.", '"': ".-..-.",
}

def parse():
    p = argparse.ArgumentParser(description="Blink Morse code on an LED (Raspberry Pi).")
    p.add_argument("repeat", type=int, help="How many times to send the message (>=1)")
    p.add_argument("message", type=str, help="Message to send (quote it)")
    p.add_argument("--pin", type=int, default=27, help="BCM pin for LED (default: 17)")
    p.add_argument("--unit-ms", type=int, default=100, help="Morse unit in ms (default: 100)")
    p.add_argument("--active-low", action="store_true",
                   help="Use if LED is wired to 3V3 and the pin should sink current")
    p.add_argument("--verbose", action="store_true", help="Print per-symbol timing")
    return p.parse_args()

def text_to_morse_printable(s: str) -> str:
    parts, last_slash = [], False
    for ch in s:
        if ch.isspace() or ch == '/':
            if not last_slash:
                parts.append('/')
                last_slash = True
        else:
            code = MORSE.get(ch.upper())
            if code:
                parts.append(code); last_slash = False
    return " ".join(parts)

def split_words_letters(s: str):
    s = s.replace('/', ' ')
    # words = ["hello", "ESP32"]
    # return [["h","e","l","l","o"], ["E","S","P","3","2"]]
    return [list(word) for word in s.split()]

def blink_symbol(led: LED, sym: str, u: float, verbose: bool):
    if sym == '.':
        led.on();  time.sleep(u);     led.off()
        if verbose: print("[.] on={}s off={}s".format(u, u))
    elif sym == '-':
        led.on();  time.sleep(3*u);   led.off()
        if verbose: print("[-] on={}s off={}s".format(3*u, u))

def blink_morse_once(led: LED, message: str, unit_ms: int, verbose: bool):
    u = unit_ms / 1000.0
    words = split_words_letters(message)
    for wi, word in enumerate(words):
        for li, letter in enumerate(word):
            code = MORSE.get(letter.upper())
            if not code: continue
            for si, sym in enumerate(code):
                blink_symbol(led, sym, u, verbose)
                if si < len(code) - 1:
                    time.sleep(u)      # 1u between symbols
            if li < len(word) - 1:
                time.sleep(2*u)       # +2u (already waited 1u) => 3u letter gap
        if wi < len(words) - 1:
            time.sleep(6*u)           # +6u (already waited 1u) => 7u word gap

def main():
    args = parse()
    if args.repeat < 1:
        sys.exit("repeat must be >= 1")

    try:
        led = LED(args.pin, active_high=not args.active_low, initial_value=False)
    except Exception as e:
        print(f"Failed to open BCM pin {args.pin}: {e}", file=sys.stderr)
        sys.exit(1)

    printable = text_to_morse_printable(args.message)
    u = args.unit_ms / 1000.0
    for i in range(args.repeat):
        print(printable)
        blink_morse_once(led, args.message, args.unit_ms, args.verbose)
        if i < args.repeat - 1:
            time.sleep(7 * u)  # gap between repeats
    led.off()

if __name__ == "__main__":
    main()
