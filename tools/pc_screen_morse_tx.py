#!/usr/bin/env python3
"""
Flashes a fullscreen white/black window in Morse code timing, so you can
hold the ESP32's photoresistor up to your PC screen to test the
lab5_2/lab5_3 optical Morse receiver firmware. No extra hardware needed.

Usage:
    python3 pc_screen_morse_tx.py "SOS" --unit-ms 20 --repeat 3

Match --unit-ms to the receiver you're testing:
    lab5_2 expects 20ms (the default), lab5_3 expects 18ms.

Hold the photoresistor directly against the screen (or as close as
possible) in a dim room for the best signal contrast. Press Esc at any
time to quit early.
"""
import argparse
import time
import tkinter as tk

MORSE = {
    'A': ".-",  'B': "-...", 'C': "-.-.", 'D': "-..",  'E': ".",
    'F': "..-.",'G': "--.",  'H': "....", 'I': "..",   'J': ".---",
    'K': "-.-", 'L': ".-..", 'M': "--",   'N': "-.",   'O': "---",
    'P': ".--.",'Q': "--.-", 'R': ".-.",  'S': "...",  'T': "-",
    'U': "..-", 'V': "...-", 'W': ".--",  'X': "-..-", 'Y': "-.--",
    'Z': "--..",
    '0': "-----", '1': ".----", '2': "..---", '3': "...--", '4': "....-",
    '5': ".....", '6': "-....", '7': "--...", '8': "---..", '9': "----.",
}


def parse():
    p = argparse.ArgumentParser(description="Blink Morse code on the PC screen.")
    p.add_argument("message", type=str, help="Message to send (letters/digits, quote it)")
    p.add_argument("--repeat", type=int, default=1, help="How many times to send it (default: 1)")
    p.add_argument("--unit-ms", type=int, default=20,
                   help="Morse unit in ms (default: 20, matching lab5_2)")
    p.add_argument("--verbose", action="store_true", help="Print per-symbol timing")
    return p.parse_args()


class ScreenBlinker:
    def __init__(self):
        self.root = tk.Tk()
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="black")
        self.root.bind("<Escape>", lambda e: self.root.destroy())
        self.label = tk.Label(
            self.root,
            text="Hold the photoresistor to the screen.\nPress Esc to quit.",
            fg="white", bg="black", font=("Arial", 28),
        )
        self.label.pack(expand=True)
        self.root.update()

    def alive(self):
        try:
            return bool(self.root.winfo_exists())
        except tk.TclError:
            return False

    def set_light(self, on):
        if not self.alive():
            return
        color = "white" if on else "black"
        self.root.configure(bg=color)
        self.label.configure(bg=color)
        self.root.update()

    def sleep(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            if not self.alive():
                return
            self.root.update()
            time.sleep(0.005)

    def close(self):
        if self.alive():
            self.root.destroy()


def blink_symbol(screen, sym, u, verbose):
    if sym == '.':
        screen.set_light(True); screen.sleep(u); screen.set_light(False)
        if verbose: print("[.] on={}s off={}s".format(u, u))
    elif sym == '-':
        screen.set_light(True); screen.sleep(3 * u); screen.set_light(False)
        if verbose: print("[-] on={}s off={}s".format(3 * u, u))


def split_words_letters(s: str):
    s = s.replace('/', ' ')
    return [list(word) for word in s.split()]


def blink_morse_once(screen, message, unit_s, verbose):
    words = split_words_letters(message)
    for wi, word in enumerate(words):
        for li, letter in enumerate(word):
            code = MORSE.get(letter.upper())
            if not code:
                continue
            for si, sym in enumerate(code):
                if not screen.alive():
                    return
                blink_symbol(screen, sym, unit_s, verbose)
                if si < len(code) - 1:
                    screen.sleep(unit_s)          # 1u between symbols
            if li < len(word) - 1:
                screen.sleep(2 * unit_s)          # +2u (already waited 1u) => 3u letter gap
        if wi < len(words) - 1:
            screen.sleep(6 * unit_s)              # +6u (already waited 1u) => 7u word gap


def main():
    args = parse()
    unit_s = args.unit_ms / 1000.0

    screen = ScreenBlinker()
    screen.sleep(2.0)  # time to position the photoresistor against the screen

    for i in range(args.repeat):
        if not screen.alive():
            break
        blink_morse_once(screen, args.message, unit_s, args.verbose)
        if i < args.repeat - 1:
            screen.sleep(7 * unit_s)  # gap between repeats

    screen.set_light(False)
    screen.sleep(1.0)
    screen.close()


if __name__ == "__main__":
    main()
