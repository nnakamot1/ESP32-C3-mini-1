#!/usr/bin/env python3
"""
Lab 7.2 server — receives POSTed temperature readings from the ESP32.

Run on your laptop/Pi (must be on the same network as the ESP32):
    pip install flask
    python3 server.py

Then set SERVER_IP in labs/lab7_2/../../platformio.ini (or the
lab7_2 build_flags) to this machine's LAN IP address.
"""
from datetime import datetime
from flask import Flask, request, jsonify

app = Flask(__name__)


@app.route("/", methods=["POST"])
def receive_reading():
    data = request.get_json(silent=True) or {}
    temp_c = data.get("sensor_temp_c")
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] Received sensor_temp_c = {temp_c}")
    return jsonify({"status": "ok"}), 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=1234)
