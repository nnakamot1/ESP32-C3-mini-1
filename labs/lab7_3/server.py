#!/usr/bin/env python3
"""
Lab 7.3 server — combines GET /location and POST reading logging.

Run on your laptop/Pi (must be on the same network as the ESP32):
    pip install flask
    python3 server.py

Edit LOCATION below to whatever city wttr.in should look up.
Then set SERVER_IP in the lab7_3 build_flags (platformio.ini) to this
machine's LAN IP address.

Test the location endpoint manually with:
    wget http://SERVER_IP:1234/location
"""
from datetime import datetime
from flask import Flask, request, jsonify

app = Flask(__name__)

# Location this weather station reports for. Spaces are fine —
# the ESP32 converts them to '+' before querying wttr.in.
LOCATION = "San Diego"


@app.route("/location", methods=["GET"])
def get_location():
    return LOCATION, 200, {"Content-Type": "text/plain"}


@app.route("/", methods=["POST"])
def receive_reading():
    data = request.get_json(silent=True) or {}
    location = data.get("location")
    outdoor_temp = data.get("outdoor_temp")
    sensor_temp_c = data.get("sensor_temp_c")

    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] location={location} outdoor_temp={outdoor_temp} sensor_temp_c={sensor_temp_c}")

    return jsonify({"status": "ok"}), 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=1234)
