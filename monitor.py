import time
import paho.mqtt.client as mqtt
import json
from collections import deque

# -------------------------------------------------------------
# MQTT CONFIG
# -------------------------------------------------------------
BROKER = "mqtt-dev.precise.seas.upenn.edu"
PORT = 1883
USERNAME = "cis441-541_2025"
PASSWORD = "cukwy2-geNwit-puqced"

# TEAM heart_racer topic layout
TOPIC_HEART = "cis441-541/heart_racer/pacemaker/heartrate"
TOPIC_ALARM = "cis441-541/heart_racer/pacemaker/alarm"

# -------------------------------------------------------------
# JSON formatting (your structures)
# -------------------------------------------------------------
def json_summary(window, avg_bpm, total, paced, pace_pct):
    return json.dumps({
        "window": window,
        "avg_bpm": float(f"{avg_bpm:.2f}"),
        "total_beats": total,
        "paced_beats": paced,
        "pace_pct": float(f"{pace_pct:.1f}")
    })

def json_fast_heart(avg_bpm, max_bpm):
    return "{\"type\":\"FAST_HEART\",\"avg_bpm\":%.2f,\"max_bpm\":%.2f}" % (avg_bpm, max_bpm)

def json_slow_heart(pace_pct, threshold):
    return "{\"type\":\"SLOW_HEART\",\"pace_pct\":%.1f,\"threshold\":%u}" % (pace_pct, threshold)

# -------------------------------------------------------------
# Sliding Window Storage
# -------------------------------------------------------------
WINDOW_SECONDS = 20
URL_MS = 180  # From Milestone 1
PACING_THRESHOLD = 60.0  # percent

events = deque(maxlen=5000)

# -------------------------------------------------------------
# Compute stats + emit JSON
# -------------------------------------------------------------
def compute_stats_and_output():
    if not events:
        return

    now = events[-1][1]  # latest timestamp

    # Keep only last WINDOW_SECONDS worth of events
    cutoff = now - WINDOW_SECONDS * 1000
    while events and events[0][1] < cutoff:
        events.popleft()

    total = len(events)
    if total == 0:
        return

    paced = sum(1 for (kind, _) in events if kind == "P")
    pace_pct = (paced / total) * 100.0

    avg_bpm = (total / WINDOW_SECONDS) * 60.0
    url_bpm_limit = 60000.0 / URL_MS

    # ---- SUMMARY JSON ----
    print(json_summary(WINDOW_SECONDS, avg_bpm, total, paced, pace_pct))

    # ---- FAST HEART ALARM ----
    if avg_bpm > url_bpm_limit:
        print(json_fast_heart(avg_bpm, url_bpm_limit))

    # ---- SLOW HEART (PACER DEPENDENT) ----
    if pace_pct > PACING_THRESHOLD:
        print(json_slow_heart(pace_pct, int(PACING_THRESHOLD)))

# -------------------------------------------------------------
# MQTT Callbacks
# -------------------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties):
    print("Connected:", reason_code)
    client.subscribe(TOPIC_HEART)
    client.subscribe(TOPIC_ALARM)

    print("Subscribing to:", TOPIC_HEART, TOPIC_ALARM)


def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"[{msg.topic}] {payload}")

    if msg.topic == TOPIC_HEART:
        try:
            payload_dict = json.loads(payload)
            # events.append((kind, int(ts)))
            # compute_stats_and_output()
            print("average bpm", payload_dict["avg_bpm"])

        except Exception as e:
            print("Malformed heartbeat payload:", e)

    if msg.topic == TOPIC_ALARM:
        print(f"ALARM RECEIVED → {payload}")

# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_forever()

