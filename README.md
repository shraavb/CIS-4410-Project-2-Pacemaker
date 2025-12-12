# CIS 4410 Project 2: Smart Pacemaker System

A real-time embedded systems project implementing a VVI (Ventricular Inhibited) pacemaker with formal verification using UPPAAL and real-time monitoring via MQTT.

## Submission Checklist:

Your submission should include presentation slides, recording, Arduino code, Codio heart-monitor code, the UPPAAL model, and your final report, all packaged into a single ZIP file and uploaded to Canvas. Please also include a README with instructions on how to run your code.

1 Presentation Slides [check]

2 Recording [check] - in the report

3. Codio code [check]

4. UPPAAL Model [check]

5. Final Report [check]

6. Readme [check]

## Project Overview

This project simulates a cardiac pacemaker system with three main components:
1. **Pacemaker Controller** - Arduino-based VVI mode pacemaker implementation
2. **Heart Simulator** - Simulates natural heart behavior with variable inter-beat intervals
3. **Remote Monitor** - Python-based MQTT monitor for real-time heartbeat analysis

The system demonstrates real-time embedded programming with FreeRTOS, formal verification with UPPAAL timed automata, and IoT communication using MQTT.

## System Architecture

![System Architecture](CIS%204410%20Proj%202_System%20Architecture.jpg)

## Project Structure

```
.
├── controller/              # Pacemaker controller code
│   ├── src/
│   │   ├── main.cpp        # VVI pacemaker implementation with FreeRTOS
│   │   └── arduino_secrets.h  # WiFi and MQTT credentials (not tracked)
│   ├── platformio.ini      # PlatformIO configuration
│   └── .vscode/            # VSCode settings
│
├── heart/                  # Heart simulator code
│   ├── src/
│   │   └── main.cpp        # Heart behavior simulation with FreeRTOS
│   ├── platformio.ini      # PlatformIO configuration
│   └── .vscode/            # VSCode settings
│
├── monitor.py              # Python MQTT monitor for heart rate analysis
├── Proj_2_UPPAAL.xml       # UPPAAL formal verification model
├── CIS 4410 - Proj 2 Report.pdf       # Project documentation
├── CIS 4410 Presentation.pdf          # Project presentation
└── README.md               # This file
```

## Hardware Requirements

- 2x Arduino Uno WiFi Rev2 (or compatible with WiFiNINA library)
- USB cables for programming and power
- Jumper wires for serial communication (TX/RX)
- WiFi network access

## Software Requirements

- [PlatformIO](https://platformio.org/) (recommended) or Arduino IDE
- Python 3.x with `paho-mqtt` library
- [UPPAAL](https://uppaal.org/) (for viewing/modifying formal models)
- MQTT broker access (configured for `mqtt-dev.precise.seas.upenn.edu`)

## Installation & Setup

### 1. Arduino Setup

Both the pacemaker controller and heart simulator use PlatformIO for building and uploading.

#### Pacemaker Controller

```bash
cd controller
```

Create `src/arduino_secrets.h` with your WiFi and MQTT credentials:

```cpp
#define SECRET_SSID "your-wifi-ssid"
#define SECRET_PASS "your-wifi-password"
#define SECRET_MQTT_USERNAME "your-mqtt-username"
#define SECRET_MQTT_PASSWORD "your-mqtt-password"
```

Build and upload to the first Arduino:

```bash
pio run --target upload
```

Monitor serial output:

```bash
pio device monitor
```

#### Heart Simulator

```bash
cd heart
pio run --target upload
```

Monitor serial output:

```bash
pio device monitor
```

### 2. Hardware Connections

Connect the two Arduino boards via serial (UART):

```
Pacemaker TX (Pin 1) ──► Heart RX (Pin 0)
Pacemaker RX (Pin 0) ◄── Heart TX (Pin 1)
Pacemaker GND ────────── Heart GND
```

### 3. Python Monitor Setup

Install required Python packages:

```bash
pip install paho-mqtt
```

Run the monitor:

```bash
python monitor.py
```

The monitor will:
- Subscribe to heartrate and alarm topics
- Display real-time heartbeat statistics
- Show alarms for fast/slow heart conditions

## System Parameters

### Pacemaker (VVI Mode)

| Parameter | Value | Description |
|-----------|-------|-------------|
| LRL | 1500 ms | Lower Rate Limit (60 bpm) |
| URL | 333 ms | Upper Rate Limit (180 bpm) |
| VRP | 150 ms | Ventricular Refractory Period |
| HRI | 1700 ms | Hysteresis Rate Interval |

### Heart Simulator

| Parameter | Value | Description |
|-----------|-------|-------------|
| minIBI | 400 ms | Minimum Inter-Beat Interval (150 bpm) |
| maxIBI | 2000 ms | Maximum Inter-Beat Interval (30 bpm) |

### Monitoring & Alarms

| Parameter | Value | Description |
|-----------|-------|-------------|
| Window Size | 20 seconds | Sliding window for statistics |
| Publish Interval | 5 seconds | MQTT update frequency |
| Pace Threshold | 70% | Alarm if >70% beats are paced |

## Operation

### Normal Operation Flow

1. **Startup**: Both Arduinos initialize and establish serial communication
2. **Heart Behavior**: Heart beats intrinsically at random intervals (400-2000ms)
3. **Pacemaker Sensing**: Pacemaker detects natural beats via serial ('S' signal)
4. **Pacemaker Pacing**: If no beat detected within LRL (1500ms), pacemaker sends pace signal ('P')
5. **Monitoring**: Statistics published to MQTT every 5 seconds
6. **Alarms**: System triggers alarms for abnormal conditions

### MQTT Topics

| Topic | Description | Format |
|-------|-------------|--------|
| `cis441-541/heart_racer/pacemaker/heartrate` | Heart rate statistics | JSON with avg_bpm, total_beats, paced_beats |
| `cis441-541/heart_racer/pacemaker/alarm` | Abnormal heart conditions | JSON with alarm type and details |
| `cis441-541/heart_racer/pacemaker/status` | Normal operation status | JSON with status |

### Example JSON Messages

**Heartrate:**
```json
{
  "window": 20,
  "avg_bpm": 65.4,
  "total_beats": 22,
  "paced_beats": 5,
  "pace_pct": 22.7
}
```

**Fast Heart Alarm:**
```json
{
  "type": "FAST_HEART",
  "avg_bpm": 185.2
}
```

**Slow Heart Alarm:**
```json
{
  "type": "SLOW_HEART",
  "pace_pct": 75.0,
  "threshold": 70
}
```

## Formal Verification (UPPAAL)

The `Proj_2_UPPAAL.xml` file contains timed automata models for:

1. **Pacemaker Template**: VVI mode state machine with timing constraints
2. **Heart Template**: Natural heart behavior with refractory periods
3. **System Properties**: Verified safety and liveness properties

### Key Verified Properties

- Pacing only occurs when necessary (after LRL timeout)
- No pacing during VRP (refractory period)
- System responds to all sensed beats appropriately
- Timing constraints are satisfied

To view/verify:
1. Open UPPAAL
2. Load `Proj_2_UPPAAL.xml`
3. Navigate to Verifier tab
4. Run property checks

## Troubleshooting

### Serial Communication Issues
- Verify TX/RX connections are correct (crossover: TX→RX, RX→TX)
- Check that both Arduinos use the same baud rate (9600)
- Ensure common ground connection

### WiFi Connection Issues
- Verify WiFi credentials in `arduino_secrets.h`
- Check WiFi network availability
- Monitor serial output for connection status

### MQTT Issues
- Verify broker address and credentials
- Check network connectivity
- Ensure MQTT topics match in both code and monitor

### Build Issues
- Ensure PlatformIO is properly installed
- Check that all dependencies are downloaded
- Try cleaning build: `pio run --target clean`

## Development Team

Team: **heart_racer**

## License

Academic project for CIS 4410 - Real-Time Embedded Systems

## References

- [VVI Pacemaker Specification](https://www.bostonscientific.com/)
- [UPPAAL Tutorial](https://uppaal.org/documentation/)
- [FreeRTOS Documentation](https://www.freertos.org/)
- [MQTT Protocol](https://mqtt.org/)
