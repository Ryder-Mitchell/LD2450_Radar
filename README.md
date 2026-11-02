# ESP32-S3 + LD2450 Radar Tracker

Track up to 3 people in real time using an LD2450 mmWave radar and an ESP32-S3-Zero, displayed as dots on a top-down radar map.

| Build | Hardware Needed | Visualization Output |
|---|---|---|
| `serial` | Radar + ESP32 | Computer screen (`viewer.py`) |
| `oled` | Radar + ESP32 + 1.3" I2C OLED | OLED display (`viewer.py` also works) |

---

## 🛠️ Wiring

### Radar (Both Builds)
| LD2450 | ESP32-S3-Zero |
|---|---|
| 5V | 5V |
| GND | GND |
| TX | GPIO4 |
| RX | GPIO5 |

### OLED (OLED Build Only)
| OLED | ESP32-S3-Zero |
|---|---|
| VCC | **3V3** (⚠️ Do **NOT** use 5V) |
| GND | GND |
| SDA | GPIO1 |
| SCL | GPIO3 |

> **Note:** Use the physical pin pads labeled `4`, `5`, `1`, and `3` (not standard RX/TX labels). Always connect grounds to a real `GND` pin, never a GPIO.

---

## ⚡ Flashing the Firmware

1. Install [PlatformIO](https://platformio.org/install).
2. Open this folder in your terminal and run the command for your setup:

    # Radar + OLED build
    pio run -e oled -t upload

    # Radar-only build (USB serial stream)
    pio run -e serial -t upload

*If the board is not detected during upload, hold the **BOOT** button while plugging in USB, then release and try uploading again.*

---

## 🖥️ Python Visualizer

1. Install Python requirements:
    pip install -r requirements.txt

2. Run the map viewer:
    # Test without hardware (simulated targets)
    python viewer.py --demo

    # Live tracking (auto-detects port)
    python viewer.py

    # Specify port manually (if needed)
    python viewer.py --port COM5         # Windows
    python viewer.py --port /dev/ttyACM0 # Linux

*(Close any open serial monitors before running `viewer.py`, as only one program can access the USB port at a time).*

---

## 📄 License
[MIT License](LICENSE)
