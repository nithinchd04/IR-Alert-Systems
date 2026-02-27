# IR Alert System 🔴

<div align="center">

![IR Alert System Banner](https://img.shields.io/badge/IR%20Alert%20System-Proximity%20Alarm-0f766e?style=for-the-badge&logo=arduino&logoColor=white)

**An Intelligent Proximity Security Alarm built with Arduino Uno**

[![Arduino](https://img.shields.io/badge/Platform-Arduino%20Uno-00979D?style=flat-square&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Language](https://img.shields.io/badge/Language-Arduino%20C%2B%2B-blue?style=flat-square&logo=cplusplus&logoColor=white)](https://www.arduino.cc/reference/en/)
[![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=flat-square)]()
[![PRs Welcome](https://img.shields.io/badge/PRs-Welcome-blue?style=flat-square)](CONTRIBUTING.md)

[Overview](#-overview) · [Features](#-features) · [Components](#-components) · [Wiring](#-wiring) · [Setup](#-getting-started) · [Code](#-code) · [Troubleshooting](#-troubleshooting)

</div>

---

## 📖 Overview

The **IR Alert System** is a real-time proximity detection alarm built on an Arduino Uno. It uses an **FC-51 infrared sensor** to continuously monitor a defined zone. The moment an object or person enters the detection range, the system responds instantly with:

- 🔔 A **buzzer alarm** (1 kHz tone via PWM)
- 💡 A **red LED indicator**
- 🖥️ A live **LCD status message** via I²C

When the zone clears, everything resets automatically — no manual intervention needed.

This project is ideal as a **starter security system**, a **learning platform** for embedded systems concepts, or a building block for more advanced IoT automation.

```
┌─────────────────────────────────────────────────────────┐
│                  SYSTEM FLOW                            │
│                                                         │
│   IR Sensor ──► Arduino Uno ──► Buzzer  (Pin 8)        │
│   (Pin 2)             │──────► LED     (Pin 13)        │
│                       └──────► LCD I²C (A4 / A5)       │
└─────────────────────────────────────────────────────────┘
```

---

## ✨ Features

- 📡 Infrared proximity detection with **adjustable range (2–30 cm)**
- 🔔 Audio alert via **passive buzzer** (PWM-driven, 1 kHz)
- 💡 Visual alert via **5mm red LED**
- 🖥️ **I²C LCD 16×2** displaying live status (`Standby...` / `ALERT! Object Detected`)
- 🔄 **Auto-reset** when the detection zone is cleared
- 📟 **Serial monitor output** at 9600 baud for PC-side debugging
- ⚡ Fully powered over **USB** — no external supply required
- 🧩 **Breadboard-friendly** — no soldering needed

---

## 🧰 Components

| # | Component | Model / Spec | Qty |
|---|-----------|-------------|-----|
| 1 | Arduino Uno | ATmega328P, 16 MHz, 14 digital I/O | 1 |
| 2 | IR Sensor Module | FC-51 / TCRT5000, adjustable range | 1 |
| 3 | Passive Buzzer | 5V, PWM-driven | 1 |
| 4 | 5mm LED | Red, Vf ≈ 2V | 1 |
| 5 | Resistor | 220Ω current-limiting | 1 |
| 6 | I²C LCD Display | 16×2, I²C backpack, address `0x27` | 1 |
| 7 | Half-size Breadboard | 400 tie-points | 1 |
| 8 | Jumper Wires | Male-to-Male | ~10 |

> 💰 **Estimated total cost:** Under $10 USD using common hobbyist suppliers.

---

## 🔌 Wiring

### IR Sensor Module (FC-51 / TCRT5000)

| IR Sensor Pin | → | Arduino Pin | Notes |
|:---:|:---:|:---:|---|
| VCC | → | 5V | Power |
| GND | → | GND | Ground |
| OUT | → | **Digital Pin 2** | Digital input signal |

### Passive Buzzer

| Buzzer Pin | → | Arduino Pin | Notes |
|:---:|:---:|:---:|---|
| + (positive) | → | **Digital Pin 8** | PWM tone output |
| − (negative) | → | GND | Ground |

### 5mm LED + 220Ω Resistor

| LED Terminal | → | Arduino Connection | Notes |
|:---:|:---:|:---:|---|
| Anode (+) | → | **Digital Pin 13** → 220Ω | Resistor in series |
| Cathode (−) | → | GND | Ground |

### I²C LCD 16×2 (address `0x27`)

| LCD Pin | → | Arduino Pin | Notes |
|:---:|:---:|:---:|---|
| VCC | → | 5V | Power |
| GND | → | GND | Ground |
| SDA | → | **Analog Pin A4** | I²C data line |
| SCL | → | **Analog Pin A5** | I²C clock line |

> [!TIP]
> Use the breadboard's **+** and **−** power rails to distribute 5V and GND across all components. Keep signal wires short to reduce noise on the IR sensor output.

> [!NOTE]
> Some I²C LCD modules use address `0x3F` instead of `0x27`. If your display shows nothing, try changing `LiquidCrystal_I2C lcd(0x27, 16, 2)` to `LiquidCrystal_I2C lcd(0x3F, 16, 2)` in the sketch, or run the [I²C Scanner sketch](https://playground.arduino.cc/Main/I2cScanner/) to detect the correct address.

---

## 📦 Required Libraries

Install via **Arduino IDE Library Manager** (`Sketch → Include Library → Manage Libraries`):

| Library | Author | Purpose |
|---------|--------|---------|
| `Wire` | Arduino (built-in) | I²C communication |
| `LiquidCrystal I2C` | Frank de Brabander | I²C LCD control |

---

## 🚀 Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (v1.8+ or v2.x)
- USB Type-B cable (Arduino Uno standard)
- All components listed above

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/ir-alert-system.git
cd ir-alert-system
```

### 2. Install the Library

Open Arduino IDE and navigate to:

```
Sketch → Include Library → Manage Libraries
```

Search for **`LiquidCrystal I2C`** and install the version by **Frank de Brabander**.

### 3. Wire the Hardware

Follow the [Wiring](#-wiring) section above. Double-check:
- IR sensor `OUT` → Pin **2**
- Buzzer `+` → Pin **8**
- LED anode → Pin **13** (with 220Ω resistor in series)
- LCD `SDA` → **A4**, `SCL` → **A5**

### 4. Open the Sketch

```
File → Open → ir_alert_system/ir_alert_system.ino
```

### 5. Upload

1. Connect Arduino Uno via USB
2. Select board: `Tools → Board → Arduino Uno`
3. Select port: `Tools → Port → COMx` (Windows) or `/dev/ttyUSBx` (Linux/macOS)
4. Click **Upload** ➜

### 6. Test

Open the Serial Monitor (`Tools → Serial Monitor`) at **9600 baud**.

Wave your hand in front of the IR sensor. You should see:

| Event | LCD | Buzzer | LED | Serial |
|-------|-----|--------|-----|--------|
| Object detected | `>> ALERT! <<` | Sounds (1 kHz) | ON 🔴 | `ALERT: Object in range` |
| Zone cleared | `Standby...` | Silent | OFF | `Zone cleared.` |

---

## 💻 Code

```cpp
// IR Alert System — Intelligent Proximity Alarm
// Arduino Uno Sketch

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin Definitions
const int IR_PIN    = 2;
const int BUZZER    = 8;
const int LED_PIN   = 13;

// LCD: I2C address 0x27, 16 columns, 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool lastState = HIGH;

void setup() {
  pinMode(IR_PIN,  INPUT);
  pinMode(BUZZER,  OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  IR Alert Sys  ");
  lcd.setCursor(0, 1);
  lcd.print("   Standby...   ");

  Serial.begin(9600);
  delay(1500);
}

void loop() {
  bool detected = (digitalRead(IR_PIN) == LOW);

  if (detected && lastState == HIGH) {
    // Object detected — trigger alert
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER, 1000, 300);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" >> ALERT! <<   ");
    lcd.setCursor(0, 1);
    lcd.print("Object Detected ");

    Serial.println("ALERT: Object in range");
  }

  if (!detected && lastState == LOW) {
    // Zone cleared — reset to standby
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  IR Alert Sys  ");
    lcd.setCursor(0, 1);
    lcd.print("   Standby...   ");

    Serial.println("Zone cleared.");
  }

  lastState = detected ? LOW : HIGH;
  delay(50);
}
```

### Logic Flow

```
SETUP
  └── Initialize pins → Init LCD → Show "Standby..." → Begin Serial

LOOP (every 50ms)
  ├── Read IR Pin 2
  │
  ├── [LOW = Object Detected]
  │     ├── LED ON
  │     ├── tone(BUZZER, 1000, 300)
  │     ├── LCD: ">> ALERT! << / Object Detected"
  │     └── Serial: "ALERT: Object in range"
  │
  └── [HIGH = Zone Clear]
        ├── LED OFF
        ├── noTone(BUZZER)
        ├── LCD: "IR Alert Sys / Standby..."
        └── Serial: "Zone cleared."
```

---

## 🔧 Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| LCD shows nothing | Wrong I²C address | Run [I²C Scanner](https://playground.arduino.cc/Main/I2cScanner/); try `0x3F` |
| LCD shows black blocks only | Contrast too low | Turn the blue potentiometer on the I²C backpack |
| IR always triggered | Ambient IR interference | Shield sensor from sunlight or fluorescent light |
| IR never triggers | Detection range too short | Turn sensor potentiometer clockwise to increase range |
| No buzzer sound | Wiring issue | Confirm buzzer `+` is on Pin 8, not directly on 5V |
| Upload fails | Wrong port or board | Check `Tools → Board` and `Tools → Port` |
| Garbled serial output | Wrong baud rate | Set Serial Monitor to **9600** baud |

---

## 🌐 Possible Extensions

| Idea | How |
|------|-----|
| **Relay switch** | Trigger a light, lock, or fan on detection |
| **Wi-Fi alerts** | Swap to ESP8266/ESP32 and send MQTT / HTTP push notifications |
| **Event counter** | Log detection count to EEPROM or SD card |
| **Multiple zones** | Add more IR sensors on Pins 3, 4, 5… |
| **Longer range** | Replace IR with HC-SR04 ultrasonic sensor (up to 4 m) |
| **Arm / Disarm** | Add a 4×4 keypad and password logic |
| **Timestamp log** | Add DS3231 RTC module to log events with date and time |

---

## 📁 Project Structure

```
ir-alert-system/
│
├── ir_alert_system/
│   └── ir_alert_system.ino     ← Main Arduino sketch
│
├── docs/
│   └── wiring_diagram.png      ← (optional) schematic image
│
├── ir-alert-system.html        ← Standalone landing page
├── README.md                   ← This file
└── LICENSE                     ← MIT License
```

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome!

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/add-wifi-alerts`
3. Commit your changes: `git commit -m 'Add Wi-Fi alert support'`
4. Push to the branch: `git push origin feature/add-wifi-alerts`
5. Open a Pull Request

---

## 📜 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

Made with ❤️ in the lab.

> *IR Alert System — Intelligent Proximity Security Alarm*  
> Platform: Arduino Uno · Language: Arduino C++ · Protocol: I²C
