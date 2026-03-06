# IR Alert System 🔴


https://wokwi.com/projects/457150179175680001

https://nithinchd04.github.io/IR-Alert-Systems/

[![Arduino](https://img.shields.io/badge/Platform-Arduino%20Uno-00979D?style=flat-square&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Python](https://img.shields.io/badge/Python-3.x-yellow?style=flat-square&logo=python&logoColor=white)](https://www.python.org/)
[![Simulate on Wokwi](https://img.shields.io/badge/Simulate-Wokwi-green?style=flat-square&logo=arduino&logoColor=white)](https://wokwi.com)
[![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)](LICENSE)

---

So I built a motion-triggered security alarm using an Arduino Uno and a few cheap components. The idea is simple — something moves in front of the sensor, and the system immediately sets off a buzzer, lights up an LED, and shows an alert on a small LCD screen. When whatever triggered it moves away, everything resets on its own.

What I really liked about this project is that you don't need to buy anything to try it out. You can run it three different ways depending on what you have available.

---

## How You Can Run It

**Option 1 — You have the actual hardware**
Buy the components (costs under $10), wire everything up, and flash the code onto an Arduino Uno. This is the real deal.

**Option 2 — You don't have hardware but want to try it**
Use [Wokwi](https://wokwi.com) — a free browser-based Arduino simulator. It has all the components you need and the simulation is surprisingly accurate. No install, no signup required.

**Option 3 — You just have a laptop**
Skip Arduino entirely. There's a Python script that uses your webcam to detect motion and mimics the exact same alarm behavior on your screen — LCD display overlay, LED indicator, buzzer sound, everything.

---

## The Hardware Version

### What you need

| Component | Details |
|-----------|---------|
| Arduino Uno | The main brain — ATmega328P chip |
| IR Sensor (FC-51) | Detects objects up to 30cm away |
| Passive Buzzer | Makes the alarm sound |
| Red LED + 220Ω resistor | Visual indicator |
| I²C LCD 16×2 | Shows status messages |
| Breadboard + wires | For connecting everything |

### How to wire it

```
IR Sensor    VCC  →  Arduino 5V
IR Sensor    GND  →  Arduino GND
IR Sensor    OUT  →  Arduino Pin 2

Buzzer        +   →  Arduino Pin 8
Buzzer        −   →  Arduino GND

LED           +   →  220Ω resistor → Arduino Pin 13
LED           −   →  Arduino GND

LCD          VCC  →  Arduino 5V
LCD          GND  →  Arduino GND
LCD          SDA  →  Arduino A4
LCD          SCL  →  Arduino A5
```

### The code

```cpp
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int IR_PIN  = 2;
const int BUZZER  = 8;
const int LED_PIN = 13;

bool lastState  = HIGH;
int  alertCount = 0;

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
}

void loop() {
  bool detected = (digitalRead(IR_PIN) == LOW);

  if (detected && lastState == HIGH) {
    alertCount++;
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER, 1000, 300);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" >> ALERT! <<   ");
    lcd.setCursor(0, 1);
    lcd.print("Object Detected!");
    Serial.print("ALERT #");
    Serial.println(alertCount);
  }

  if (!detected && lastState == LOW) {
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
  delay(100);
}
```

Before uploading, install the **LiquidCrystal I2C** library by Frank de Brabander from the Arduino IDE Library Manager (`Sketch → Include Library → Manage Libraries`).

---

## The Wokwi Simulation

If you don't have hardware, this is the easiest way to see the project working.

Go to [wokwi.com](https://wokwi.com), click **Arduino (Uno, Mega, Nano)**, then **Arduino Uno**. You'll land on an empty project with a blank sketch.

Add these components using the **+** button:

- PIR Motion Sensor *(this replaces the FC-51 IR sensor — same idea, same digital output)*
- LCD 1602 I2C
- Buzzer
- LED
- Resistor *(set value to 220)*

Wire them up like this:

```
PIR   +    →  Arduino 5V
PIR   D    →  Arduino Pin 2
PIR   −    →  Arduino GND

Buzzer +   →  Arduino Pin 8
Buzzer −   →  Arduino GND

LED   +    →  220Ω  →  Arduino Pin 13
LED   −    →  Arduino GND

LCD  VCC   →  Arduino 5V
LCD  GND   →  Arduino GND
LCD  SDA   →  Arduino A4
LCD  SCL   →  Arduino A5
```

Then paste this into the sketch editor:

```cpp
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int PIR_PIN = 2;
const int BUZZER  = 8;
const int LED_PIN = 13;

bool lastState  = LOW;
int  alertCount = 0;

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER,  OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  IR Alert Sys  ");
  lcd.setCursor(0, 1);
  lcd.print("   Standby...   ");

  Serial.begin(9600);
}

void loop() {
  bool detected = (digitalRead(PIR_PIN) == HIGH);

  if (detected && lastState == LOW) {
    alertCount++;
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER, 1000, 300);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" >> ALERT! <<   ");
    lcd.setCursor(0, 1);
    lcd.print("Motion Detected!");
    Serial.print("ALERT #");
    Serial.println(alertCount);
  }

  if (!detected && lastState == HIGH) {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  IR Alert Sys  ");
    lcd.setCursor(0, 1);
    lcd.print("   Standby...   ");
    Serial.println("Zone cleared.");
  }

  lastState = detected ? HIGH : LOW;
  delay(100);
}
```

Hit **▶ Run**. If it asks you to install the LiquidCrystal_I2C library, just click the blue install button that pops up, then run it again.

Once it's running, click directly on the PIR sensor component to simulate motion. The LCD will switch to the alert message, the LED turns on, and the buzzer fires. Click it again to clear the zone.

---

## The Webcam Version

No Arduino, no browser simulator — just Python and your laptop camera.

Install the dependencies:

```bash
pip install opencv-python numpy pygame
```

Save this as `ir_alert_webcam.py` and run it with `python ir_alert_webcam.py`:

```python
import cv2
import numpy as np
import pygame
import time

# Tweak these if needed
CAMERA_INDEX   = 0      # try 1 or 2 if wrong camera opens
SENSITIVITY    = 25     # lower = more sensitive
MIN_AREA       = 1500   # minimum motion size to trigger
ALERT_COOLDOWN = 2.0    # seconds between beeps
BEEP_FREQ      = 1000
BEEP_DURATION  = 300

pygame.mixer.init(frequency=44100, size=-16, channels=1, buffer=512)

def beep():
    sample_rate = 44100
    samples = int(sample_rate * BEEP_DURATION / 1000)
    t = np.linspace(0, BEEP_DURATION / 1000, samples, False)
    wave = (np.sin(2 * np.pi * BEEP_FREQ * t) * 32767).astype(np.int16)
    wave = np.column_stack([wave, wave])
    pygame.sndarray.make_sound(wave).play()

def draw_lcd(frame, line1, line2, alert=False):
    h, w = frame.shape[:2]
    x, y = w - 360, h - 90
    cv2.rectangle(frame, (x, y), (x+340, y+70),
                  (0, 0, 80) if alert else (0, 50, 0), -1)
    cv2.rectangle(frame, (x, y), (x+340, y+70),
                  (0, 0, 255) if alert else (0, 200, 0), 2)
    color = (0, 80, 255) if alert else (0, 255, 0)
    cv2.putText(frame, line1[:16], (x+10, y+25),
                cv2.FONT_HERSHEY_PLAIN, 1.4, color, 1)
    cv2.putText(frame, line2[:16], (x+10, y+55),
                cv2.FONT_HERSHEY_PLAIN, 1.4, color, 1)

def draw_led(frame, on=False):
    h, w = frame.shape[:2]
    cv2.circle(frame, (w-30, 30), 14,
               (0, 0, 255) if on else (40, 40, 80), -1)
    if on:
        cv2.circle(frame, (w-30, 30), 22, (0, 0, 120), 2)
    cv2.putText(frame, "LED", (w-44, 60),
                cv2.FONT_HERSHEY_PLAIN, 0.9, (150, 150, 150), 1)

def draw_status(frame, status, alerts):
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 36), (20, 20, 20), -1)
    color = (0, 80, 255) if status == "ALERT" else (0, 200, 100)
    cv2.putText(frame,
                f"IR Alert System  |  {status}  |  Alerts: {alerts}",
                (10, 24), cv2.FONT_HERSHEY_PLAIN, 1.3, color, 1)

def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    _, bg = cap.read()
    bg_gray = cv2.GaussianBlur(
        cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY), (21, 21), 0)

    alert_active    = False
    last_alert_time = 0
    alert_count     = 0

    print("Running — Q to quit, R to reset background, S to save snapshot")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        gray  = cv2.GaussianBlur(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (21, 21), 0)

        delta    = cv2.absdiff(bg_gray, gray)
        thresh   = cv2.threshold(delta, SENSITIVITY, 255, cv2.THRESH_BINARY)[1]
        thresh   = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = any(cv2.contourArea(c) > MIN_AREA for c in contours)

        for c in contours:
            if cv2.contourArea(c) > MIN_AREA:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(frame, "OBJECT", (x, y-8),
                            cv2.FONT_HERSHEY_PLAIN, 1.1, (0, 0, 255), 1)

        now = time.time()

        if detected:
            alert_active = True
            if now - last_alert_time > ALERT_COOLDOWN:
                alert_count += 1
                beep()
                last_alert_time = now
                print(f"ALERT #{alert_count}: Motion detected!")
        else:
            if alert_active:
                print("Zone cleared. Standby...")
            alert_active = False

        line1 = " >> ALERT! <<   " if alert_active else "  IR Alert Sys  "
        line2 = "Motion Detected!" if alert_active else "   Standby...   "

        if alert_active:
            cv2.rectangle(frame, (0, 0),
                          (frame.shape[1]-1, frame.shape[0]-1), (0, 0, 255), 4)

        draw_status(frame, "ALERT" if alert_active else "STANDBY", alert_count)
        draw_lcd(frame, line1, line2, alert=alert_active)
        draw_led(frame, on=alert_active)
        cv2.imshow("IR Alert System", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            bg_gray = gray.copy()
            print("Background reset.")
        elif key == ord('s'):
            name = f"snapshot_{int(time.time())}.png"
            cv2.imwrite(name, frame)
            print(f"Saved: {name}")

    cap.release()
    cv2.destroyAllWindows()
    pygame.mixer.quit()

if __name__ == "__main__":
    main()
```

It works by comparing each frame against a background reference. When something moves enough to cross the threshold, it triggers the alert. Press **R** to recalibrate the background if the room lighting changes or you're getting too many false alerts.

---

## Things that might go wrong

**LCD is blank** — most likely the I²C address is wrong. Try changing `0x27` to `0x3F` in the code. Some modules ship with one, some with the other.

**LCD shows solid black blocks** — the contrast needs adjusting. There's a small blue potentiometer on the back of the I²C module. Turn it slowly until the text appears.

**IR sensor always triggered** — it's picking up ambient infrared light. Move it away from direct sunlight or fluorescent bulbs, or turn the small potentiometer on the sensor to reduce sensitivity.

**Wokwi says library not found** — just click the blue install button in the popup. It installs automatically, then click run again.

**Webcam version triggers constantly** — raise `SENSITIVITY` to around 40–60 and `MIN_AREA` to 3000. Also make sure the lighting in your room is stable.

**Wrong camera opens** — change `CAMERA_INDEX` from `0` to `1` or `2`.

---

## What I'd add next

A few ideas I haven't gotten around to yet:

- Send a **Telegram message** when motion is detected using the `python-telegram-bot` library
- Log every alert to a **CSV file** with timestamps so you can review them later
- Add a **keypad** to arm and disarm the system with a password
- Swap the Arduino for an **ESP32** and get Wi-Fi alerts on your phone
- Add a **camera snapshot** that automatically saves a photo when the alarm triggers

---

## Files in this repo

```
ir-alert-system/
│
├── hardware/
│   └── ir_alert_hardware.ino    ← for real Arduino + FC-51 sensor
│
├── wokwi/
│   └── ir_alert_wokwi.ino       ← for Wokwi browser simulation
│
├── webcam/
│   └── ir_alert_webcam.py       ← for Python + webcam
│
├── ir-alert-system.html         ← project landing page
└── README.md                    ← you're reading it
```

---

## License

MIT — do whatever you want with it.

---

*Built in the lab. Started as a weekend project, ended up here.*
