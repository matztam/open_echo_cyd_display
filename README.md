# CYD open_echo Display
<img src="https://github.com/matztam/open_echo_cyd_display/blob/main/misc/img1.jpg?raw=true" />
Ultrasonic echo waterfall display for open_echo using ESP32 CYD (Cheap Yellow Display).

## Hardware

- **ESP32 CYD** - ESP32-WROOM-32 with 2.8" ILI9341 TFT (320×240 pixels)
- **Connection:** RP2040 TX (GPIO0) → ESP32 GPIO22 (RX), GND → GND
- **Baud Rate:** 921600

## Features

- **Full screen waterfall** (320×240 pixels, right-to-left scrolling)
- **Flicker-free overlays** (depth value, vertical scale)
- **Automatic normalization** (Mean±2σ dynamic range)
- **Dead zone** (excludes sensor ringing from statistics)
- **Dynamic scale** (automatically calculated step size)

## Configuration

Edit `main.cpp` (USER CONFIGURATION section):

### Serial Communication
```cpp
#define SERIAL_BAUD 921600         // must match RP2040
#define RXD2 22                    // ESP32 RX pin (connects to RP2040 TX)
```

### Measurement Settings
```cpp
#define NUM_SAMPLES 5000           // must match RP2040
#define SPEED_OF_SOUND 330         // m/s (330=air, ~1500=water)
#define SAMPLE_TIME 1.554e-6       // must match RP2040 setup
```

### Display Settings
```cpp
#define COLUMN_WIDTH 2             // pixels per measurement
                                   // 1=320 columns (slow), 2=160, 4=80 (fast)
```

### Normalization Settings
```cpp
#define NORM_DEADZONE_M 0.30       // exclude first X meters from auto-gain
                                   // typical: 0.3-0.7m (prevents ringing)
```

## Building

```bash
pio run -t upload
```
