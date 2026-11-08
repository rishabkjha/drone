# Drone

ESP32-C3 SuperMini 3 inch quad: the flight controller and the **HoverIt** phone app that flies and tunes it over Wi-Fi.

| Folder | What it is | Guide |
|---|---|---|
| `src/`, `include/`, `platformio.ini` | Flight controller (PlatformIO, Arduino core): IMU, PIDs, flight modes, height hold, app link | [src/README.md](src/README.md) |
| `HoverIt/` | HoverIt Android app (Kotlin + Jetpack Compose), the phone controller | [HoverIt/README.md](HoverIt/README.md) |

## Flight controller

Open the repo root in PlatformIO, set your hotspot name and password in `include/Config.h`, then Upload and Monitor (115200). Wiring, flight modes, tuning and troubleshooting are in [src/README.md](src/README.md).

## HoverIt app

Open `HoverIt/HoverIt` in Android Studio (Ladybug or newer), let Gradle sync, plug in the phone with USB debugging on and press Run. minSdk 26 (Android 8), landscape only.

Screens: Loiter (sticks), Tuning (KP / KI / KD / FF sliders per loop, sent live to the drone), and Options (flight mode, Enable Baro, theme). Connect, Arm and battery level sit in the centre column on every screen.

## How they talk

UDP over Wi-Fi, protocol v3. Phone → drone on port 4210 at 50 Hz (sticks, arm, mode, gains). Drone → phone on port 4211 at 10 Hz (battery, height, status, gains). The drone broadcasts until the phone answers, so no IP needs typing. The drone disarms after 500 ms with no packet.

The drone side of the link is `include/HoverItRx.h`. The `HoverIt/HoverItRx.h` copy and the link table in `HoverIt/README.md` are the older protocol v1, kept as they came with the app: use the one in `include/`.

## Licence

The flight controller is based on the Carbon Aeronautics quadcopter flight controller and is CC BY-NC-SA 4.0 (non-commercial, share alike). See [src/README.md](src/README.md#15-credits-and-licence).
