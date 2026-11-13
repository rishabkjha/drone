# HoverIt

Flight controller firmware (ESP32-C3) and Android controller app for a 3 inch quad.

HoverIt is the whole build: the firmware that flies the ESP32-C3 SuperMini quad, and the phone app that flies and tunes it over Wi-Fi.

| Part | Folder | What it is | Guide |
|---|---|---|---|
| Firmware | `src/`, `include/`, `platformio.ini` | Flight controller (PlatformIO, Arduino core): IMU, PIDs, flight modes, height hold, app link | [src/README.md](src/README.md) |
| App | `HoverIt/` | Android controller app (Kotlin + Jetpack Compose), glassmorphism theme | [HoverIt/README.md](HoverIt/README.md) |
| App (Liquid Glass) | `HoverIt-LiquidGlass/` | The same app with a Liquid Glass look; installs alongside the original | [HoverIt-LiquidGlass/README.md](HoverIt-LiquidGlass/README.md) |

## Firmware

Open the repo root in PlatformIO, set your hotspot name and password in `include/Config.h`, then Upload and Monitor (115200). Wiring, flight modes, tuning and troubleshooting are in [src/README.md](src/README.md).

## App

Open `HoverIt/HoverIt` in Android Studio (Ladybug or newer), let Gradle sync, plug in the phone with USB debugging on and press Run. minSdk 26 (Android 8), landscape only.

There are two versions of the app side by side: `HoverIt/` (the original glassmorphism theme) and `HoverIt-LiquidGlass/` (Liquid Glass: lens-like see-through glass, light that follows the phone's tilt, springy presses). They fly the drone the same way and can both be installed at once.

Screens: sticks, Tuning (KP / KI / KD / FF sliders per loop, sent live to the drone), and Options (flight mode, Enable Baro, theme). Connect, Arm and battery level sit in the centre column on every screen.

## How the app and firmware talk

UDP over Wi-Fi, protocol v3. Phone → drone on port 4210 at 50 Hz (sticks, arm, mode, gains). Drone → phone on port 4211 at 10 Hz (battery, height, status, gains). The drone broadcasts until the phone answers, so no IP needs typing. The drone disarms after 500 ms with no packet.

The drone side of the link is [`include/HoverItRx.h`](include/HoverItRx.h); the app side is `HoverIt/HoverIt/app/src/main/java/com/hoverit/app/DroneLink.kt`. Byte layout: [HoverIt/README.md](HoverIt/README.md#link-udp-protocol-v3).

## Future scope

The long-term goal is to grow HoverIt into a two-drone surveillance system:

- **Mother drone:** a 10-11 inch prop carrier drone that stays airborne over the mission area.
- **Mini drone (the HoverIt 3 inch quad):** carries a camera, flies a set shape (for example a square or grid) to survey a small area, and avoids obstacles on its own (active obstacle avoidance).
- **Air refueling:** the mini drone returns to the airborne mother drone to recharge, then goes back out, without landing.

**Why:** a small drone surveys cheaply but has a short flight time. Recharging it from an airborne carrier lets one surveillance mission cover a larger survey area and keep surveying for longer, which improves the endurance of the mission.

**Planned steps:** reliable position hold (optical flow, see the limits in [src/README.md](src/README.md#14-limits)), a camera and obstacle sensor on the mini drone, autonomous shape flights (the AUTONOMOUS square is the first step), precise approach and docking to the mother drone in flight, and in-flight charging.

## Licence

The flight controller is based on the Carbon Aeronautics quadcopter flight controller and is CC BY-NC-SA 4.0 (non-commercial, share alike). See [src/README.md](src/README.md#15-credits-and-licence).
