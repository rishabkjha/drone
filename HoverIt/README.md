# HoverIt

Android app (Kotlin + Jetpack Compose, glassmorphism) that flies the ESP32 SuperMini drone over Wi-Fi, plus the drone-side receiver `HoverItRx.h`.

## Run the app
1. Android Studio (Ladybug or newer) > Open > the `HoverIt` folder. Let Gradle sync (AGP 8.7.3, Kotlin 2.0.21, Compose BOM 2024.12.01, Gradle 8.11.1 wrapper).
2. Plug in the phone with USB debugging on and press Run. minSdk 26 (Android 8), landscape only.

## Set up the drone
1. Copy `HoverItRx.h` next to your flight controller sketch.
2. Put your phone hotspot name and password in `HOTSPOT_SSID` / `HOTSPOT_PASS`.
   No hotspot? `#define HOVERIT_SOFTAP 1` before the include and join the phone to the "HoverIt" network (password `hoverit123`).
3. Follow the example at the bottom of the header.
4. Turn on the hotspot, power the drone, open the app, tap Connect. It turns green when telemetry arrives.

## Screens
- Startup: animated HoverIt logo, then Loiter.
- Loiter: left stick = throttle (Y, holds) + yaw (X, springs back); right stick = pitch/roll (both spring back).
- Tuning: KP, KI, KD, RR sliders, 0-255, saved on the phone.
- Options: Loiter Mode, Tuning Mode, Autonomous Mode (1-bit toggle), Light/Dark Theme.
- Centre column on every screen: Connect (green connected / red not), Arm/Disarm (green disarmed / red armed), Options (fills with water to battery/255).

## Link (UDP)
| Direction | Port | Rate | Bytes |
|---|---|---|---|
| phone -> drone | 4210 | 50 Hz | `'H' seq thr yaw pitch roll kp ki kd rr flags ver xor` (13) |
| drone -> phone | 4211 | 10 Hz | `'D' seq battery flags xor` (5) |

`flags` (phone): bit0 armed, bit1 autonomous, bit2 tuning screen open. `flags` (drone): bit0 armed.
The drone broadcasts telemetry until the phone answers, so no IP needs typing.

## Safety
- Drone failsafe: no packet for 500 ms -> disarmed, sticks centred, throttle 0.
- App: shows disconnected after 1 s without telemetry and disarms itself.
- Arming needs throttle at the bottom (app and drone both check); the drone needs a fresh 0 -> 1 arm press.
