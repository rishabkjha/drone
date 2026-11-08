# HoverIt

Android app (Kotlin + Jetpack Compose, glassmorphism) that flies the ESP32 SuperMini drone over Wi-Fi. The drone-side receiver is [`include/HoverItRx.h`](../include/HoverItRx.h) in the flight code.

## Run the app
1. Android Studio (Ladybug or newer) > Open > the `HoverIt/HoverIt` folder (the one with `settings.gradle.kts`). Let Gradle sync (AGP 8.7.3, Kotlin 2.0.21, Compose BOM 2024.12.01, Gradle 8.11.1 wrapper).
2. Plug in the phone with USB debugging on and press Run. minSdk 26 (Android 8), landscape only.

## Set up the drone
1. The flight code in this repo already includes the receiver, [`include/HoverItRx.h`](../include/HoverItRx.h).
2. Put your phone hotspot name and password in `include/Config.h` (`WIFI_SSID` / `WIFI_PASS`).
   No hotspot? `#define HOVERIT_SOFTAP 1` and join the phone to the "HoverIt" network (password `hoverit123`).
3. Upload the flight code (see [src/README.md](../src/README.md)).
4. Turn on the hotspot, power the drone, open the app, tap Connect. It turns green when telemetry arrives.

## Screens
- **Startup:** animated HoverIt logo, then the stick screen.
- **Sticks:** left stick = throttle `THR` (Y, stays where you leave it) + yaw `YAW` (X, springs back); right stick = pitch `PIT` / roll `ROL` (both spring back). Top left shows the flight mode and what the drone is doing; top right shows the height (`cm · baro` while the barometer is in use). Land locks the throttle; Autonomous locks both sticks (`AUTOPILOT`).
- **Tuning:** the loop picker (‹ ROLL+PITCH / YAW RATE / VERT VEL ›) and the `TUNING OFF` / `TUNING ON` toggle across the top, then four 0-255 sliders: KP, KI, KD, FF. The sliders load the drone's gains for the picked loop and stay locked until you turn tuning on; changes are live. Leaving Tuning while disarmed saves the gains on the drone.
- **Options:**
  - Left: four flight-mode tiles, Stabilize (manual throttle), Loiter (50% stick holds height), Land (descend + disarm) and Autonomous. Tapping one picks it and opens the stick screen.
  - Right: **Tuning Mode** (opens Tuning), **Enable Baro** (OFF = ToF only; ON = ToF below 50 cm, barometer above; the tile shows when the barometer is in use), and **Theme** (Red / Blue).
- **Centre column on every screen:** Connect (`Searching…` while looking, green `Connected` with the drone's IP), Arm / Disarm (green disarmed, red armed; shakes if arming is refused), and Options (fills with water to the battery level and shows `battery N%`; tilt the phone and the water stays level).

The app remembers the theme, Enable Baro, the picked PID loop and the last manual mode (Stabilize or Loiter). It never starts in Land or Autonomous.

## Link (UDP, protocol v3)
Defined in [`include/HoverItRx.h`](../include/HoverItRx.h) (drone) and `app/src/main/java/com/hoverit/app/DroneLink.kt` (phone); keep the two in sync.

| Direction | Port | Rate | Bytes |
|---|---|---|---|
| phone -> drone | 4210 | 50 Hz | `'H' seq thr yaw pitch roll kp ki kd ff loop flags ver xor` (14) |
| drone -> phone | 4211 | 10 Hz | `'D' seq battery flags loop kp ki kd ff height status xor` (12) |

- `ver` = 3. `xor` = XOR of all bytes before it.
- `loop`: which PID loop the gains are for (0 ROLL+PITCH, 1 YAW RATE, 2 VERT VEL). The drone sends back its own gains for that loop, so the sliders show the drone's values.
- `flags` (phone): bit0 armed, bit1 Enable Baro, bit2 tuning sliders enabled, bits4-5 flight mode (0 Stabilize, 1 Loiter, 2 Land, 3 Autonomous).
- `flags` (drone): bit0 armed, bit1 barometer in use for height, bits4-5 mode it is flying.
- `height`: cm / 2 (0-510 cm). `status`: what the mode is doing (list in `include/FlightModeCommon.h`).

The drone broadcasts telemetry until the phone answers, so no IP needs typing.

## Safety
- Drone failsafe: no packet for 500 ms -> disarmed, sticks centred, throttle 0.
- App: shows disconnected after 1 s without telemetry and disarms itself.
- Arming needs throttle at the bottom (app and drone both check); the drone needs a fresh 0 -> 1 arm press.
