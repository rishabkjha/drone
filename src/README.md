# FlightController_v2

Flight controller for the **ESP32-C3 SuperMini 3 inch quad**, flown and tuned from the **HoverIt** phone app.

This is the single-file `SuperMini_Simple.ino` (the version that flies) split into one class per file. The loops, filters, gains, flight modes and app protocol are the same. In the simulator, this version and the single file gave the same motor outputs and angles at every step, with and without motor vibration. It has not been flown yet.

---

## Quick reference: suggested gains and trim

**App Tuning sliders** (the number in brackets is what the slider shows, 0-255):

| Loop | KP | KI | KD | FF |
|---|---|---|---|---|
| ROLL+PITCH | 0.60 (77) | 3.5 (112) | 0.030 (96) | 0.10 (26) |
| YAW RATE | 2.0 (102) | 12 (122) | 0 (0) | 0.40 (51) |
| VERT VEL | 1.0 (51) | 0.5 (26) | 0 (0) | 0.65 (55) |

**Fixed in `Config.h`:** `ANGLE_KP 4.0`, `ANGLE_FF 0.5`, `HEIGHT_KP 1.0`, `HEIGHT_FF 0.8`, `HOVER_US 1335`.

**Trim in `Config.h`:** `TRIM_ROLL_DEG 1.2`, `TRIM_PITCH_DEG 1.0`. Drifts back → pitch up 0.5; forward → pitch down 0.5; left → roll up 0.5; right → roll down 0.5.

Details: tuning in [section 8](#8-tuning-from-the-app), trim in [section 9](#9-fixing-drift-trim).

---

## 1. Contents

1. [What you need](#2-what-you-need)
2. [Wiring](#3-wiring)
3. [Files and classes](#4-files-and-classes)
4. [Uploading](#5-uploading)
5. [First power-up checks (props OFF)](#6-first-power-up-checks-props-off)
6. [Flying](#7-flying)
7. [Tuning from the app](#8-tuning-from-the-app)
8. [Fixing drift (trim)](#9-fixing-drift-trim)
9. [Reading the Serial Monitor](#10-reading-the-serial-monitor)
10. [Settings reference (Config.h)](#11-settings-reference-configh)
11. [How it works](#12-how-it-works)
12. [Troubleshooting](#13-troubleshooting)
13. [Limits](#14-limits)
14. [Credits and licence](#15-credits-and-licence)

---

## 2. What you need

| Part | Notes |
|---|---|
| ESP32-C3 SuperMini | Arduino ESP32 core 3.x (IDE) or espressif32 6.9 (PlatformIO) |
| MPU6050 | I2C 0x68. The Y arrow points to the front and the chip is upside down; the code works this out at boot. |
| VL53L0X ToF | I2C 0x29, pointing down. Height below 50 cm, and it trains the barometer. |
| BMP280 barometer | I2C 0x76 (or 0x77, found at boot). Height above 50 cm with the Enable Baro switch on, and the backup when the ToF is lost. Cover the hole with a bit of foam to keep prop wash off it. |
| 4 ESCs | Set to 400 Hz PWM |
| 2S LiPo | Through an 18K / 10K divider to GPIO 0 |
| Phone | HoverIt app, protocol v3 (the current app; nothing to change) |

**Libraries:** `ESP32Servo`, `Adafruit_VL53L0X`. WiFi, Wire and Preferences come with the ESP32 core. The BMP280 is read directly (no library).

---

## 3. Wiring

| Signal | GPIO |
|---|---|
| I2C SDA / SCL (MPU6050 + VL53L0X) | 9 / 8 |
| ESC front left (prop ACW) | 5 |
| ESC front right (prop CW) | 6 |
| ESC rear left (prop CW) | 10 |
| ESC rear right (prop ACW) | 3 |
| Battery divider | 0 |

```
        FRONT
  FL (5, ACW)  FR (6, CW)
         \     /
          \   /
          /   \
         /     \
  RL (10, CW)  RR (3, ACW)
        BACK
```

---

## 4. Files and classes

Each file holds one class, and the class has one global object the rest of the code uses.

| File | Class | Object | What it does |
|---|---|---|---|
| `FlightController_v2.ino` | | | `setup()` and the 500 Hz `loop()`: sensors, crash cut, flight mode, PID, motors, Serial print |
| `Config.h` | | | **Every setting**: Wi-Fi, pins, gains, trims, hover throttle, limits |
| `HoverItRx.h` | | | App link over Wi-Fi/UDP (protocol v3): sticks, arm, mode, sliders in; battery, height, status, gains out |
| `IMU.h` | `Imu` | `imu` | MPU6050 read, gyro calibration, auto-level, 1D Kalman per axis, vibration gate |
| `PIDFF.h` | `Pid` | | PID: P on error, trapezoidal I clamped, D on measurement (filtered), FF on setpoint, I preload |
| `AttitudePID.h` | `Attitude` | `attitude` | Sticks → angle setpoint → angle P + stick-speed FF → rate PIDs → motor corrections |
| `Motors.h` | `Motors` | `motors` | 4 ESCs at 400 Hz, quad X mixer, boot motor test |
| `Battery.h` | `Battery` | `battery` | 2S voltage → 0-255 level for the app |
| `HeightFusion.h` | `HeightEstimator` | `height` | VL53L0X + accelerometer → height and climb rate |
| `HeightPID.h` | `HeightHold` | `heightHold` | Held height → climb rate → VERT VEL PID → throttle, touchdown detection |
| `PIDTune.h` | `Tuner` | `tuner` | App Tuning sliders → gains, saved to flash |
| `FlightModeCommon.h` | `FlightMode` (interface) | | Mode numbers, status list, the `Cmd` a mode returns |
| `ModeStabilize.h` | `ModeStabilize` | | STABILIZE |
| `ModeLoiter.h` | `ModeLoiter` | | LOITER (height hold) |
| `ModeLand.h` | `ModeLand` | | LAND |
| `ModeAuto.h` | `ModeAuto` | | AUTONOMOUS (square, then land) |
| `FlightModes.h` | `FlightModes` | `flightModes` | Runs the mode picked in the app, handles switching mode in the air |

Each loop runs these steps in order:

```
imu.update → height.update → hoverItUpdate → tuner.update → battery.update
→ crash cut → flightModes.update (throttle) → attitude.update (OutR/OutP/OutY)
→ motors.mix → telemetry to the app → Serial print (5 Hz)
```

The old FlightController's `ModeAutonomous.h` and `TrainFlight.h` are replaced by `ModeAuto.h`. The app's Options switch (once called Baro Training) is now **Enable Baro** (section 7).

---

## 5. Uploading

**Arduino IDE**
1. Unzip `FlightController_v2.zip`. Keep the folder name `FlightController_v2`, because it must match the .ino name.
2. Open `FlightController_v2.ino`. The `.h` files open as tabs.
3. Board **ESP32C3 Dev Module**, **USB CDC On Boot: Enabled**.
4. Set your hotspot name and password in `Config.h` (`WIFI_SSID`, `WIFI_PASS`). To have the drone make its own network instead, uncomment `#define HOVERIT_SOFTAP 1`. The network is "HoverIt", password `hoverit123`.
5. Upload, then open the Serial Monitor at **115200**.

**PlatformIO**
Unzip `FlightController_v2_PlatformIO.zip` and open the folder. `platformio.ini` already lists the libraries, so just Upload and Monitor. Its `src/` holds the same files as the Arduino folder, with the .ino renamed to `main.cpp`.

**Gains carry over:** this version reads the same flash slot as SuperMini_Simple (`simpletune`), so the gains you saved from the app are loaded. Set `LOAD_SAVED_GAINS = false` in `Config.h` to ignore flash and use the defaults.

---

## 6. First power-up checks (props OFF)

1. **Power up sitting level and still.** Boot takes about 5 s: IMU calibration, ToF zero, then the motor test. With `AUTO_LEVEL` on, the position at power-up counts as level.
2. **Motor test:** each motor spins alone for 1 s and Serial names it. If a different motor spins, that ESC is on the wrong pin. Fix this first, because it reverses roll or pitch.
3. **Tilt by hand** and watch Serial:
   - right side down: `ANG R` goes **+**
   - nose down: `ANG P` goes **+**
   - turn clockwise (seen from above): `RATE Y` goes **+**
4. **Arm** with the throttle at the bottom, then raise it just above the bottom (the PIDs start at 1060 us):
   - press the right side down: the **right** motors speed up
   - press the nose down: the **front** motors speed up
   - yaw stick right: **front left + rear right** speed up
5. Then props on, a low hover over grass, tethered if you can.

If a stick moves the drone the wrong way, flip `ROLL_SIGN`, `PITCH_SIGN` or `YAW_SIGN` in `Config.h`.

---

## 7. Flying

**Arming:** the app's arm button works only with the throttle stick at or below 15 / 255. With no packet for 500 ms the drone disarms (failsafe). Tilting past 60° cuts the motors until you disarm (crash cut).

**Modes** (pick in the app's Options; switching in the air is smooth, from the current throttle):

| Mode | Throttle stick | Roll / pitch / yaw |
|---|---|---|
| **STABILIZE** | 0-255 = 1000-2000 us, capped at 1800 us. It hovers at about **85 / 255 (1335 us)**. | Stick = angle (full stick = 30°), yaw = rate (full stick = 75°/s) |
| **LOITER** | Middle (±10 %) = **hold height**; above = climb, below = sink (up to 40 cm/s). From the ground, push above the middle to take off. Hold below the middle on the ground and the motors stop. Max height 50 cm (3 m with Enable Baro on). | Same as STABILIZE |
| **LAND** | Ignored. Descends at 25 cm/s, stops the motors on touchdown and disarms. | Same as STABILIZE |
| **AUTONOMOUS** | Ignored. **Arm = go:** motors idle 1 s, climb to 50 cm, hover 2 s, fly a **30 × 30 cm square** (forward, right, back, left; 2 s hover at each corner), land, disarm. | Ignored (the app locks the sticks) |

LOITER needs a ToF or barometer reading: without either it refuses to take off, and Serial says "no ToF or barometer reading".

**Height limit:** with Enable Baro off the drone never goes above **50 cm**, in every mode. In LOITER the held height stops at 50 cm. In STABILIZE the throttle is capped as it nears 50 cm, so it slows and stops there even with the stick high. With Enable Baro on the limit is 3 m.

**Enable Baro switch** (app Options; remembered by the app):
- **OFF:** the ToF gives the height, max 50 cm. If the ToF reads -1 (lost) in the air, the barometer takes over at once and hands back when the ToF returns.
- **ON:** below 50 cm the ToF is used and the barometer is trained against it. Above 50 cm the barometer gives the height, up to 3 m. Back below 45 cm it returns to the ToF.
- **Training:** while the ToF is in use the barometer's zero follows it, and its scale (how many cm per cm of real height) is fitted from the climb. The scale is saved when you disarm and used from the next flight on, so the barometer is already trained before take-off; each flight refines it.
- **No jumps:** both sensors feed the same filter, so switching between them changes only the measurement, not the height or climb rate. The app's height badge shows `baro` while the barometer is in use.

**AUTONOMOUS, how it flies the square:** there is no position sensor, so each side is flown by time. It leans 5° toward the side (on top of your trim), then 5° the other way to stop, then hovers. Height is held exactly as in LOITER, with the same VERT VEL gains and `HOVER_US`. So the square is only as good as the trim: trim the drone first (section 9) so it hovers in LOITER without drifting. To stop it at any point, pick LOITER (it holds height where it is), LAND, or disarm. Fly it with Enable Baro **off**: at 50 cm the ToF is more precise, and 50 cm is where Enable Baro hands over to the barometer.

If the sides come out too short, raise `AUTO_SIDE_GAIN` (1.2 = 20 % longer); too long, lower it. If it drifts back after each side, lower `AUTO_BRAKE_GAIN`; if it keeps sliding on, raise it.

**For a hands-off hover:** fly LOITER (no throttle work), power up on a flat floor, and set the trim once (section 9). See section 14 for why it still drifts slowly sideways.

---

## 8. Tuning from the app

Tuning screen → **ENABLE** at the top right. Pick the loop at the top left. The sliders jump to the drone's current gains, and moving a slider changes the gain live. Leaving Tuning (or DISABLE) **while disarmed** saves the gains to flash, and Serial prints them as lines you can paste into `Config.h` as the new defaults.

The sliders show 0-255; gain = slider / 255 × max.

| App loop | What it controls | Max (KP / KI / KD / FF) | **Suggested** gains (slider) |
|---|---|---|---|
| **ROLL+PITCH** | Rate loop for roll and pitch, us per deg/s | 2 / 8 / 0.08 / 1 | KP 0.60 (77), KI 3.5 (112), KD 0.030 (96), FF 0.10 (26) |
| **YAW RATE** | Yaw rate loop, us per deg/s | 5 / 25 / 0.10 / 2 | KP 2.0 (102), KI 12 (122), KD 0 (0), FF 0.40 (51) |
| **VERT VEL** | Climb-rate loop for LOITER / LAND, us per cm/s | 5 / 5 / 0.5 / 3 | KP 1.0 (51), KI 0.5 (26), KD 0 (0), FF 0.65 (55) |

**Fixed in `Config.h`** (not on the app):

| Setting | Value | Meaning |
|---|---|---|
| `ANGLE_KP` | 4.0 | Angle error (deg) → wanted rate (deg/s). Higher = stiffer. |
| `ANGLE_FF` | 0.5 | Stick speed passed straight to the rate loop, so it tilts the moment the stick moves |
| `HEIGHT_KP` | 1.0 | Height error (cm) → climb rate (cm/s) |
| `HEIGHT_FF` | 0.8 | Climb stick passed straight through |
| `HOVER_US` | 1335 | Throttle that hovers, used as LOITER's starting point |

**Step by step** (props on, low over grass, tethered first; change one gain at a time by about 10 %, land and disarm between big changes):

1. **Start from the suggested gains** above. If you lost them, set `LOAD_SAVED_GAINS = false` for one boot (defaults load), then back to `true`.
2. **HOVER_US first.** In STABILIZE, find the throttle where it just hovers. Slider value × 1000 / 255 + 1000 = us (85 → 1335). Put it in `HOVER_US`.
3. **ROLL+PITCH KP.** Raise it until the drone starts a fast shake, then go back about 30 %. Too low feels mushy and slow to level.
4. **ROLL+PITCH KD.** Raise it slowly to stop the bounce after a quick stick flick. Stop if the motors get hot or noisy.
5. **ROLL+PITCH KI.** Raise it until it holds its angle against a breeze and stays level in a long hover. Too much gives a slow wobble.
6. **YAW RATE.** KP until yaw stops sharply without bouncing back, then KI until it no longer slowly turns by itself.
7. **FF on both** (ROLL+PITCH FF, YAW RATE FF): raise until the sticks feel instant, back off if it overshoots when you let go.
8. **Angle loop** (`ANGLE_KP` in `Config.h`, 3-5): higher snaps back to level faster, too high wobbles slowly.
9. **VERT VEL in LOITER.** KP until the height holds firmly, back off if it bobs. KI until it stops sagging over time.
10. **Trim** (section 9) last, once it flies well.

Leave Tuning while **disarmed** to save. Serial then prints the gains as lines you can paste into `Config.h` as the new defaults.

**What to change when:**

| You see | Change |
|---|---|
| Fast shaking / buzzing after a stick flick | ROLL+PITCH KD up a little; if it gets worse, KP down |
| Slow wobble, overshoots | ROLL+PITCH KP down, or `ANGLE_KP` down |
| Feels mushy, slow to level | ROLL+PITCH KP up, or `ANGLE_KP` up (3-5) |
| Sticks feel late | ROLL+PITCH FF up (0.1-0.2), `ANGLE_FF` up (0.5-0.8) |
| Yaw drifts or bounces back | YAW RATE KI up / KP down |
| LOITER bobs up and down | VERT VEL KP down |
| LOITER sags or is slow to hold | VERT VEL KI up, check `HOVER_US` |

Change one thing at a time, by about 10 %.

---

## 9. Fixing drift (trim)

With the sticks centred, the drone holds what it thinks is level. A small error in that "level" becomes a steady drift.

1. **Always power up on a flat floor** (`AUTO_LEVEL = true` takes the power-up position as level).
2. If it still slides one way in STABILIZE, change these in `Config.h` by 0.5 at a time:

| It drifts | Change |
|---|---|
| Forward | `TRIM_PITCH_DEG` more **negative** |
| Back | `TRIM_PITCH_DEG` more **positive** |
| Right | `TRIM_ROLL_DEG` more **negative** |
| Left | `TRIM_ROLL_DEG` more **positive** |

Current values: `TRIM_ROLL_DEG = 1.2`, `TRIM_PITCH_DEG = 1.0`.

To use a fixed level instead of the power-up reading, set `AUTO_LEVEL = false` and enter the sensor's level error in `LEVEL_TRIM_ROLL` / `LEVEL_TRIM_PITCH` (Serial prints the reading at boot).

---

## 10. Reading the Serial Monitor

At boot it prints: the chip orientation, the level reading and trim, the accel at rest, the ToF ground reading, the gains, the motor test, and Wi-Fi status. While running it prints a line 5 times a second:

```
LINK ARM LOITER | ANG R 0.1/ 1.2 P -0.2/ 1.0 | RATE R 0.3/ 0.5 P ... | OUT 2 -1 0 | M 1352 1349 1355 1350 | vib 0.08 ACC | H 44.2/45.0 v 0.4 tof 44.0 baro  43.1 TOF max 50 | K 0.604 3.514 0.0301 | bat 7.92V | loop 610/1180 us late 0
```

| Field | Meaning |
|---|---|
| `LINK` / `----` | App connected or not |
| `ARM` / `---` | Armed; then the mode being flown; `CUT` = crash cut |
| `ANG R a/b P a/b` | Angle / setpoint, deg |
| `RATE R P Y` | Gyro / wanted rate, deg/s |
| `OUT` | PID correction roll pitch yaw, us |
| `M` | Motors FL FR RL RR, us |
| `vib x ACC/GYRO` | Accel shaking in g. `GYRO` means the vibration gate is ignoring the accelerometer. |
| `H a/b v tof` | Height / held height (cm), climb rate (cm/s), raw ToF height (-1 = no reading) |
| `baro x TOF/BARO max N` | Barometer height (cm, trained), which sensor is in use, height limit (50 or 300) |
| `K` | ROLL+PITCH KP KI KD now in use |
| `bat` | Battery volts |
| `loop avg/max us late n` | Loop work time; must stay under 2000. `late` counts overruns. |

Also: `[HoverIt] N packets/s ...` every 5 s (50 expected), `[Tune] ...` when you tune, and `MODE:`, `LOITER:`, `LAND:` messages.

---

## 11. Settings reference (Config.h)

| Group | Setting | Value | Notes |
|---|---|---|---|
| Wi-Fi | `WIFI_SSID`, `WIFI_PASS` | your hotspot | `HOVERIT_SOFTAP 1` = own network |
| Loop | `LOOP_US` | 2000 | 500 Hz |
| ESCs | `ESC_FREQ_HZ` | 400 | |
| | `THROTTLE_MAX_US` | 1800 | Headroom for the PID |
| | `ESC_IDLE_US` | 1100 | No motor below this while flying (raise to 1150-1180 if a motor stutters) |
| | `PID_ON_US` | 1060 | PIDs run from here; below it every I term is cleared |
| | `READY_SPIN_US` | 1060 | Boot motor test |
| Sticks | `MAX_ANGLE_DEG` | 30 | Full roll/pitch stick |
| | `MAX_YAW_RATE` | 75 | deg/s |
| | `ROLL/PITCH/YAW_SIGN` | 1 | -1 to reverse |
| | `CRASH_TILT` | 60 | deg |
| Gains | `ANGLE_KP`, `ANGLE_FF` | 4.0, 0.5 | |
| | `TRIM_ROLL_DEG`, `TRIM_PITCH_DEG` | 1.2, 1.0 | Section 9 |
| | `DTERM_LPF_HZ` | 30 | Rate D filter |
| | `DEFAULT_RP / YAW / VERT` | see section 8 | First boot only, then flash |
| IMU | `MPU_DLPF_CFG` | 0x03 | ~44 Hz |
| | `FORWARD_AXIS`, `FORWARD_SIGN` | 'Y', 1 | Chip arrow pointing to the front |
| | `AUTO_LEVEL` | true | Power-up position = level |
| | `ACC_VIB_MAX / OK` | 0.25 / 0.12 g | Vibration gate out / back in |
| | `ACC_SETTLE_MS` | 500 | Quiet time before the accel is used again |
| | `ACC_AGREE_DEG` | 15 | Accel angle this far off is ignored |
| Height | `HOVER_US` | 1335 | |
| | `HOLD_MIN_US / MAX_US` | 1150 / 1800 | LOITER/LAND throttle limits |
| | `LOITER_CLIMB` | 40 cm/s | Full stick |
| | `LOITER_DEADBAND` | 0.10 | ±10 % around the middle = hold |
| | `LAND_CMS` | 25 cm/s | |
| | `MAX_HEIGHT_CM` | 50 | Limit with Enable Baro off |
| | `CEIL_DECEL`, `CEIL_KV` | 100 cm/s², 1.0 | How hard STABILIZE brakes toward the limit |
| Autonomous | `AUTO_HEIGHT_CM`, `AUTO_SIDE_CM` | 50, 30 | Square height and side |
| | `AUTO_TILT_DEG`, `AUTO_TILT_RATE` | 5°, 40°/s | Lean while moving, how fast the lean changes |
| | `AUTO_SIDE_GAIN`, `AUTO_BRAKE_GAIN` | 1.0, 1.0 | Push time and stopping lean (section 7) |
| | `AUTO_CLIMB_CMS` | 25 cm/s | Take-off climb |
| | `AUTO_SPOOL_S`, `AUTO_SETTLE_S`, `AUTO_CORNER_S` | 1, 2, 2 s | Idle before lift-off, hover before the square, hover at each corner |
| | `AUTO_PATH` | F, R, B, L | Order of the sides |
| Barometer | `BARO_HANDOFF_CM` | 50 | ToF below, barometer above (5 cm hysteresis) |
| | `MAX_HEIGHT_BARO_CM` | 300 | Limit with Enable Baro on |
| | `HEIGHT_TAU_BARO_S` | 1.0 s | Baro/accel blend (0.35 s while the accel is gated) |
| | `BARO_OFFSET_TAU_S` | 2 s | How fast the baro zero follows the ToF |
| | `BARO_SCALE_MIN / MAX` | 0.6 / 1.6 | Allowed trained scale |
| | `CLEAR_BARO_SCALE` | false | `true` for one flash = forget the saved scale |
| | `LEASH_CM` | 40 | Held height stays this close to the real one |
| | `HEIGHT_TAU_S` | 0.4 s | ToF/accel blend (0.25 s while the accel is gated) |
| | `TOF_LOST_MS` | 300 | No ToF this long = height unknown |
| Battery | `BATT_DIVIDER`, `BATT_CAL` | 2.8, 1.062 | `BATT_CAL` = multimeter V / printed V |
| | `BATT_EMPTY_V / FULL_V` | 6.8 / 8.4 V | App shows 0 / full |

---

## 12. How it works

**Attitude (from RX_RP_Tune.ino, the controller tuned in a few minutes)**
- **Kalman filter per axis:** the gyro predicts the angle, the accelerometer corrects it (gyro std 4°/s, accel std 3°). It runs with the real measured loop time.
- **Vibration gate:** when motor vibration shakes the accelerometer past 0.25 g, the angle runs on the gyro alone. The accel comes back only after 0.5 s below 0.12 g, and only if it agrees within 15°.
- **Cascade:** stick → angle setpoint (+ trim) → `ANGLE_KP` × error + `ANGLE_FF` × stick speed → wanted rate → rate PID on the gyro → motor correction. D works on the gyro (no kick when the stick moves), with a 30 Hz filter. I is trapezoidal and clamped.
- **Mixer (quad X):** `FL = T+R−P+Y`, `FR = T−R−P−Y`, `RL = T+R+P−Y`, `RR = T−R+P+Y`.

**Height (LOITER, LAND)**
- The VL53L0X distance × cos(roll) × cos(pitch), minus the ground reading at boot = height. It is ignored past 35° tilt or 2 m.
- The BMP280 is read every 45 ms (64-bit Bosch maths) and turned into cm. Its zero follows the ToF while the ToF is in use, and its scale is fitted against the ToF (blended with the saved value, stored in flash at disarm).
- One complementary filter blends the measurement (ToF or barometer) with the accelerometer (height, climb rate, accel bias). Going to the barometer, its zero is set to the current height; coming back to the ToF, the difference is glided out over 1 s. So the climb rate never jumps.
- Held height → `HEIGHT_KP` → wanted climb rate → VERT VEL PID → throttle around `HOVER_US`, tilt-compensated. Switching into LOITER/LAND in the air preloads the I term from the current throttle, so there is no jump.
- **Touchdown:** below 10 cm (or ToF below 6 cm) and still for 0.6 s, or pushing down without moving for 2 s.

**App link:** phone → drone 50 Hz on UDP 4210; drone → phone 10 Hz on UDP 4211. The drone holds the master copy of the gains and sends the selected loop's gains back, so the sliders always show the drone's values.

---

## 13. Troubleshooting

| Problem | Check |
|---|---|
| App never connects | Serial `not on hotspot ... status 1` = name wrong or 5 GHz-only hotspot (use 2.4 GHz); `status 4` = wrong password. Or use `HOVERIT_SOFTAP 1`. |
| Link drops in bursts | Serial packets/s well below 50. Try `WIFI_TX_POWER` in `HoverItRx.h` (`WIFI_POWER_11dBm` or `_5dBm`). |
| "packets rejected ... OLD version" | Reinstall the current HoverIt app (protocol 3, 14 bytes) |
| Arm refused | Throttle stick not at the bottom |
| Flips at takeoff | Motor order or prop direction wrong (section 6, steps 2-4) |
| Leans/drifts one way | Power up level; trim (section 9) |
| Drift grows during a flight, Serial shows `GYRO` | Vibration. Soft-mount the MPU6050, balance props. |
| LOITER won't take off | No ToF and no barometer. Check wiring; the ToF must see the floor. |
| Won't climb above 50 cm | Expected with Enable Baro off. Turn it on in Options. |
| Height wanders with Enable Baro on | On the bench, motors off, watch `baro` for a minute: it should stay within a few cm. If it drifts, shade it from light and air (foam). A bad saved scale: flash once with `CLEAR_BARO_SCALE true`. |
| LOITER sags at first | Set `HOVER_US` to the throttle it hovers at in STABILIZE |
| Battery % wrong | Measure with a multimeter, set `BATT_CAL` |
| `late` > 0 often | Loop overrunning, usually I2C or Serial; tell me the numbers |

---

## 14. Limits

- **No position hold.** Nothing on the drone can see it sliding sideways, so with the sticks centred it holds level, not position. A small, slow drift always remains (wind, a fraction of a degree of tilt, gyro drift). The trim makes it slow but can't make it zero. To truly hold a spot (sticks centred = stays put, push = moves, release = holds the new spot) it needs an **optical flow sensor**. The Matek 3901-L0X does flow and height in one and would replace the VL53L0X.
- Height is limited to 50 cm with Enable Baro off and 3 m with it on. Above 50 cm the barometer holds to about ±10-15 cm, not as tight as the ToF.
- AUTONOMOUS flies the square by time, not by position: expect it to be a few cm off, and more if the trim is off or there's a draught.

---

## 15. Credits and licence

Based on the Carbon Aeronautics quadcopter flight controller (v1.0, 2022), through working_drone.ino and RX_RP_Tune.ino: its Kalman filter, PID equation, cascaded angle → rate loops and gains. That work is licensed **CC BY-NC-SA 4.0**, so this is too: credit Carbon Aeronautics, non-commercial use only, share changes under the same licence. Carbon Aeronautics does not endorse this version.
