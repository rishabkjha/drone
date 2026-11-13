package com.hoverit.app

import android.app.Application
import android.content.Context
import android.os.SystemClock
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

/** Fly = the two-stick screen, flown in [HoverItViewModel.flightMode]. */
enum class Screen { Splash, Fly, Tuning, Options }

class HoverItViewModel(app: Application) : AndroidViewModel(app) {

    private val prefs = app.getSharedPreferences("hoverit", Context.MODE_PRIVATE)

    val link = DroneLink(app, viewModelScope)

    var screen by mutableStateOf(Screen.Splash)
        private set
    /** The control page Options returns to. */
    private var lastControlScreen = Screen.Fly

    /** Theme button: false = red (coral/magenta), true = blue. */
    var blueTheme by mutableStateOf(prefs.getBoolean("blue", false))
        private set
    var armed by mutableStateOf(false)
        private set
    /** Flight mode picked in Options (sent every frame; the drone switches at once). */
    var flightMode by mutableStateOf(
        FlightMode.entries.getOrElse(prefs.getInt("mode", 0)) { FlightMode.STABILIZE }
            .let { if (it == FlightMode.STABILIZE || it == FlightMode.LOITER) it else FlightMode.STABILIZE },
    )
        private set
    /** Options > Enable Baro: the drone holds height on the barometer above 50 cm
     *  (the ToF trains it below). Remembered between app starts. */
    var training by mutableStateOf(prefs.getBoolean("baro", false))
        private set
    /** Tuning screen: the top-right toggle. Sliders only move (and the drone only takes them) while on. */
    var tuningEnabled by mutableStateOf(false)
        private set
    /** Bumped when an arm request is refused, so the button can shake. */
    var armRefusals by mutableIntStateOf(0)
        private set

    // 8-bit outputs
    var throttle by mutableIntStateOf(0); private set
    var yaw by mutableIntStateOf(128); private set
    var pitch by mutableIntStateOf(128); private set
    var roll by mutableIntStateOf(128); private set
    // Tuning: the drone holds the gains (PIDTune.h). Picking a loop loads the
    // drone's values into the sliders; until then the sliders are locked.
    var kp by mutableIntStateOf(0); private set
    var ki by mutableIntStateOf(0); private set
    var kd by mutableIntStateOf(0); private set
    var ff by mutableIntStateOf(0); private set
    var loop by mutableIntStateOf(prefs.getInt("loop", 0).coerceIn(0, PID_LOOPS.size - 1)); private set
    /** True once the sliders show the drone's gains for [loop]. */
    var gainsSynced by mutableStateOf(false); private set
    private var lastTouchMs = 0L

    init {
        pushFrame()
        // Losing the drone disarms the app too, so it never re-arms by itself
        // when the link comes back. If the drone reports disarmed for a full
        // second while we think it is armed (it refused or failsafed), follow it.
        viewModelScope.launch {
            combine(link.connected, link.droneArmed) { c, a -> c to a }.collectLatest { (connected, droneArmed) ->
                if (!connected) {
                    setArmedInternal(false)
                } else if (armed && !droneArmed) {
                    delay(1000)
                    setArmedInternal(false)
                }
            }
        }
        // Follow the drone's gains: on a new loop, after a reconnect, or when the
        // drone ignored a slider move. Never while a slider is being moved.
        viewModelScope.launch {
            while (true) {
                delay(100)
                syncGains()
            }
        }
    }

    private fun syncGains() {
        val g = link.gains.value
        if (!link.connected.value || g == null || g.loop != loop) {
            gainsSynced = false
            return
        }
        val idle = SystemClock.elapsedRealtime() - lastTouchMs > 600
        if (!gainsSynced || idle) {
            if (kp != g.kp || ki != g.ki || kd != g.kd || ff != g.ff) {
                kp = g.kp; ki = g.ki; kd = g.kd; ff = g.ff
                pushFrame()
            }
            gainsSynced = true
        }
    }

    fun finishSplash() {
        if (screen == Screen.Splash) screen = Screen.Fly
    }

    fun goTo(target: Screen) {
        if (target == Screen.Fly || target == Screen.Tuning) lastControlScreen = target
        // sticks you can no longer see go back to centre; throttle holds
        if (target != Screen.Fly) { yaw = 128; pitch = 128; roll = 128 }
        // leaving Tuning locks the sliders (and saves on the drone while disarmed)
        if (target != Screen.Tuning) tuningEnabled = false
        screen = target
        pushFrame()
    }

    /** Centre "Options" button: opens Options, or goes back from it. */
    fun toggleOptions() {
        goTo(if (screen == Screen.Options) lastControlScreen else Screen.Options)
    }

    fun back(): Boolean {
        if (screen == Screen.Options) { goTo(lastControlScreen); return true }
        if (screen == Screen.Tuning) { goTo(Screen.Fly); return true }
        return false
    }

    fun toggleConnect() {
        if (link.enabled.value) {
            setArmedInternal(false)
            link.stop()
        } else {
            link.start()
        }
    }

    /** Returns false when arming was refused (no drone, or throttle not at the bottom). */
    fun toggleArm(): Boolean {
        if (armed) { setArmedInternal(false); return true }
        // Land / Auto: the throttle stick is locked and unused, so it can't block arming
        if (flightMode.throttleLocked) throttle = 0
        if (!link.connected.value || throttle > 15) {
            armRefusals++
            return false
        }
        setArmedInternal(true)
        return true
    }

    /**
     * Options: pick a flight mode, then the stick screen opens. The throttle
     * value is made safe for the new mode, because a locked stick (Land, Auto)
     * may be sitting anywhere:
     *  - Land / Auto picked on the ground: throttle to 0, so arming works.
     *  - Stabilize / Loiter picked in the air after Land / Auto: throttle to
     *    50 % (Loiter: hold; Stabilize: 1500 us, near hover) instead of
     *    wherever it was, which could be 0 = motors off.
     */
    fun selectMode(m: FlightMode) {
        val inAir = armed && link.connected.value && link.flight.value.airborne
        if (!inAir && m.throttleLocked) throttle = 0
        if (inAir && flightMode.throttleLocked && !m.throttleLocked) throttle = 128
        flightMode = m
        // only the manual modes are remembered: never start the app in Land / Auto
        prefs.edit().putInt("mode", if (m.throttleLocked) 0 else m.ordinal).apply()
        goTo(Screen.Fly)
    }

    fun toggleTraining() {
        training = !training
        prefs.edit().putBoolean("baro", training).apply()
        pushFrame()
    }

    fun toggleTuning() { tuningEnabled = !tuningEnabled; pushFrame() }

    fun toggleTheme() {
        blueTheme = !blueTheme
        prefs.edit().putBoolean("blue", blueTheme).apply()
    }

    fun setLeftStick(x: Int, y: Int) {
        if (flightMode.sticksLocked) return
        yaw = x
        if (!flightMode.throttleLocked) throttle = y
        pushFrame()
    }
    fun setRightStick(x: Int, y: Int) {
        if (flightMode.sticksLocked) return
        roll = x; pitch = y; pushFrame()
    }

    fun setGain(which: String, value: Int) {
        if (!gainsSynced || !tuningEnabled) return
        lastTouchMs = SystemClock.elapsedRealtime()
        when (which) {
            "kp" -> kp = value
            "ki" -> ki = value
            "kd" -> kd = value
            "ff" -> ff = value
        }
        pushFrame()
    }

    /** Slider let go: give the drone a moment to echo the value back. */
    fun gainReleased() { lastTouchMs = SystemClock.elapsedRealtime() }

    /** Step through the PID loops (wraps). The drone's gains for it load next. */
    fun stepLoop(delta: Int) {
        val n = PID_LOOPS.size
        loop = ((loop + delta) % n + n) % n
        gainsSynced = false
        prefs.edit().putInt("loop", loop).apply()
        pushFrame()
    }

    private fun setArmedInternal(value: Boolean) {
        if (armed != value) { armed = value; pushFrame() }
    }

    private fun pushFrame() {
        link.frame = ControlFrame(
            throttle = throttle, yaw = yaw, pitch = pitch, roll = roll,
            kp = kp, ki = ki, kd = kd, ff = ff, loop = loop,
            armed = armed, training = training,
            tuning = screen == Screen.Tuning && tuningEnabled,
            mode = flightMode,
        )
    }

    override fun onCleared() {
        link.stop()
        super.onCleared()
    }
}
