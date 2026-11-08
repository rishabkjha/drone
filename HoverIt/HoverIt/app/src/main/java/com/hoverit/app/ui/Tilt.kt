package com.hoverit.app.ui

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.view.Surface
import android.view.WindowManager
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.withFrameNanos
import androidx.compose.ui.platform.LocalContext
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min
import kotlin.math.tan

/**
 * Water in the battery button, as a row of columns that behave like a shallow
 * tank: tilting the phone moves the "level" the water wants to sit at, and the
 * columns flow towards it, push on their neighbours, overshoot and slosh back.
 * So a quick tilt sends a swell across the button that bends the surface, and a
 * still phone leaves a calm, flat surface. The amount of water never changes.
 *
 * heights[i] = how far column i stands above the flat level, in button widths.
 * Read [frame] while drawing so the drawing follows each simulation step.
 */
class WaterSurface(val columns: Int = 32) {
    val heights = FloatArray(columns)
    private val speed = FloatArray(columns)
    val frame = mutableIntStateOf(0)

    /** Phone roll in radians after smoothing (what the water is following). */
    var tilt = 0f
        private set
    internal var rawTilt = 0f

    /** Advance by dt seconds. */
    fun step(dt: Float) {
        // smooth the sensor first so hand shake does not jiggle the water
        tilt += (rawTilt - tilt) * (1f - exp(-dt / TILT_SMOOTH_S))
        val slope = tan(tilt)
        val n = columns
        val k = WAVE_SPEED * WAVE_SPEED * n * n      // wave equation, dx = 1 / n widths
        var t = dt
        var moving = false
        while (t > 0f) {
            val h = min(t, SUBSTEP_S)
            t -= h
            // distance of each column from where the tilted level wants it
            for (i in 0 until n) {
                val off = heights[i] - level(i, slope)
                val left = if (i > 0) heights[i - 1] - level(i - 1, slope) else off
                val right = if (i < n - 1) heights[i + 1] - level(i + 1, slope) else off
                speed[i] += (k * (left + right - 2f * off) - DAMPING * speed[i]) * h
            }
            for (i in 0 until n) {
                heights[i] += speed[i] * h
                if (abs(speed[i]) > 1e-4f) moving = true
            }
        }
        if (moving || abs(rawTilt - tilt) > 1e-3f) frame.intValue++
    }

    // Tilted level for column i: right side up for a + roll
    private fun level(i: Int, slope: Float) = ((i + 0.5f) / columns - 0.5f) * slope

    companion object {
        const val WAVE_SPEED = 1.6f       // button widths per second: one swing across ~0.6 s
        const val DAMPING = 2.2f          // 1/s: a few gentle swings, then still
        const val TILT_SMOOTH_S = 0.12f   // sensor smoothing
        const val SUBSTEP_S = 0.004f      // keeps the simulation stable
        const val MAX_TILT = 1.2f         // ~70 deg
    }
}

/**
 * A [WaterSurface] driven by how far the phone is rolled: gravity sensor
 * (accelerometer as a fallback), remapped for the landscape rotation. Stays
 * flat when the phone has no such sensor.
 */
@Composable
fun rememberWater(): WaterSurface {
    val context = LocalContext.current
    val water = remember { WaterSurface() }

    DisposableEffect(context) {
        val sm = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
        val sensor = sm?.getDefaultSensor(Sensor.TYPE_GRAVITY) ?: sm?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        @Suppress("DEPRECATION")
        val wm = context.getSystemService(Context.WINDOW_SERVICE) as? WindowManager
        val listener = object : SensorEventListener {
            override fun onSensorChanged(e: SensorEvent) {
                // sensor axes are the phone's portrait axes: turn them into screen axes
                @Suppress("DEPRECATION")
                val rot = wm?.defaultDisplay?.rotation ?: Surface.ROTATION_90
                val dx = e.values[0]
                val dy = e.values[1]
                val (sx, sy) = when (rot) {
                    Surface.ROTATION_90 -> -dy to dx
                    Surface.ROTATION_180 -> -dx to -dy
                    Surface.ROTATION_270 -> dy to -dx
                    else -> dx to dy
                }
                // gravity reads "up" in screen axes; the level is perpendicular to it
                val a = atan2(-sx, sy)
                water.rawTilt = max(-WaterSurface.MAX_TILT, min(WaterSurface.MAX_TILT, a))
            }

            override fun onAccuracyChanged(s: Sensor?, accuracy: Int) {}
        }
        if (sm != null && sensor != null) sm.registerListener(listener, sensor, SensorManager.SENSOR_DELAY_GAME)
        onDispose { sm?.unregisterListener(listener) }
    }

    LaunchedEffect(water) {
        var last = 0L
        while (true) {
            withFrameNanos { now ->
                if (last != 0L) water.step(((now - last) / 1e9f).coerceIn(0f, 0.05f))
                last = now
            }
        }
    }
    return water
}
