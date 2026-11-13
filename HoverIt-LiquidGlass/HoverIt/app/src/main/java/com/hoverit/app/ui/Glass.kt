package com.hoverit.app.ui

import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.Spring
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.spring
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.clickable
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsPressedAsState
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawWithContent
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.graphics.lerp
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sin

/** Rounder than before: liquid glass panels read as soft, continuous shapes. */
val GlassShape = RoundedCornerShape(28.dp)

/**
 * Liquid glass button. Pressing swells it a little on a soft spring and lights
 * the glass from under your finger.
 */
@Composable
fun GlassButton(
    label: String,
    modifier: Modifier = Modifier,
    tint: Color? = null,
    subLabel: String? = null,
    onClick: () -> Unit,
    background: @Composable () -> Unit = {},
    labelStyle: TextStyle = LabelStyle,
) {
    val p = LocalGlass.current
    val interaction = remember { MutableInteractionSource() }
    val pressed by interaction.collectIsPressedAsState()
    val press by animateFloatAsState(
        if (pressed) 1f else 0f,
        spring(dampingRatio = 0.55f, stiffness = Spring.StiffnessMediumLow),
        label = "press",
    )
    var touch by remember { mutableStateOf<Offset?>(null) }
    Box(
        modifier
            .fillMaxSize()
            .graphicsLayer { val sc = 1f + 0.035f * press; scaleX = sc; scaleY = sc }
            .pointerInput(Unit) {
                // follow the finger for the glow; never consumes, so the click still works
                awaitEachGesture {
                    touch = awaitFirstDown(requireUnconsumed = false).position
                    while (true) {
                        val e = awaitPointerEvent()
                        val c = e.changes.firstOrNull() ?: break
                        if (!c.pressed) break
                        touch = c.position
                    }
                }
            }
            .glass(tint = tint, press = press, touch = touch)
            .clickable(interactionSource = interaction, indication = null, onClick = onClick),
        contentAlignment = Alignment.Center,
    ) {
        background()
        Column(Modifier.padding(horizontal = 8.dp), horizontalAlignment = Alignment.CenterHorizontally) {
            Text(label, style = labelStyle, color = if (tint != null) Color.White else p.text, textAlign = TextAlign.Center)
            if (subLabel != null) {
                Text(
                    subLabel,
                    style = LabelStyle.copy(fontSize = 12.sp),
                    color = if (tint != null) Color.White.copy(alpha = 0.8f) else p.textDim,
                    textAlign = TextAlign.Center,
                )
            }
        }
    }
}

/** Green when the drone is connected, red when not. */
@Composable
fun ConnectButton(enabled: Boolean, connected: Boolean, address: String?, modifier: Modifier, onClick: () -> Unit) {
    val tint by animateColorAsState(
        if (connected) HoverColors.GreenDark else HoverColors.RedDark,
        tween(450),
        label = "connectTint",
    )
    // gentle pulse while searching for the drone
    val pulse = rememberInfiniteTransition(label = "searchPulse")
    val glow by pulse.animateFloat(
        0.55f, 1f,
        infiniteRepeatable(tween(700), RepeatMode.Reverse),
        label = "glow",
    )
    val searching = enabled && !connected
    GlassButton(
        label = when {
            connected -> "Connected"
            searching -> "Searching…"
            else -> "Connect"
        },
        subLabel = if (connected) address else if (searching) "tap to stop" else null,
        tint = if (searching) tint.copy(alpha = glow) else tint,
        modifier = modifier,
        onClick = onClick,
    )
}

/** Green when disarmed, red when armed. Shakes when arming is refused. */
@Composable
fun ArmButton(armed: Boolean, refusals: Int, modifier: Modifier, onClick: () -> Unit) {
    val tint by animateColorAsState(
        if (armed) HoverColors.RedDark else HoverColors.GreenDark,
        tween(350),
        label = "armTint",
    )
    val shake = remember { Animatable(0f) }
    LaunchedEffect(refusals) {
        if (refusals == 0) return@LaunchedEffect
        for (x in listOf(14f, -12f, 9f, -6f, 3f, 0f)) shake.animateTo(x, tween(45))
    }
    GlassButton(
        label = if (armed) "Disarm" else "Arm",
        subLabel = if (armed) "ARMED" else "disarmed",
        tint = tint,
        modifier = modifier.graphicsLayer { translationX = shake.value * density },
        onClick = onClick,
    )
}

/**
 * Options button that doubles as the battery gauge: filled like water in a
 * container, level = battery / 255 (127 -> ~50 %).
 */
@Composable
fun OptionsBatteryButton(battery: Int, connected: Boolean, active: Boolean, modifier: Modifier, onClick: () -> Unit) {
    val level by animateFloatAsState(
        (battery.coerceIn(0, 255) / 255f).takeIf { connected } ?: 0f,
        spring(dampingRatio = 0.55f, stiffness = 40f),
        label = "level",
    )
    val low = (1f - (level / 0.25f)).coerceIn(0f, 1f)   // blend to red below 25 %
    val waterTop = lerp(HoverColors.WaterLight, HoverColors.RedLight, low)
    val waterBottom = lerp(HoverColors.WaterDark, HoverColors.RedDark, low)
    val percent = (battery.coerceIn(0, 255) * 100 + 127) / 255
    // tilt the phone and the water flows to stay level with the world (Tilt.kt)
    val water = rememberWater()

    GlassButton(
        label = "Options",
        subLabel = if (connected) "battery $percent%" else "battery --",
        modifier = modifier,
        onClick = onClick,
        background = {
            Canvas(Modifier.fillMaxSize()) {
                water.frame.intValue   // redraw on every simulation step
                if (level <= 0.001f) return@Canvas
                val w = size.width
                val h = size.height
                val surface = h * (1f - level)
                val cols = water.heights
                val n = cols.size
                fun yAt(i: Int) = surface - cols[i] * w
                // smooth curve through the columns, no outline: just the water body
                val path = Path().apply {
                    moveTo(0f, h)
                    lineTo(0f, yAt(0))
                    for (i in 0 until n - 1) {
                        val x0 = (i + 0.5f) / n * w
                        val x1 = (i + 1.5f) / n * w
                        quadraticTo(x0, yAt(i), (x0 + x1) / 2f, (yAt(i) + yAt(i + 1)) / 2f)
                    }
                    quadraticTo((n - 0.5f) / n * w, yAt(n - 1), w, yAt(n - 1))
                    lineTo(w, h)
                    close()
                }
                var top = h
                for (i in 0 until n) top = minOf(top, yAt(i))
                drawPath(
                    path,
                    Brush.verticalGradient(
                        listOf(waterTop.copy(alpha = 0.8f), waterTop.copy(alpha = 0.7f), waterBottom.copy(alpha = 0.85f)),
                        startY = top, endY = h,
                    ),
                )
            }
        },
    )
}
