package com.hoverit.app.ui

import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.Spring
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.spring
import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.BoxScope
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import com.hoverit.app.FlightMode
import com.hoverit.app.FlightState
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.gestures.drag
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.runtime.snapshotFlow
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.input.pointer.positionChange
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.launch
import kotlin.math.roundToInt

private val ThumbShape = RoundedCornerShape(50)   // round droplet

/** -1..1 -> 0..255, centre = 128 */
private fun toByte(v: Float): Int = (((v.coerceIn(-1f, 1f) + 1f) / 2f) * 255f).roundToInt()

/** 0..255 -> -1..1 */
private fun fromByte(b: Int): Float = b.coerceIn(0, 255) / 255f * 2f - 1f

/**
 * Liquid glass droplet in the accent colours: the background bends through
 * its rim, and it swells and brightens while you hold it.
 */
@Composable
private fun Thumb(active: Boolean, modifier: Modifier = Modifier, locked: Boolean = false) {
    val lift by animateFloatAsState(
        if (active) 1f else 0f,
        spring(dampingRatio = 0.5f, stiffness = Spring.StiffnessMediumLow),
        label = "thumbLift",
    )
    Box(
        modifier
            .graphicsLayer { val sc = 1f + 0.12f * lift; scaleX = sc; scaleY = sc }
            .shadow((8f + 14f * lift).dp, ThumbShape, ambientColor = HoverColors.Coral, spotColor = HoverColors.Magenta)
            .glass(shape = ThumbShape, tintBrush = HoverColors.Accent, borderWidth = 1.4.dp, lift = lift),
    )
}

/**
 * Square glass pad with a crosshair and a gradient thumb (stick screen).
 * Outputs two 8-bit values: x (0 = left, 255 = right) and y (0 = bottom, 255 = top).
 * Dragging is relative, so touching the pad never makes the thumb jump.
 * X always springs back to centre; Y springs back unless [holdY] (throttle).
 * [lockX] / [lockY]: that axis ignores your finger and shows [initialX] / [initialY]
 * (Land / Autonomous). [holdMark] draws the "hold" band across the middle (Loiter).
 * [overlay] is drawn on top, e.g. the mode badge.
 */
@Composable
fun Joystick(
    modifier: Modifier,
    holdY: Boolean,
    initialX: Int,
    initialY: Int,
    labelX: String,
    labelY: String,
    onChange: (x: Int, y: Int) -> Unit,
    lockX: Boolean = false,
    lockY: Boolean = false,
    holdMark: Boolean = false,
    overlay: @Composable BoxScope.() -> Unit = {},
) {
    val p = LocalGlass.current
    val scope = rememberCoroutineScope()
    val ax = remember { Animatable(fromByte(initialX)) }
    val ay = remember { Animatable(fromByte(initialY)) }
    var active by remember { mutableStateOf(false) }
    val change by rememberUpdatedState(onChange)
    val lx by rememberUpdatedState(lockX)
    val ly by rememberUpdatedState(lockY)

    LaunchedEffect(Unit) {
        snapshotFlow { toByte(ax.value) to toByte(ay.value) }.collect { (x, y) -> change(x, y) }
    }
    // A LOCKED axis shows the app's value (e.g. the app zeroing the throttle in
    // Land). Free axes never follow it: the stick's own spring back to centre
    // must not be interrupted by its own echo.
    LaunchedEffect(initialX, lockX) {
        if (lockX && toByte(ax.value) != initialX) ax.animateTo(fromByte(initialX), spring(dampingRatio = 0.8f))
    }
    LaunchedEffect(initialY, lockY) {
        if (lockY && toByte(ay.value) != initialY) ay.animateTo(fromByte(initialY), spring(dampingRatio = 0.8f))
    }
    val lockedAll = lockX && lockY
    val dim by animateFloatAsState(if (lockedAll) 0.55f else 1f, label = "padDim")

    BoxWithConstraints(modifier.glass()) {
        val density = LocalDensity.current
        val thumb = minOf(maxWidth, maxHeight) * 0.26f
        val thumbPx = with(density) { thumb.toPx() }
        val rangeX = (constraints.maxWidth - thumbPx) / 2f
        val rangeY = (constraints.maxHeight - thumbPx) / 2f

        val bandColor = HoverColors.Coral.copy(alpha = 0.14f)
        Canvas(Modifier.fillMaxSize().graphicsLayer { alpha = dim }) {
            val c = center
            val stroke = 1.3.dp.toPx()
            if (holdMark) {
                // Loiter: the band around 50 % where the drone holds its height
                val band = (size.height - thumbPx) / 2f * 0.10f
                drawRect(
                    bandColor,
                    topLeft = Offset(0f, c.y - band),
                    size = androidx.compose.ui.geometry.Size(size.width, band * 2),
                )
            }
            val dash = PathEffect.dashPathEffect(floatArrayOf(6.dp.toPx(), 6.dp.toPx()))
            drawLine(p.gridLine, Offset(c.x, 0f), Offset(c.x, size.height), stroke, pathEffect = if (lockX) dash else null)
            drawLine(p.gridLine, Offset(0f, c.y), Offset(size.width, c.y), stroke, pathEffect = if (lockY) dash else null)
        }

        Box(
            Modifier
                .fillMaxSize()
                .pointerInput(rangeX, rangeY) {
                    awaitEachGesture {
                        val down = awaitFirstDown(requireUnconsumed = false)
                        active = true
                        scope.launch { ax.stop(); ay.stop() }
                        drag(down.id) { ch ->
                            val d = ch.positionChange()
                            ch.consume()
                            scope.launch {
                                if (!lx) ax.snapTo((ax.value + d.x / rangeX).coerceIn(-1f, 1f))
                                if (!ly) ay.snapTo((ay.value - d.y / rangeY).coerceIn(-1f, 1f))
                            }
                        }
                        active = false
                        val back = spring<Float>(dampingRatio = 0.45f, stiffness = Spring.StiffnessMediumLow)
                        if (!lx) scope.launch { ax.animateTo(0f, back) }
                        if (!holdY && !ly) scope.launch { ay.animateTo(0f, back) }
                    }
                },
        )

        Thumb(
            active && !lockedAll,
            locked = lockedAll,
            modifier = Modifier
                .align(Alignment.Center)
                .size(thumb)
                .offset { IntOffset((ax.value * rangeX).roundToInt(), (-ay.value * rangeY).roundToInt()) }
                .graphicsLayer { alpha = dim },
        )

        overlay()
    }
}

/**
 * Tall glass slider for the Tuning screen. 8-bit output, 0 at the bottom.
 * The value holds where you leave it. When [value] changes from outside (the
 * drone's gains arriving) the thumb glides there; [onChange] fires only for
 * your own moves. Locked and dimmed while [enabled] is false.
 */
@Composable
fun GainSlider(
    title: String,
    value: Int,
    modifier: Modifier,
    onChange: (Int) -> Unit,
    onRelease: () -> Unit,
    enabled: Boolean = true,
) {
    val p = LocalGlass.current
    val scope = rememberCoroutineScope()
    val pos = remember { Animatable(value.coerceIn(0, 255) / 255f) }
    var active by remember { mutableStateOf(false) }
    val change by rememberUpdatedState(onChange)
    val release by rememberUpdatedState(onRelease)
    val canMove by rememberUpdatedState(enabled)
    val dim by animateFloatAsState(if (enabled) 1f else 0.45f, label = "dim")

    LaunchedEffect(value) {
        if (!active) pos.animateTo(value.coerceIn(0, 255) / 255f, spring(dampingRatio = 0.8f, stiffness = Spring.StiffnessMediumLow))
    }

    BoxWithConstraints(modifier.graphicsLayer { alpha = dim }.glass()) {
        val density = LocalDensity.current
        val pad = 10.dp
        val header = 56.dp
        val thumb = minOf(maxWidth - pad * 2, 104.dp)
        val travelPx = with(density) { (maxHeight - header - pad - thumb).toPx() }.coerceAtLeast(1f)

        // level fill behind the thumb
        val fillFrac = pos.value
        Box(
            Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .height(with(density) { (travelPx * fillFrac).toDp() } + thumb / 2 + pad)
                .background(
                    Brush.verticalGradient(
                        listOf(HoverColors.Coral.copy(alpha = 0.28f), HoverColors.Magenta.copy(alpha = 0.10f)),
                    ),
                ),
        )

        Text(
            title,
            style = LabelStyle,
            color = p.text,
            modifier = Modifier.align(Alignment.TopCenter).padding(top = 10.dp),
        )
        Text(
            "${(pos.value * 255f).roundToInt()}",
            style = ReadoutStyle,
            color = p.textDim,
            modifier = Modifier.align(Alignment.TopCenter).padding(top = 32.dp),
        )

        Box(
            Modifier
                .fillMaxSize()
                .pointerInput(travelPx) {
                    awaitEachGesture {
                        val down = awaitFirstDown(requireUnconsumed = false)
                        if (!canMove) return@awaitEachGesture
                        active = true
                        var cur = pos.value
                        var last = (cur * 255f).roundToInt()
                        drag(down.id) { ch ->
                            val d = ch.positionChange()
                            ch.consume()
                            cur = (cur - d.y / travelPx).coerceIn(0f, 1f)
                            val target = cur
                            scope.launch { pos.snapTo(target) }
                            val b = (cur * 255f).roundToInt()
                            if (b != last) { last = b; change(b) }
                        }
                        active = false
                        release()
                    }
                },
        )

        Thumb(
            active,
            Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = pad)
                .size(thumb, thumb * 0.95f)
                .offset { IntOffset(0, -(pos.value * travelPx).roundToInt()) },
        )
    }
}

/** Big glass tile for the Options screen; [accent] tints it while [selected]. */
@Composable
fun OptionTile(
    label: String,
    modifier: Modifier,
    selected: Boolean = false,
    subLabel: String? = null,
    accent: Color = HoverColors.Magenta,
    onClick: () -> Unit,
) {
    GlassButton(
        label = label,
        subLabel = subLabel,
        tint = if (selected) accent else null,
        modifier = modifier,
        onClick = onClick,
    )
}

/**
 * Flight mode tile (Options, left side): a coloured stripe and dot in the
 * mode's colour, its name and a one-line description (none for Autonomous). The picked mode glows
 * in its colour. Tapping opens the stick screen in that mode.
 */
@Composable
fun ModeTile(mode: FlightMode, selected: Boolean, flying: Boolean, modifier: Modifier, onClick: () -> Unit) {
    val p = LocalGlass.current
    val c = modeColor(mode)
    GlassButton(
        label = mode.title,
        subLabel = if (mode.blurb == null) null else if (selected && flying) "flying now" else mode.blurb,
        labelStyle = LabelStyle.copy(fontSize = 14.sp, letterSpacing = 0.sp),
        tint = if (selected) c.copy(alpha = 0.85f) else null,
        modifier = modifier,
        onClick = onClick,
    )
}

/**
 * Tuning screen, top right: enables / disables the four sliders. Off = sliders
 * locked and the drone ignores them (switching off while disarmed saves).
 */
@Composable
fun TuningToggle(enabled: Boolean, synced: Boolean, modifier: Modifier, onClick: () -> Unit) {
    val tint by animateColorAsState(if (enabled) HoverColors.GreenDark else HoverColors.RedDark, tween(350), label = "tuneTint")
    GlassButton(
        label = if (enabled) "TUNING ON" else "TUNING OFF",
        subLabel = when {
            !enabled -> "tap to unlock sliders"
            !synced -> "waiting for drone"
            else -> "live \u00b7 tap to save"
        },
        tint = tint,
        modifier = modifier,
        onClick = onClick,
        background = {
            // a small switch drawn on the right
            Canvas(Modifier.fillMaxSize()) {
                val w = 38.dp.toPx(); val h = 20.dp.toPx()
                val x = size.width - w - 14.dp.toPx(); val y = (size.height - h) / 2
                drawRoundRect(Color.White.copy(alpha = 0.25f), Offset(x, y), androidx.compose.ui.geometry.Size(w, h), CornerRadius(h / 2))
                val knob = h / 2 - 3.dp.toPx()
                drawCircle(Color.White, knob, Offset(if (enabled) x + w - h / 2 else x + h / 2, y + h / 2))
            }
        },
    )
}

/** Mode name, plus what the drone is doing once it is connected. Kept to one quiet line. */
@Composable
fun BoxScope.ModeBadge(mode: FlightMode, flight: FlightState, connected: Boolean) {
    val p = LocalGlass.current
    val status = when {
        !connected -> null
        flight.mode != mode -> "switching\u2026"
        else -> flight.statusText
    }
    Text(
        if (status != null) "${mode.title.uppercase()}  \u00b7  $status" else mode.title.uppercase(),
        style = LabelStyle.copy(fontSize = 11.sp, letterSpacing = 1.5.sp),
        color = p.textDim,
        modifier = Modifier.align(Alignment.TopStart).padding(16.dp),
    )
}

/** Height, top right of the right pad; only while the drone is connected. */
@Composable
fun BoxScope.HeightBadge(flight: FlightState, connected: Boolean) {
    if (!connected) return
    Text(
        if (flight.training) "${flight.heightCm} cm \u00b7 baro" else "${flight.heightCm} cm",
        style = ReadoutStyle.copy(fontSize = 11.sp),
        color = LocalGlass.current.textDim,
        modifier = Modifier.align(Alignment.TopEnd).padding(16.dp),
    )
}

/** Big centred note on a fully locked pad (Autonomous). */
@Composable
fun BoxScope.LockedNote(text: String) {
    Text(
        text,
        style = LabelStyle.copy(fontSize = 12.sp, letterSpacing = 3.sp),
        color = LocalGlass.current.textDim,
        textAlign = TextAlign.Center,
        modifier = Modifier.align(Alignment.TopCenter).padding(top = 72.dp),
    )
}

/**
 * Picks which PID loop the Tuning sliders change: tap the name or the arrows
 * to step through [names].
 */
@Composable
fun LoopPicker(
    names: List<String>,
    index: Int,
    synced: Boolean,
    modifier: Modifier,
    onStep: (Int) -> Unit,
) {
    Row(modifier, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        GlassButton("\u2039", Modifier.weight(0.55f), onClick = { onStep(-1) })
        GlassButton(
            label = names.getOrElse(index) { "?" },
            subLabel = if (synced) "loop ${index + 1} of ${names.size}" else "loading from drone\u2026",
            tint = HoverColors.Magenta,
            modifier = Modifier.weight(2f),
            onClick = { onStep(1) },
        )
        GlassButton("\u203A", Modifier.weight(0.55f), onClick = { onStep(1) })
    }
}
