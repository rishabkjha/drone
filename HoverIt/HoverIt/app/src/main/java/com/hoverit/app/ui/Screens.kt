package com.hoverit.app.ui

import androidx.activity.compose.BackHandler
import androidx.compose.animation.AnimatedContent
import androidx.compose.ui.unit.IntOffset
import androidx.compose.animation.SizeTransform
import androidx.compose.animation.AnimatedContentTransitionScope
import androidx.compose.animation.ContentTransform
import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.FastOutSlowInEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.Spring
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.spring
import androidx.compose.animation.core.tween
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.scaleIn
import androidx.compose.animation.scaleOut
import androidx.compose.animation.slideInHorizontally
import androidx.compose.animation.slideOutHorizontally
import androidx.compose.animation.togetherWith
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.graphics.ColorMatrix
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.hoverit.app.FlightMode
import com.hoverit.app.FlightState
import com.hoverit.app.HoverItViewModel
import com.hoverit.app.PID_LOOPS
import com.hoverit.app.R
import com.hoverit.app.Screen
import kotlinx.coroutines.delay

@Composable
fun HoverItApp(vm: HoverItViewModel) {
    HoverItTheme(blue = vm.blueTheme) {
        Box(Modifier.fillMaxSize()) {
            Backdrop()
            BackHandler(enabled = vm.screen == Screen.Options || vm.screen == Screen.Tuning) { vm.back() }

            AnimatedContent(
                targetState = vm.screen == Screen.Splash,
                transitionSpec = {
                    (fadeIn(tween(700, delayMillis = 150)) + scaleIn(tween(700, delayMillis = 150), initialScale = 0.92f)) togetherWith
                            (fadeOut(tween(450)) + scaleOut(tween(450), targetScale = 1.18f))
                },
                label = "splash",
            ) { splash ->
                if (splash) SplashScreen(onDone = vm::finishSplash) else ControlDeck(vm)
            }
        }
    }
}

/**
 * bg_HoverIt image, slowly breathing. The blue theme cross-fades to the same
 * image recoloured blue (pink glow -> blue glow).
 */
@Composable
private fun Backdrop() {
    val p = LocalGlass.current
    val t = rememberInfiniteTransition(label = "bg")
    val s by t.animateFloat(1f, 1.08f, infiniteRepeatable(tween(9000, easing = FastOutSlowInEasing), RepeatMode.Reverse), label = "bgScale")
    val painter = painterResource(R.drawable.bg_hoverit)
    val breathe = Modifier.fillMaxSize().graphicsLayer { scaleX = s; scaleY = s }
    if (p.blue < 1f) {
        Image(painter, null, breathe, contentScale = ContentScale.Crop)
    }
    if (p.blue > 0f) {
        Image(
            painter, null, breathe.graphicsLayer { alpha = p.blue },
            contentScale = ContentScale.Crop,
            colorFilter = ColorFilter.colorMatrix(SwapRedBlue),
        )
    }
}

/** Turns the background's pink/red glow into a sky-to-royal blue glow (black stays black). */
private val SwapRedBlue = ColorMatrix(
    floatArrayOf(
        0.15f, 0f, 0f, 0f, 0f,
        0.45f, 0.2f, 0f, 0f, 0f,
        1.0f, 0f, 0.3f, 0f, 0f,
        0f, 0f, 0f, 1f, 0f,
    ),
)

@Composable
private fun SplashScreen(onDone: () -> Unit) {
    val appear = remember { Animatable(0f) }
    LaunchedEffect(Unit) {
        appear.animateTo(1f, spring(dampingRatio = 0.6f, stiffness = Spring.StiffnessVeryLow))
        delay(700)
        onDone()
    }
    val wiggle = rememberInfiniteTransition(label = "tilde")
    val dx by wiggle.animateFloat(-6f, 6f, infiniteRepeatable(tween(900, easing = FastOutSlowInEasing), RepeatMode.Reverse), label = "dx")

    val logo = TextStyle(fontFamily = LogoFont, fontSize = 84.sp, brush = HoverColors.Accent)
    val tilde = TextStyle(fontFamily = LogoFont, fontSize = 44.sp)
    Box(
        Modifier
            .fillMaxSize()
            .clickable(remember { MutableInteractionSource() }, indication = null, onClick = onDone),
        contentAlignment = Alignment.Center,
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.graphicsLayer {
                alpha = appear.value.coerceIn(0f, 1f)
                val sc = 0.75f + 0.25f * appear.value
                scaleX = sc; scaleY = sc
            },
        ) {
            Text("~~", style = tilde.copy(color = HoverColors.Coral), modifier = Modifier.graphicsLayer { translationX = dx * density })
            Text("HoverIt", style = logo, modifier = Modifier.padding(horizontal = 10.dp))
            Text("~~", style = tilde.copy(color = HoverColors.Magenta), modifier = Modifier.graphicsLayer { translationX = -dx * density })
        }
    }
}

/** Fades + rises each centre button in turn when the deck first appears. */
@Composable
private fun Modifier.staggerIn(index: Int): Modifier {
    val a = remember { Animatable(0f) }
    LaunchedEffect(Unit) {
        delay(120L * index)
        a.animateTo(1f, spring(dampingRatio = 0.7f, stiffness = Spring.StiffnessLow))
    }
    return graphicsLayer {
        alpha = a.value.coerceIn(0f, 1f)
        translationY = (1f - a.value) * 40f * density
    }
}

private fun AnimatedContentTransitionScope<Screen>.sideTransition(fromLeft: Boolean): ContentTransform {
    val sign = if (fromLeft) -1 else 1
    // springs instead of fixed curves: panels settle with a soft overshoot (same as HoverIt Liquid)
    val slide = spring<IntOffset>(dampingRatio = 0.78f, stiffness = Spring.StiffnessLow)
    val grow = spring<Float>(dampingRatio = 0.62f, stiffness = Spring.StiffnessLow)
    return ((slideInHorizontally(slide) { sign * it / 3 } +
            fadeIn(tween(360, delayMillis = 60)) + scaleIn(grow, initialScale = 0.86f)) togetherWith
            (slideOutHorizontally(tween(300)) { -sign * it / 5 } + fadeOut(tween(220)) + scaleOut(tween(300), targetScale = 0.9f)))
        // don't clip to the pane while animating: panels may slide past its edge next to the centre buttons
        .using(SizeTransform(clip = false))
}

@Composable
private fun ControlDeck(vm: HoverItViewModel) {
    val enabled by vm.link.enabled.collectAsState()
    val connected by vm.link.connected.collectAsState()
    val battery by vm.link.battery.collectAsState()
    val address by vm.link.droneAddress.collectAsState()
    val flight by vm.link.flight.collectAsState()

    Row(
        Modifier.fillMaxSize().padding(horizontal = 36.dp, vertical = 26.dp),
        horizontalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        AnimatedContent(
            targetState = vm.screen,
            transitionSpec = { sideTransition(fromLeft = true) },
            modifier = Modifier.weight(420f).fillMaxHeight(),
            label = "left",
        ) { s -> LeftPane(s, vm, flight, connected) }

        Column(
            Modifier.weight(290f).fillMaxHeight(),
            verticalArrangement = Arrangement.spacedBy(16.dp),
        ) {
            ConnectButton(enabled, connected, address, Modifier.weight(1f).staggerIn(0), vm::toggleConnect)
            ArmButton(vm.armed, vm.armRefusals, Modifier.weight(1f).staggerIn(1)) { vm.toggleArm() }
            OptionsBatteryButton(battery, connected, vm.screen == Screen.Options, Modifier.weight(1f).staggerIn(2), vm::toggleOptions)
        }

        AnimatedContent(
            targetState = vm.screen,
            transitionSpec = { sideTransition(fromLeft = false) },
            modifier = Modifier.weight(420f).fillMaxHeight(),
            label = "right",
        ) { s -> RightPane(s, vm, flight, connected) }
    }
}

private val TuningHeader = 64.dp

@Composable
private fun LeftPane(s: Screen, vm: HoverItViewModel, flight: FlightState, connected: Boolean) {
    when (s) {
        Screen.Tuning -> Column(Modifier.fillMaxSize(), verticalArrangement = Arrangement.spacedBy(16.dp)) {
            LoopPicker(PID_LOOPS, vm.loop, vm.gainsSynced, Modifier.fillMaxWidth().height(TuningHeader), vm::stepLoop)
            Row(Modifier.weight(1f), horizontalArrangement = Arrangement.spacedBy(40.dp)) {
                GainSlider("KP", vm.kp, Modifier.weight(1f).fillMaxHeight(), { vm.setGain("kp", it) }, vm::gainReleased, vm.gainsSynced && vm.tuningEnabled)
                GainSlider("KI", vm.ki, Modifier.weight(1f).fillMaxHeight(), { vm.setGain("ki", it) }, vm::gainReleased, vm.gainsSynced && vm.tuningEnabled)
            }
        }
        // Flight modes, 2 x 2. Tapping one picks it and opens the stick screen.
        Screen.Options -> Column(Modifier.fillMaxSize(), verticalArrangement = Arrangement.spacedBy(16.dp)) {
            for (row in FlightMode.entries.chunked(2)) {
                Row(Modifier.weight(1f), horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                    for (m in row) {
                        ModeTile(
                            mode = m,
                            selected = vm.flightMode == m,
                            flying = vm.armed && flight.airborne,
                            modifier = Modifier.weight(1f).fillMaxHeight(),
                        ) { vm.selectMode(m) }
                    }
                }
            }
        }
        // Stick screen (and the instant before Splash hands over)
        else -> Joystick(
            Modifier.fillMaxSize(),
            holdY = true,                // throttle stays where you leave it
            initialX = vm.yaw,
            initialY = vm.throttle,
            labelX = "YAW",
            labelY = "THR",
            onChange = vm::setLeftStick,
            lockX = vm.flightMode.sticksLocked,
            lockY = vm.flightMode.throttleLocked,
            holdMark = vm.flightMode == FlightMode.LOITER,
        ) {
            ModeBadge(vm.flightMode, flight, connected)
            if (vm.flightMode.sticksLocked) LockedNote("AUTOPILOT")
        }
    }
}

@Composable
private fun RightPane(s: Screen, vm: HoverItViewModel, flight: FlightState, connected: Boolean) {
    when (s) {
        Screen.Tuning -> Column(Modifier.fillMaxSize(), verticalArrangement = Arrangement.spacedBy(16.dp)) {
            TuningToggle(vm.tuningEnabled, vm.gainsSynced, Modifier.fillMaxWidth().height(TuningHeader), vm::toggleTuning)
            Row(Modifier.weight(1f), horizontalArrangement = Arrangement.spacedBy(40.dp)) {
                GainSlider("KD", vm.kd, Modifier.weight(1f).fillMaxHeight(), { vm.setGain("kd", it) }, vm::gainReleased, vm.gainsSynced && vm.tuningEnabled)
                GainSlider("FF", vm.ff, Modifier.weight(1f).fillMaxHeight(), { vm.setGain("ff", it) }, vm::gainReleased, vm.gainsSynced && vm.tuningEnabled)
            }
        }
        Screen.Options -> Column(Modifier.fillMaxSize(), verticalArrangement = Arrangement.spacedBy(16.dp)) {
            OptionTile("Tuning Mode", Modifier.weight(1f), subLabel = "PID sliders") { vm.goTo(Screen.Tuning) }
            OptionTile(
                "Enable Baro",
                Modifier.weight(1f),
                selected = vm.training,
                subLabel = when {
                    vm.training && flight.training -> "ON \u00b7 baro in use"
                    vm.training -> "ON \u00b7 ToF below 50 cm"
                    flight.training -> "OFF \u00b7 ToF lost, baro in use"
                    else -> "OFF \u00b7 ToF only"
                },
                onClick = vm::toggleTraining,
            )
            OptionTile(
                "Theme",
                Modifier.weight(1f),
                subLabel = if (vm.blueTheme) "Blue" else "Red",
                onClick = vm::toggleTheme,
            )
        }
        else -> Joystick(
            Modifier.fillMaxSize(),
            holdY = false,
            initialX = vm.roll,
            initialY = vm.pitch,
            labelX = "ROL",
            labelY = "PIT",
            onChange = vm::setRightStick,
            lockX = vm.flightMode.sticksLocked,
            lockY = vm.flightMode.sticksLocked,
        ) {
            HeightBadge(flight, connected)
        }
    }
}
