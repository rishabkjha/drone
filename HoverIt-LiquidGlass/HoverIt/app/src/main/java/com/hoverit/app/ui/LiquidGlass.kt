package com.hoverit.app.ui

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.State
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawWithCache
import androidx.compose.ui.draw.drawWithContent
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Outline
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.graphics.addOutline
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.clipPath
import androidx.compose.ui.graphics.drawscope.translate
import androidx.compose.ui.graphics.layer.GraphicsLayer
import androidx.compose.ui.graphics.layer.drawLayer
import androidx.compose.ui.graphics.rememberGraphicsLayer
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.layout.positionInRoot
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

/*
 * Liquid Glass
 * ------------
 * Every glass surface shows the backdrop behind it through a lens: a light
 * blur, a little extra colour, and edges that bend the background like the
 * rim of a glass droplet (LiquidEffect.kt). On top sit a thin specular rim
 * that catches the light, a soft inner bevel, and a glow under your finger.
 * The light comes from the top left and swings as you tilt the phone.
 *
 * How the backdrop gets in: LiquidGlassRoot records the background into a
 * GraphicsLayer once per frame; each glass element redraws that layer,
 * shifted to its own position, through its lens effect.
 */

/** The recorded background, shared by every glass surface. */
val LocalBackdrop = staticCompositionLocalOf<GraphicsLayer?> { null }

/** Phone roll in radians (smoothed): tilts the light on every glass rim. */
val LocalPhoneRoll = staticCompositionLocalOf<State<Float>> { mutableFloatStateOf(0f) }

/**
 * Draws [backdrop] full screen, records it for the glass, and lays [content]
 * over it.
 */
@Composable
fun LiquidGlassRoot(backdrop: @Composable () -> Unit, content: @Composable () -> Unit) {
    val layer = rememberGraphicsLayer()
    val roll = rememberPhoneRoll()
    Box(Modifier.fillMaxSize()) {
        Box(
            Modifier
                .fillMaxSize()
                .drawWithContent {
                    layer.record { this@drawWithContent.drawContent() }
                    drawLayer(layer)
                },
        ) { backdrop() }
        CompositionLocalProvider(LocalBackdrop provides layer, LocalPhoneRoll provides roll) {
            content()
        }
    }
}

private class GlassPosition {
    var offset by mutableStateOf(Offset.Zero)
}

/**
 * Liquid glass surface.
 * [tint] colours the glass (green / red state buttons, the picked tile);
 * [tintBrush] does the same with a gradient (joystick thumbs).
 * [press] 0..1 lights the glass from [touch] (a point inside it, px).
 * [lift] 0..1 makes the rim and bevel brighter (an active thumb).
 */
@Composable
fun Modifier.glass(
    shape: Shape = GlassShape,
    tint: Color? = null,
    borderWidth: Dp = 1.2.dp,
    tintBrush: Brush? = null,
    press: Float = 0f,
    touch: Offset? = null,
    lift: Float = 0f,
): Modifier {
    val p = LocalGlass.current
    val backdrop = LocalBackdrop.current
    val roll = LocalPhoneRoll.current
    val pos = remember { GlassPosition() }
    val lensOn = backdrop != null && liquidLensSupported()

    return this
        .onGloballyPositioned { pos.offset = it.positionInRoot() }
        .clip(shape)   // content (water, fills) stays inside the glass
        .drawWithCache {
            val outline = shape.createOutline(size, layoutDirection, this)
            val path = Path().apply { addOutline(outline) }
            val radius = (outline as? Outline.Rounded)?.roundRect?.topLeftCornerRadius?.x ?: 0f
            val effect = liquidRenderEffect(size.width, size.height, radius, density)
            val lensLayer = if (backdrop != null) obtainGraphicsLayer().apply { renderEffect = effect } else null
            val rim = borderWidth.toPx()
            val minSide = minOf(size.width, size.height)

            onDrawWithContent {
                // 1. the backdrop, seen through the lens
                if (lensLayer != null && backdrop != null) {
                    lensLayer.record {
                        val o = pos.offset
                        translate(-o.x, -o.y) { drawLayer(backdrop) }
                    }
                    clipPath(path) { drawLayer(lensLayer) }
                }

                // 2. the glass body: almost clear where the lens works, frostier where it can't
                val bodyAlpha = if (lensOn) 0.55f else 1f
                when {
                    tintBrush != null -> drawPath(path, tintBrush, alpha = 0.78f)
                    tint != null -> drawPath(
                        path,
                        Brush.verticalGradient(listOf(tint.copy(alpha = tint.alpha * 0.44f), tint.copy(alpha = tint.alpha * 0.22f))),
                    )
                    else -> drawPath(
                        path,
                        Brush.verticalGradient(listOf(p.glassTop, p.glassBottom)),
                        alpha = bodyAlpha,
                    )
                }

                // 3. glow under the finger
                if (press > 0.001f && touch != null) {
                    clipPath(path) {
                        drawCircle(
                            Brush.radialGradient(
                                listOf(Color.White.copy(alpha = 0.22f * press), Color.Transparent),
                                center = touch,
                                radius = max(size.width, size.height) * 0.75f,
                            ),
                            radius = max(size.width, size.height),
                            center = touch,
                        )
                    }
                }

                drawContent()

                // 4. light: from the top left, swung by the phone's roll
                val a = (-2.2f + roll.value * 0.9f)
                val dir = Offset(cos(a), sin(a))
                val c = center
                val reach = max(size.width, size.height) * 0.6f
                val from = c + dir * reach
                val to = c - dir * reach
                val glow = 1f + lift * 0.6f
                val spec = Brush.linearGradient(
                    0f to Color.White.copy(alpha = (0.95f * glow).coerceAtMost(1f)),
                    0.30f to Color.White.copy(alpha = 0.18f),
                    0.55f to Color.White.copy(alpha = 0.06f),
                    0.80f to Color.White.copy(alpha = 0.22f),
                    1f to Color.White.copy(alpha = (0.60f * glow).coerceAtMost(1f)),
                    start = from,
                    end = to,
                )
                // soft inner bevel: a few wide, faint strokes inside the edge
                clipPath(path) {
                    val bevel = (minSide * 0.08f).coerceIn(4.dp.toPx(), 14.dp.toPx())
                    for (i in 1..6) {
                        val f = i / 6f
                        drawPath(path, spec, alpha = 0.022f * glow, style = Stroke(bevel * 2f * f))
                    }
                    // the thin specular rim itself (inside half of a 2x stroke)
                    drawPath(path, spec, style = Stroke(rim * 2f))
                }
            }
        }
}
