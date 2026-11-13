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
import androidx.compose.ui.draw.shadow
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

    val shaderLight = lensOn && liquidShaderSupported()
    val lens = remember { LiquidLens() }

    return this
        .onGloballyPositioned { pos.offset = it.positionInRoot() }
        // floats above the backdrop: soft, wide, dark shadow (only outside the shape shows)
        .shadow(if (lensOn) 14.dp else 0.dp, shape, clip = false,
            ambientColor = Color.Black.copy(alpha = 0.55f), spotColor = Color.Black.copy(alpha = 0.7f))
        .clip(shape)   // content (water, fills) stays inside the glass
        .drawWithCache {
            val outline = shape.createOutline(size, layoutDirection, this)
            val path = Path().apply { addOutline(outline) }
            val radius = (outline as? Outline.Rounded)?.roundRect?.topLeftCornerRadius?.x ?: 0f
            val lensLayer = if (backdrop != null) obtainGraphicsLayer() else null
            val rim = borderWidth.toPx()
            val minSide = minOf(size.width, size.height)

            onDrawWithContent {
                // with the shader, a solid tint goes into the lens itself (still see-through)
                val shaderTint = if (shaderLight && tint != null) tint.copy(alpha = (tint.alpha * 0.85f).coerceAtMost(1f)) else Color.Transparent
                // light from the top left, swung by the phone's roll
                val a = (-2.2f + roll.value * 0.9f)
                val dir = Offset(cos(a), sin(a))
                val shine = 1f + lift * 0.7f + press * 0.5f

                // 1. the backdrop, seen through the lens (the shader also lights the rim)
                if (lensLayer != null && backdrop != null) {
                    lensLayer.renderEffect = lens.effect(size.width, size.height, radius, density, dir.x, dir.y, shine, shaderTint)
                    lensLayer.record {
                        val o = pos.offset
                        translate(-o.x, -o.y) { drawLayer(backdrop) }
                    }
                    clipPath(path) { drawLayer(lensLayer) }
                }

                // 2. the glass body: clear where the lens works, frosted where it can't
                when {
                    tintBrush != null -> drawPath(path, tintBrush, alpha = if (lensOn) 0.30f else 0.78f)
                    tint != null && shaderLight -> drawPath(
                        path,
                        Brush.verticalGradient(listOf(tint.copy(alpha = tint.alpha * 0.10f), Color.Transparent)),
                    )
                    tint != null -> drawPath(
                        path,
                        Brush.verticalGradient(listOf(tint.copy(alpha = tint.alpha * 0.40f), tint.copy(alpha = tint.alpha * 0.26f))),
                    )
                    else -> drawPath(
                        path,
                        Brush.verticalGradient(listOf(p.glassTop, p.glassBottom)),
                        alpha = if (lensOn) 0.12f else 1f,
                    )
                }

                // 3. glow under the finger
                if (press > 0.001f && touch != null) {
                    clipPath(path) {
                        drawCircle(
                            Brush.radialGradient(
                                listOf(Color.White.copy(alpha = 0.20f * press), Color.Transparent),
                                center = touch,
                                radius = max(size.width, size.height) * 0.6f,
                            ),
                            radius = max(size.width, size.height),
                            center = touch,
                        )
                    }
                }

                drawContent()

                // 4. rim light drawn here only where the shader can't (Android 12 and older),
                //    plus a faint hairline everywhere so the edge stays crisp over tints
                val c = center
                val reach = max(size.width, size.height) * 0.6f
                val glow = shine
                val spec = Brush.linearGradient(
                    0f to Color.White.copy(alpha = (0.95f * glow).coerceAtMost(1f)),
                    0.30f to Color.White.copy(alpha = 0.18f),
                    0.55f to Color.White.copy(alpha = 0.06f),
                    0.80f to Color.White.copy(alpha = 0.22f),
                    1f to Color.White.copy(alpha = (0.60f * glow).coerceAtMost(1f)),
                    start = c + dir * reach,
                    end = c - dir * reach,
                )
                clipPath(path) {
                    if (shaderLight) {
                        if (tint != null || tintBrush != null) drawPath(path, spec, alpha = 0.55f, style = Stroke(rim * 2f))
                    } else {
                        val bevel = (minSide * 0.08f).coerceIn(4.dp.toPx(), 14.dp.toPx())
                        for (i in 1..6) {
                            val f = i / 6f
                            drawPath(path, spec, alpha = 0.022f * glow, style = Stroke(bevel * 2f * f))
                        }
                        drawPath(path, spec, style = Stroke(rim * 2f))
                    }
                }
            }
        }
}
