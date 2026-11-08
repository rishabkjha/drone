package com.hoverit.app.ui

import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.tween
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.Immutable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.Font
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import com.hoverit.app.FlightMode
import com.hoverit.app.R

object HoverColors {
    val GreenLight = Color(0xFF3BE08A)
    val GreenDark = Color(0xFF0B8F52)
    val RedLight = Color(0xFFFF5468)
    val RedDark = Color(0xFFB0102C)
    val WaterLight = Color(0xFF6FE3FF)
    val WaterDark = Color(0xFF1E7BFF)

    /** Light end of the accent gradient: coral (red theme) or sky blue (blue theme). */
    val Coral: Color @Composable get() = LocalGlass.current.accentLight
    /** Deep end of the accent gradient: magenta (red theme) or royal blue (blue theme). */
    val Magenta: Color @Composable get() = LocalGlass.current.accentDeep
    /** Thumbs, logo: light -> deep accent. */
    val Accent: Brush @Composable get() = Brush.linearGradient(listOf(Coral, Magenta))
}

/** Flight mode accent: the theme's deep accent for every mode. */
@Suppress("UNUSED_PARAMETER")
@Composable
fun modeColor(m: FlightMode): Color = HoverColors.Magenta

val LogoFont = FontFamily(Font(R.font.abril_fatface))

/** Colours that change with the red / blue theme. */
@Immutable
data class GlassPalette(
    val text: Color,
    val textDim: Color,
    val glassTop: Color,
    val glassBottom: Color,
    val borderTop: Color,
    val borderBottom: Color,
    val gridLine: Color,
    val accentLight: Color,
    val accentDeep: Color,
    val blue: Float,          // 0 = red background image, 1 = blue-tinted version
)

/** The original HoverIt look: coral -> magenta. */
private val RedPalette = GlassPalette(
    text = Color(0xFFFFFFFF),
    textDim = Color(0xB3FFFFFF),
    glassTop = Color(0x29FFFFFF),
    glassBottom = Color(0x0AFFFFFF),
    borderTop = Color(0xCCFFFFFF),
    borderBottom = Color(0x40FFFFFF),
    gridLine = Color(0x8CFFFFFF),
    accentLight = Color(0xFFFF5A5F),
    accentDeep = Color(0xFFA8005A),
    blue = 0f,
)

/** Same glass, sky -> royal blue accent, background tinted blue. */
private val BluePalette = RedPalette.copy(
    accentLight = Color(0xFF4FC3FF),
    accentDeep = Color(0xFF1E3FD0),
    blue = 1f,
)

val LocalGlass = staticCompositionLocalOf { RedPalette }

val LabelStyle = TextStyle(
    fontFamily = FontFamily.SansSerif,
    fontWeight = FontWeight.SemiBold,
    fontSize = 17.sp,
    letterSpacing = 1.sp,
)

/** Numbers and small readouts (stick values, height, gains). */
val ReadoutStyle = TextStyle(
    fontFamily = FontFamily.Monospace,
    fontWeight = FontWeight.Medium,
    fontSize = 12.sp,
    letterSpacing = 0.5.sp,
)

@Composable
fun HoverItTheme(blue: Boolean, content: @Composable () -> Unit) {
    val target = if (blue) BluePalette else RedPalette
    val spec = tween<Color>(600)
    val light by animateColorAsState(target.accentLight, spec, label = "accentLight")
    val deep by animateColorAsState(target.accentDeep, spec, label = "accentDeep")
    val mix by animateFloatAsState(target.blue, tween(600), label = "blueMix")
    CompositionLocalProvider(
        LocalGlass provides target.copy(accentLight = light, accentDeep = deep, blue = mix),
        content = content,
    )
}
