package com.hoverit.app.ui

import android.graphics.RuntimeShader
import android.graphics.Shader
import android.os.Build
import androidx.compose.ui.graphics.RenderEffect
import androidx.compose.ui.graphics.asComposeRenderEffect

/**
 * The lens behind every liquid glass surface (Android side of LiquidGlass.kt).
 *  - Android 13+: light blur, then a shader that bends the background near the
 *    edges like the rim of a droplet, splits colour a touch at the very edge
 *    and lifts saturation.
 *  - Android 12: blur only.
 *  - Older: no lens; the glass falls back to a frosted fill.
 */
fun liquidLensSupported(): Boolean = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S

/** Blur radius in dp. Small on purpose: Liquid Glass stays clear, not frosted. */
private const val BLUR_DP = 3.5f
/** How far (dp) the very edge pulls the background inwards. */
private const val BEND_DP = 22f
/** Width (dp) of the bending band along the edge, at most a third of the shorter side. */
private const val BAND_DP = 30f

fun liquidRenderEffect(width: Float, height: Float, radius: Float, density: Float): RenderEffect? {
    if (width < 1f || height < 1f) return null
    val sdk = Build.VERSION.SDK_INT
    if (sdk < Build.VERSION_CODES.S) return null
    val blurPx = BLUR_DP * density
    val blur = android.graphics.RenderEffect.createBlurEffect(blurPx, blurPx, Shader.TileMode.CLAMP)
    if (sdk < Build.VERSION_CODES.TIRAMISU) return blur.asComposeRenderEffect()

    val band = minOf(BAND_DP * density, minOf(width, height) / 3f)
    val shader = RuntimeShader(LENS_SHADER).apply {
        setFloatUniform("size", width, height)
        setFloatUniform("radius", minOf(radius, minOf(width, height) / 2f))
        setFloatUniform("bend", minOf(BEND_DP * density, band * 0.9f))
        setFloatUniform("band", band)
    }
    val lens = android.graphics.RenderEffect.createRuntimeShaderEffect(shader, "content")
    // blur first, then the lens bends the blurred backdrop
    return android.graphics.RenderEffect.createChainEffect(lens, blur).asComposeRenderEffect()
}

/** AGSL. coord is in the glass element's own pixels. */
internal const val LENS_SHADER = """
uniform shader content;
uniform float2 size;
uniform float radius;
uniform float bend;
uniform float band;

// signed distance to a rounded rectangle centred on 0 (negative inside)
float sdBox(float2 p, float2 b, float r) {
    float2 q = abs(p) - b + r;
    return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - r;
}

half4 main(float2 coord) {
    float2 hs = size * 0.5;
    float2 p = coord - hs;
    float d = sdBox(p, hs, radius);

    // 0 in the middle, 1 at the rim; curved so the bend builds up near the edge
    float t = clamp(1.0 + d / band, 0.0, 1.0);
    float k = t * t * t;

    // outward normal from the distance field
    float e = 1.0;
    float2 n = float2(
        sdBox(p + float2(e, 0.0), hs, radius) - sdBox(p - float2(e, 0.0), hs, radius),
        sdBox(p + float2(0.0, e), hs, radius) - sdBox(p - float2(0.0, e), hs, radius));
    float nl = length(n);
    n = nl > 0.0001 ? n / nl : float2(0.0);

    // pull the sample inwards: the rim magnifies and bends what is behind it
    float2 off = n * bend * k;
    half4 c;
    c.r = content.eval(coord - off * 1.08).r;
    c.g = content.eval(coord - off).g;
    c.b = content.eval(coord - off * 0.92).b;
    c.a = content.eval(coord - off).a;

    // a little more colour, like light through glass
    half l = dot(c.rgb, half3(0.2126, 0.7152, 0.0722));
    c.rgb = mix(half3(l), c.rgb, 1.35);
    c.rgb += half3(0.025 * k);
    return clamp(c, 0.0, 1.0);
}
"""
