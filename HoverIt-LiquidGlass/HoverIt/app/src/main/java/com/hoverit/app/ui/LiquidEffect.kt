package com.hoverit.app.ui

import android.graphics.RuntimeShader
import android.graphics.Shader
import android.os.Build
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.RenderEffect
import androidx.compose.ui.graphics.asComposeRenderEffect

/**
 * The lens behind every liquid glass surface (Android side of LiquidGlass.kt).
 *
 *  - Android 13+: one AGSL shader does the whole optic: a thick convex rim that
 *    bends and squeezes the backdrop (like the edge of a water droplet), a slight
 *    zoom through the middle, colour split at the rim, and the light itself: a
 *    thin bright specular line, brightest on the side facing the light with a
 *    softer echo on the opposite corner, a soft sheen and a thin inner shadow.
 *  - Android 12: a light blur only; the rim light is drawn by Compose.
 *  - Older: no lens; the glass falls back to a frosted fill.
 */
fun liquidLensSupported(): Boolean = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S

/** True when the shader also draws the rim light (so Compose must not). */
fun liquidShaderSupported(): Boolean = Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU

/** Blur radius in dp. Tiny on purpose: Liquid Glass is clear, not frosted. */
private const val BLUR_DP = 1.2f
/** Width (dp) of the curved rim, at most 40 % of the shorter side. */
private const val BEZEL_DP = 26f
/** How far (dp) the very edge pulls the backdrop in. */
private const val DEPTH_DP = 40f
/** Centre zoom: < 1 samples closer to the middle, i.e. magnifies. */
private const val ZOOM = 0.94f

/**
 * One per glass element. Holds the shader so only the uniforms change from
 * frame to frame (the light moves with the phone).
 */
class LiquidLens {
    private val shader: Any? =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) RuntimeShader(LENS_SHADER) else null

    /**
     * [lightX], [lightY]: unit vector pointing towards the light, screen space (y down).
     * [shine]: rim brightness, 1 = normal, more for an active or pressed element.
     */
    fun effect(
        width: Float, height: Float, radius: Float, density: Float,
        lightX: Float, lightY: Float, shine: Float,
        tint: Color = Color.Transparent,
    ): RenderEffect? {
        if (width < 1f || height < 1f) return null
        val sdk = Build.VERSION.SDK_INT
        if (sdk < Build.VERSION_CODES.S) return null
        val blurPx = BLUR_DP * density
        val blur = android.graphics.RenderEffect.createBlurEffect(blurPx, blurPx, Shader.TileMode.CLAMP)
        if (sdk < Build.VERSION_CODES.TIRAMISU || shader == null) return blur.asComposeRenderEffect()

        val minSide = minOf(width, height)
        val bezel = minOf(BEZEL_DP * density, minSide * 0.4f)
        val rs = shader as RuntimeShader
        rs.setFloatUniform("size", width, height)
        rs.setFloatUniform("radius", minOf(radius, minSide / 2f))
        rs.setFloatUniform("bezel", bezel)
        rs.setFloatUniform("depth", minOf(DEPTH_DP * density, bezel * 1.3f))
        rs.setFloatUniform("zoom", ZOOM)
        rs.setFloatUniform("light", lightX, lightY)
        rs.setFloatUniform("shine", shine)
        rs.setFloatUniform("px", density)
        rs.setFloatUniform("tint", tint.red, tint.green, tint.blue, tint.alpha)
        val lens = android.graphics.RenderEffect.createRuntimeShaderEffect(rs, "content")
        // blur first (barely), then the lens bends the backdrop and lights the rim
        return android.graphics.RenderEffect.createChainEffect(lens, blur).asComposeRenderEffect()
    }
}

/** AGSL. coord is in the glass element's own pixels. */
internal const val LENS_SHADER = """
uniform shader content;
uniform float2 size;
uniform float radius;
uniform float bezel;
uniform float depth;
uniform float zoom;
uniform float2 light;
uniform float shine;
uniform float px;
uniform float4 tint;   // rgb + strength: colours the light passing through, never hides it

// signed distance to a rounded rectangle centred on 0 (negative inside)
float sdBox(float2 p, float2 b, float r) {
    float2 q = abs(p) - b + r;
    return min(max(q.x, q.y), 0.0) + length(max(q, 0.0)) - r;
}

half4 main(float2 coord) {
    float2 hs = size * 0.5;
    float2 p = coord - hs;
    float d = sdBox(p, hs, radius);
    float inset = max(-d, 0.0);                        // px in from the rim
    float s = clamp(1.0 - inset / bezel, 0.0, 1.0);    // 1 at the rim, 0 past the bezel

    // outward normal from the distance field
    float e = 1.0;
    float2 n = float2(
        sdBox(p + float2(e, 0.0), hs, radius) - sdBox(p - float2(e, 0.0), hs, radius),
        sdBox(p + float2(0.0, e), hs, radius) - sdBox(p - float2(0.0, e), hs, radius));
    float nl = length(n);
    n = nl > 0.0001 ? n / nl : float2(0.0);

    // circular bevel: flat in the middle, bending harder and harder towards the rim
    float circ = 1.0 - sqrt(max(1.0 - s * s, 0.0));
    float bend = 0.45 * s * s + 0.55 * circ;
    float2 base = hs + p * zoom;
    float2 off = n * depth * bend;

    // colour splits where the glass bends most
    float spread = 0.14 * bend;
    half4 c;
    c.r = content.eval(base - off * (1.0 + spread)).r;
    c.g = content.eval(base - off).g;
    c.b = content.eval(base - off * (1.0 - spread)).b;
    c.a = 1.0;

    // light through glass: a touch more colour and brightness, more at the curved rim
    half lum = dot(c.rgb, half3(0.2126, 0.7152, 0.0722));
    c.rgb = mix(half3(lum), c.rgb, 1.25);
    c.rgb = c.rgb * 1.05 + half3(0.018 + 0.10 * bend * bend);

    // tinted glass: the colour multiplies the light coming through, plus a little glow
    half3 tc = half3(tint.rgb);
    c.rgb = mix(c.rgb, c.rgb * tc * 1.6 + tc * 0.22, tint.a);

    // the light: facing side bright, opposite corner a softer echo
    float l = dot(n, light);
    float lit = pow(max(l, 0.0), 2.0) + 0.55 * pow(max(-l, 0.0), 2.0);
    float rim = exp(-inset / (1.0 * px));                             // crisp hairline at the edge
    float sheen = exp(-inset / (10.0 * px)) * pow(max(l, 0.0), 3.0);  // wide soft glow inside it
    // light focused through the body lands as a bright crescent inside the far edge
    float ck = (inset - 7.0 * px) / (4.5 * px);
    float caustic = exp(-ck * ck) * pow(max(-l, 0.0), 2.0);
    c.rgb += half3(rim * (0.35 + 1.6 * lit) * shine + sheen * 0.24 * shine + caustic * 0.16 * shine);

    // thin shadow just inside the far edge gives the glass thickness
    float sd = (inset - 3.5 * px) / (2.5 * px);
    c.rgb *= 1.0 - 0.22 * exp(-sd * sd) * max(-l, 0.0);

    return clamp(c, 0.0, 1.0);
}
"""
