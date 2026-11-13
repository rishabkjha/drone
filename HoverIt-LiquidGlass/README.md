# HoverIt Liquid

The HoverIt app with a **Liquid Glass** look, kept next to the original glassmorphism app in [`../HoverIt`](../HoverIt), which is unchanged. Same screens, same drone link (protocol v3), same colours; only the glass changes. It installs as a separate app ("HoverIt Liquid", id `com.hoverit.liquid`), so both can sit on the phone at once.

Open `HoverIt-LiquidGlass/HoverIt` in Android Studio and press Run, as for the original. Setup, screens and the link are described in [`../HoverIt/README.md`](../HoverIt/README.md).

## What is different from glassmorphism

| | Glassmorphism (`HoverIt/`) | Liquid Glass (`HoverIt-LiquidGlass/`) |
|---|---|---|
| Glass | frosted fill over the background | the real background seen through a lens: light blur, extra colour, and edges that bend it like the rim of a droplet |
| Edge | flat white border | thin specular rim plus a soft inner bevel; the light comes from the top left and swings as you tilt the phone |
| Press | button shrinks | button swells on a soft spring and glows under your finger |
| Stick / slider thumbs | flat gradient | coral-to-magenta (or blue) glass droplets that swell and brighten while held |
| Shapes | 22 dp corners | rounder, 28 dp corners |
| Screen changes | fixed-time curves | springs that settle with a little overshoot |

## How it works
- `ui/LiquidGlass.kt`: `LiquidGlassRoot` records the background into a `GraphicsLayer`; `Modifier.glass()` redraws that layer behind each panel, shifted to the panel's position, through the lens, then draws the tint, finger glow, rim and bevel.
- `ui/LiquidEffect.kt`: the lens. Android 13+ uses a blur followed by an AGSL shader (rounded-rectangle distance field, inward pull near the rim, slight colour split at the edge, saturation lift). Android 12 gets the blur only. Android 8 to 11 fall back to the frosted fill.
- `ui/Tilt.kt`: one roll sensor (`rememberPhoneRoll`) now feeds both the battery water and the glass light.
