Concept

A dual delay/reverb module for eurorack, built on the Daisy Seed2DFM, that treats delay and reverb as a continuous parameter space rather than separate effects. By reframing reverb as "what happens when echo density and diffusion become high enough that individual reflections smear into a wash," the module lets users navigate freely between clean delays, slapbacks, early reflections, plates, halls, and infinite ambience — all on the same set of controls.Two independent delay lines, each with twenty-four parameters across four pages, plus shared modes that determine how the lines interact with the stereo signal path.

The 6 pots are organized as two stacks of three, one stack per delay line. Pages cycle the meaning of those pots, using elegant pot-catch behavior with LED-ring visual feedback to handle the parameter changes.

Each delay line is internally a multi-tap delay buffer feeding an all-pass diffusion network with a damped, modulated feedback path. This architecture is always running; the controls just push it toward different regions of behavior:

* Long time + low diffusion + few taps → clean repeating delay
* Medium time + medium diffusion → slapback and early reflections
* Short time + high diffusion + high feedback → reverb
* Very short time + max diffusion + max feedback → infinite ambient wash

There is no "reverb mode" — reverb is just a region of the parameter space.

# The Four Pages

## Page 1 — Time Domain Essentials

* Delay time
* Feedback
* Wet/dry mix

## Page 2 — Space

* Diffusion (sparse echoes ↔ smooth wash, via the all-pass network)
* Damping (frequency-dependent decay, models air and surface absorption)
* Bloom (size & motion of the diffuser network — short fixed all-pass loops at 0 for clean delay, slowly-modulated long loops at 1 for plate/hall tails)

## Page 3 — Character

* Engine morph: digi → BBD → tape → vinyl, a continuous coloration sweep
* Engine color depth (intensity of the chosen coloration; at 0, perfectly clean digital regardless of morph position)
* Cross-feedback with the other delay line

## Page 4 — Motion

* Mod depth
* Mod rate (slow drift to audio-rate)
* Mod shape (continuous morph from sine through triangle, square, ramp, S&H, smoothed noise, tape character, chaos)

Each engine carries its own internal modulation signature (tape flutter, vinyl wow, BBD clock noise) that's separate from the global mod page — preserving each engine's identity while letting users layer additional motion on top.

# Modes (Settings Page 1)

Selectable global routing topologies for how the two delay lines relate to the stereo signal:

* Series stereo (V1 default) — delay 1 feeds delay 2 sequentially
* Parallel stereo — input copies to both, mixed at output
* Isolated mono — L and R each get their own line, output separately
* Spectral stereo — parallel, but split by crossover frequency (lows to delay 1, highs to delay 2)

V1 ships with series stereo only; other modes follow.

# Buttons & Interaction

* Page button — cycles 1 → 2 → 3 → 4 → 1
* Tap 1 — tap tempo for delay line 1; with shift = freeze delay 1
* Tap 2 — tap tempo for delay line 2; with shift = freeze delay 2
* Long-press both tap buttons (2 seconds) — enters Settings mode; exit via either freeze button

In Settings mode, the six pots become configuration controls across three setting pages: global options (modes, clock priority, spectral crossover), per-line options (output gain, fixed low-cut, stereo width), and presets.
LED Behavior

Each page has a unique color profile, with subtle animation around the focused parameter

**Pot-catch indicator:** when the physical pot position doesn't match the stored parameter value (i.e., after a page change), a white pip appears on the LED ring at the stored parameter's position. Once the pot is rotated through that position and "catches," the pip disappears and the pot is live. No pip = active; pip visible = catch first.

## What Makes It Distinctive

* Reverb-as-region, not as mode. The DSP architecture treats delay and reverb as one continuum, so users can find sounds that don't have names yet between the two regions.
* Continuous engine morph. Four lo-fi colorations on a single knob, with a depth control that scales the whole intensity. Easy to dial in, vast sonic territory.
* Bloom as a first-class continuous parameter — morphs the diffuser between V1's short fixed all-pass loops (clean delay) and slowly-modulated long loops (plate/hall tails), giving size and motion a single-knob control anywhere on the delay/reverb continuum.
* Cross-feedback between lines as a panel control, not a hidden setting — encourages dual-line patches that are genuinely interactive rather than just two delays running in parallel.
* Crackle as a writeable signal. The vinyl engine's crackle goes into the delay buffer itself, meaning it gets diffused, damped, and modulated like any other signal — opening up textural possibilities that purely-output-stage crackle can't reach.
* Motion that can go to audio rate. Modulation isn't just LFO chorus territory — it extends into FM/sideband generation for inharmonic, metallic textures.
* Pot-catch with ring-LED feedback. Multi-page control surfaces usually feel clumsy; the white-pip catch indicator makes parameter targeting precise and physical.


---

BUTTONS
Page
tap tempo delay 1 (+shift = freeze delay 1)
tap tempo delay 2 (+shift = freeze delay 2)

Enter settings: long press both tap temp buttons for 2 seconds. Exit by pressing either freeze button. Each pot becomes an option. Options:

Page 1: General options
- spectral mode crossover freq (1 pot)
- Freeze behavior
- Tap time vs input clock rate quantization etc priorities
- Delay mode: (1 pot)
(mode 1 - series stereo) Delay 1 feeds to delay 2 sequentially
(mode 2 - parallel stereo) input is copied to delay 1 and delay two simultaneously and mixed together at output
(mode 3 - isolated mono) audio L and audio R each independently go to delay 1 and delay 2 and are output separately as 1 and 2. 
(mode 4 - spectral stereo) Parallel, but delay one gets low freq, delay two gets high

Page 2: per-line options. Each option below is x2, one for each delay line.
- Output gain
- fixed low cut damping amount for each delay line
- stereo width (applied when in stereo modes)

Page 3:
- Presets (UX TBD)


