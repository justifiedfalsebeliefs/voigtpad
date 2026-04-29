Consider the existing implementation and UX. Take docs/gas_chords.dsp faust patch, compile to C, and implement in hardware, replacing the dual delay module, but copying it's best practices. Obviously we no longer need midi, and should instead use eurorack conventions. It should always drone. Requirements:

Static params (not exposed to user):
- Stereo spread and stereo width maxed
- Sub pulse rate: .37 hz Sub Depth: .6
- Fog Mod Depth: .4, rate .06

Params exposed as pots:
Defaults (always set to these on initialization), max, min:
Page1 Pot 1: root pitch. 
Page1 Pot 2: Level for Shimmer (default 50% on all level pots)
Page1 Pot 3: detune (default 15 cents, max 30 cents, min 0 cents)
Page1 Pot 4: Level, main
Page1 Pot 5: Chord, main (default min, options are min;maj;min7;sus2;sus4)
Page1 Pot 6: Level, sub osc
Page2 Pot 1: Shimmer drift rate (default 0.15. Max and min as per patch)
Page2 Pot 2: Shimmer octave. Select between 0-4, default 2.
Page2 Pot 3: Shimmer air. Default .5. from 0-1
Page2 Pot 4: Fog cutoff. Default 4450hz. 200-8000 range.
Page2 Pot 5: Sub warmth. Default .2. 0-1.

Top button: change page as is with the delay.
Mid button: Hold for deferred apply of any param (apply on release). Show param moving on bar with breathing animation when deferred.
Buttom button: reserved for later

Use the existing pot catch implementation.

For the params that have discrete options (Chord, Shimmer octave), split their lights in the pot to discrete color bands, and slightly brighten the selected color band. The idea being I can hold mid button, find a band, and relase without have to sweep through all of them. 

Considerably lower the output gain so as to not hit clipping unless the three mix params for shimmer/main/sub are at 75% or higher.

Your solution must be performant and properly use ISR, DMA, etc.
