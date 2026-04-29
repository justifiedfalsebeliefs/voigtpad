declare name        "GAS Ethereal Textures";
declare author      "faust-patches";
declare version     "1.0";
declare license     "MIT";
declare description "Evolving polyphonic ambient texture generator inspired by
                     Wolfgang Voigt (GAS). Four sound-design columns:
                     1. Chord (polyphonic chord generator: the harmonic root)
                     2. Sub Drone (deep, slowly pulsing low end)
                     3. Shimmer (drifting bell partials + filtered air)
                     4. Space (slow LFO low-pass fog + master)";

import("stdfaust.lib");

//=================================================================
// Utility helpers
//=================================================================
midi2hz(n) = ba.midikey2hz(n);

pan(p) = _ <: *(sqrt(1.0 - p)), *(sqrt(p));

superSaw(note, detCents) =
    ( os.sawtooth(midi2hz(note + detCents/100.0))
    + os.sawtooth(midi2hz(note))
    + os.sawtooth(midi2hz(note - detCents/100.0))) / 3.0;

slowLFO(rate) = 0.5*os.osc(rate) + 0.5*os.osc(rate*1.37);

xfade(m) = _, _ : *(1.0 - m), *(m) :> _;

//=================================================================
// COLUMN 1 — CHORD
//=================================================================
root    = hslider("h:[1] Chord/[1] Root (MIDI)",       36, 24, 60, 1) : si.smoo;
ctype   = nentry ("h:[1] Chord/[2] Chord {min;maj;min7;maj7;sus2;sus4;dim;aug}",
                  1, 0, 7, 1);
detune  = hslider("h:[1] Chord/[3] Detune (cents)",    8, 0, 30, 0.1) : si.smoo;
spread  = hslider("h:[1] Chord/[4] Stereo Spread",     0.8, 0, 1, 0.01);
chordLv = hslider("h:[1] Chord/[5] Level",             0.7, 0, 1, 0.01) : si.smoo;

iv3 = ba.selectn(8, ctype,      3,  4,  3,  4,  2,   5,   3,  4);
iv5 = ba.selectn(8, ctype,      7,  7,  7,  7,  7,   7,   6,  8);
iv7 = ba.selectn(8, ctype,     12, 12, 10, 11, 12,  12,  12, 12);

chordVoice(n, d) = superSaw(n, d) : fi.lowpass(2, 2200);

chordSection =
    ( (chordVoice(root,       detune)      : pan(0.5 - 0.45*spread))
    , (chordVoice(root + iv3, detune*1.13) : pan(0.5 - 0.18*spread))
    , (chordVoice(root + iv5, detune*0.91) : pan(0.5 + 0.18*spread))
    , (chordVoice(root + iv7, detune*1.21) : pan(0.5 + 0.45*spread))
    ) :> _,_ : *(chordLv), *(chordLv);

//=================================================================
// COLUMN 2 — SUB DRONE
//=================================================================
subOct    = nentry ("h:[2] Sub Drone/[1] Octave Shift",      -1, -3, 0, 1);
subPulse  = hslider("h:[2] Sub Drone/[2] Pulse Rate (Hz)",   0.07, 0.01, 1.0, 0.001) : si.smoo;
subDepth  = hslider("h:[2] Sub Drone/[3] Pulse Depth",       0.6, 0, 1, 0.01) : si.smoo;
subWarm   = hslider("h:[2] Sub Drone/[4] Warmth",            0.3, 0, 1, 0.01) : si.smoo;
subLevel  = hslider("h:[2] Sub Drone/[5] Level",             0.5, 0, 1, 0.01) : si.smoo;

subNote   = root + 12*subOct;
subFund   = midi2hz(subNote);
subTone   = os.osc(subFund)
          + 0.40*os.osc(subFund * 2.0)
          + 0.15*os.osc(subFund * 3.0);
subSat    = subTone * (1.0 + 4.0*subWarm) : ma.tanh : /(1.0 + 2.0*subWarm);
subEnv    = (1.0 - subDepth) + subDepth * (0.5 + 0.5*os.osc(subPulse));

subSection = subSat * subEnv * subLevel <: _,_;

//=================================================================
// COLUMN 3 — SHIMMER
//=================================================================
shmrLevel = hslider("h:[3] Shimmer/[1] Level",               0.35, 0, 1, 0.01) : si.smoo;
shmrOct   = nentry ("h:[3] Shimmer/[2] Octave Shift",        2, 0, 4, 1);
shmrDrift = hslider("h:[3] Shimmer/[3] Drift Rate (Hz)",     0.11, 0.01, 1.0, 0.001) : si.smoo;
shmrAir   = hslider("h:[3] Shimmer/[4] Air (noise)",         0.25, 0, 1, 0.01) : si.smoo;
shmrWidth = hslider("h:[3] Shimmer/[5] Stereo Width",        0.9, 0, 1, 0.01);

shmrBase  = root + 12*shmrOct;

bellL = 0.5 * (
        os.osc(midi2hz(shmrBase)       * (1.0 + 0.001*slowLFO(shmrDrift)))
      + os.osc(midi2hz(shmrBase + iv5) * (1.0 + 0.001*slowLFO(shmrDrift*1.27))));
bellR = 0.5 * (
        os.osc(midi2hz(shmrBase + iv3) * (1.0 + 0.001*slowLFO(shmrDrift*0.83)))
      + os.osc(midi2hz(shmrBase + iv7) * (1.0 + 0.001*slowLFO(shmrDrift*1.51))));

airL = no.pink_noise : fi.bandpass(2, 4000, 8000) : *(shmrAir);
airR = no.pink_noise : fi.bandpass(2, 4500, 9000) : *(shmrAir);

breath = 0.5 + 0.5*slowLFO(shmrDrift*0.31);

shmrL = (bellL + airL) * shmrLevel * breath;
shmrR = (bellR + airR) * shmrLevel * breath;

shmrMono = (shmrL + shmrR) * 0.5;
shmrSection = shmrMono*(1.0 - shmrWidth) + shmrL*shmrWidth,
              shmrMono*(1.0 - shmrWidth) + shmrR*shmrWidth;

//=================================================================
// COLUMN 4 — SPACE (fog only, no reverb)
//=================================================================
fogCutoff = hslider("h:[4] Space/[1] Fog Cutoff (Hz)",       1200, 200, 8000, 1) : si.smoo;
fogMod    = hslider("h:[4] Space/[2] Fog Mod Depth",         0.5, 0, 1, 0.01) : si.smoo;
fogRate   = hslider("h:[4] Space/[3] Fog Mod Rate (Hz)",     0.05, 0.005, 0.5, 0.001) : si.smoo;
master    = hslider("h:[4] Space/[4] Master",                0.8, 0, 1, 0.01) : si.smoo;

fogLFO  = 0.5 + 0.5*slowLFO(fogRate);
fogF    = fogCutoff * (1.0 - fogMod*0.6 + fogMod*1.2*fogLFO)
          : max(60.0) : min(ma.SR/2.2);
fog     = fi.lowpass(3, fogF), fi.lowpass(3, fogF);

space = fog : *(master), *(master);

//=================================================================
// PROCESS
//=================================================================
sum3 = _,_,_ :> _;

process = (chordSection, subSection, shmrSection)
        : ro.interleave(2, 3)
        : sum3, sum3
        : space;