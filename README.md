# NukeAudio

The default audio module for [NukeEngine](https://github.com/Luastris/NukeEngine-Eco):
[miniaudio](https://github.com/mackron/miniaudio) + stb_vorbis compiled INTO the DLL —
a clean clone builds with no extra dependencies. Implements the engine's `iAudio` POD
seam.

## Features

- Plain audio FILES as clips — ogg/wav/mp3/flac by content-relative path, no custom
  asset format; packed games decode straight from pak bytes (nothing extracts).
- `AudioSource` (clip, volume/pitch/loop, 3D spatialization with min/max distance, bus,
  play-on-start, decode mode) + `AudioListener` components.
- Mix buses (Music / SFX / editor Preview) + master volume; game pause pauses game buses
  only.
- The reflected `nuke::Audio` facade — C++, Lua (`nuke.Audio.*`), C# (`Audio.*`):
  `Play/PlayAt/Stop/IsPlaying/Seek/Time/Length/SetVolume/SetPitch/...`, plus
  `PlayData(bytes)` to play script-composed audio from memory.
- **Music analysis** of the master mix — beat/kick/snare envelopes, bass energy, chroma
  (dominant note), BPM/beat phase — exposed to scripts (`Audio.GetKick()` ...) and fed
  into post-effect system params for audio-reactive visuals.

```csharp
Audio.Play("music/theme.ogg", 0.8, loop: true, bus: 0);
double kick = Audio.GetKick();   // 0..1 envelope, this frame
```

## Building

Part of the [NukeEngine-Eco](https://github.com/Luastris/NukeEngine-Eco) superbuild, or
standalone: `cmake -S . -B build -G "Visual Studio 17 2022" -A x64` +
`cmake --build build --config Debug` (needs `VCPKG_ROOT`; engine first).
