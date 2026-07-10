// NukeAudio — the audio service provider.
//
// An ordinary plugin (unified model): exports "plugin", provides()="audio",
// queryService() returns the iAudio implementation. The ENGINE drives it once per frame
// from World::Render (listener pose, pause state, update pump); this module only mixes
// and analyses behind the POD seam (service/iAudio.h).
//
// Backend: miniaudio v0.11 (public domain, single header) + stb_vorbis for Ogg Vorbis —
// both compiled INTO this DLL, no extra runtime files. The engine object runs in
// no-device mode and WE own the output device: its data callback reads the master mix
// from the engine and taps it into a lock-free ring for the DSP analysis (FFT -> drum
// onsets, bass energy, chroma/notes) that audio-reactive consumers (the musicvis post
// effect, scripts) read through NukeAudioAnalysis.
//
// Headless mode: if no output device exists (CI, tests; forced via NUKE_AUDIO_NULL=1)
// the mix is pulled manually in update(dt) at the same sample rate — playback and the
// whole analysis pipeline behave identically, just silently.

// miniaudio + stb_vorbis first (engine headers do `using namespace std;` internally).
#define STB_VORBIS_HEADER_ONLY
#include "stb_vorbis.c"            // enables Ogg Vorbis decoding in miniaudio

#define MINIAUDIO_IMPLEMENTATION
#define MA_NO_ENCODING
#include "miniaudio.h"

// NOTE: the stb_vorbis IMPLEMENTATION is included at the very END of this file — its
// internal macros (single-letter ones included) break boost headers otherwise.

// Engine headers last.
#include <interface/NUKEEInteface.h>   // NUKEModule (unified plugin model)
#include <service/iAudio.h>            // the contract this module provides

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using namespace nuke;

// ---- constants ------------------------------------------------------------------------
static constexpr ma_uint32 kSampleRate = 48000;
static constexpr ma_uint32 kChannels   = 2;
static constexpr int       kFFT        = 2048;                  // ~23.4 Hz per bin @ 48 kHz
static constexpr int       kRing       = 8192;                  // mono tap ring (power of 2)
static constexpr int       kBusCount   = 3;                     // 0 Music, 1 SFX, 2 Preview

// ---- lock-free mono tap ring (audio thread writes, game thread reads) ------------------
struct TapRing
{
	float                 buf[kRing] = {};
	std::atomic<uint64_t> written{ 0 };                          // total mono samples written

	void Push(const float* interleaved, ma_uint32 frames, ma_uint32 channels)
	{
		uint64_t w = written.load(std::memory_order_relaxed);
		for (ma_uint32 f = 0; f < frames; ++f)
		{
			float m = 0.0f;
			for (ma_uint32 c = 0; c < channels; ++c) m += interleaved[f * channels + c];
			buf[(w + f) & (kRing - 1)] = m / (float)channels;
		}
		written.store(w + frames, std::memory_order_release);
	}
	// Copy the freshest `count` samples (oldest-first). False until enough audio flowed.
	bool Latest(float* out, int count) const
	{
		uint64_t w = written.load(std::memory_order_acquire);
		if (w < (uint64_t)count) return false;
		uint64_t start = w - (uint64_t)count;
		for (int i = 0; i < count; ++i) out[i] = buf[(start + i) & (kRing - 1)];
		return true;
	}
};

// ---- radix-2 FFT (iterative, real input -> magnitude spectrum) -------------------------
// Small, allocation-free, exact for power-of-2 N. Twiddles + bit-reversal precomputed once.
struct FFT
{
	int   n = 0;
	std::vector<int>   rev;
	std::vector<float> cosT, sinT;   // per-stage twiddles, packed

	void Init(int N)
	{
		n = N;
		rev.resize(N);
		int lg = 0; while ((1 << lg) < N) ++lg;
		for (int i = 0; i < N; ++i)
		{
			int r = 0;
			for (int b = 0; b < lg; ++b) if (i & (1 << b)) r |= 1 << (lg - 1 - b);
			rev[i] = r;
		}
		cosT.resize(N / 2); sinT.resize(N / 2);
		for (int i = 0; i < N / 2; ++i)
		{
			double a = -2.0 * 3.14159265358979323846 * i / N;
			cosT[i] = (float)std::cos(a); sinT[i] = (float)std::sin(a);
		}
	}

	// in: n windowed samples; outMag: n/2 magnitudes. re/im are caller scratch (size n).
	void Magnitudes(const float* in, float* re, float* im, float* outMag) const
	{
		for (int i = 0; i < n; ++i) { re[rev[i]] = in[i]; im[rev[i]] = 0.0f; }
		for (int len = 2; len <= n; len <<= 1)
		{
			int half = len >> 1, step = n / len;
			for (int i = 0; i < n; i += len)
				for (int j = 0; j < half; ++j)
				{
					float wr = cosT[j * step], wi = sinT[j * step];
					int a = i + j, b = i + j + half;
					float tr = re[b] * wr - im[b] * wi;
					float ti = re[b] * wi + im[b] * wr;
					re[b] = re[a] - tr; im[b] = im[a] - ti;
					re[a] += tr;        im[a] += ti;
				}
		}
		for (int i = 0; i < n / 2; ++i)
			outMag[i] = std::sqrt(re[i] * re[i] + im[i] * im[i]) * (2.0f / n);
	}
};

// ---- onset detector: banded spectral flux with an adaptive threshold --------------------
struct OnsetBand
{
	int    lo = 0, hi = 0;          // bin range (inclusive)
	std::vector<float> prev;        // previous magnitudes of the band
	float  fluxEma = 0.0f;          // running mean of the flux (adaptive threshold)
	float  env = 0.0f;              // output envelope 0..1
	double lastOnset = -1.0;        // seconds (beat tracking)
	float  interval = 0.5f;         // smoothed inter-onset interval (seconds)

	void Init(int lo_, int hi_) { lo = lo_; hi = hi_; prev.assign(hi_ - lo_ + 1, 0.0f); }

	// Returns true on a NEW onset this hop.
	bool Step(const float* mag, float dt, double now)
	{
		float flux = 0.0f;
		for (int b = lo; b <= hi; ++b)
		{
			float d = mag[b] - prev[b - lo];
			if (d > 0.0f) flux += d;
			prev[b - lo] = mag[b];
		}
		fluxEma += (flux - fluxEma) * 0.04f;                    // slow adaptive floor
		bool onset = false;
		float thr = fluxEma * 2.2f + 1e-5f;
		if (flux > thr && env < 0.55f)                          // refractory via the envelope itself
		{
			float hit = flux / (thr * 1.6f); if (hit > 1.0f) hit = 1.0f;
			env = hit < env ? env : hit;
			if (lastOnset >= 0.0)
			{
				float iv = (float)(now - lastOnset);
				if (iv > 0.18f && iv < 2.0f)                    // 30..330 bpm plausibility
					interval += (iv - interval) * 0.25f;
			}
			lastOnset = now;
			onset = true;
		}
		env *= std::exp(-dt * 9.0f);                            // percussive decay
		if (env < 1e-4f) env = 0.0f;
		return onset;
	}
};

// ---- the iAudio implementation ----------------------------------------------------------
class MiniAudio final : public iAudio
{
public:
	bool init() override
	{
		if (m_inited) return m_ok;
		m_inited = true;

		// Engine in NO-DEVICE mode: we own the device and pull the mix ourselves — that
		// single read point is also the analysis tap (and the headless fallback).
		ma_engine_config ec = ma_engine_config_init();
		ec.noDevice   = MA_TRUE;
		ec.channels   = kChannels;
		ec.sampleRate = kSampleRate;
		if (ma_engine_init(&ec, &m_engine) != MA_SUCCESS)
		{
			cout << "[NukeAudio]\tengine init FAILED" << endl;
			return false;
		}
		for (int b = 0; b < kBusCount; ++b)
			if (ma_sound_group_init(&m_engine, 0, nullptr, &m_bus[b]) != MA_SUCCESS)
				cout << "[NukeAudio]\tbus " << b << " init failed" << endl;

		const char* nullDev = std::getenv("NUKE_AUDIO_NULL");
		if (!(nullDev && nullDev[0] == '1'))
		{
			ma_device_config dc = ma_device_config_init(ma_device_type_playback);
			dc.playback.format   = ma_format_f32;
			dc.playback.channels = kChannels;
			dc.sampleRate        = kSampleRate;
			dc.dataCallback      = &MiniAudio::DeviceCallback;
			dc.pUserData         = this;
			if (ma_device_init(nullptr, &dc, &m_device) == MA_SUCCESS &&
			    ma_device_start(&m_device) == MA_SUCCESS)
			{
				m_haveDevice = true;
			}
			else
			{
				cout << "[NukeAudio]\tno playback device — running headless (silent) mix" << endl;
				ma_device_uninit(&m_device);
			}
		}
		else cout << "[NukeAudio]\theadless mix forced (NUKE_AUDIO_NULL=1)" << endl;

		m_fft.Init(kFFT);
		m_kick.Init(2, 6);          // ~47..140 Hz — kick / low drums
		m_snare.Init(64, 256);      // ~1.5..6 kHz — snare / hats
		m_ok = true;
		cout << "[NukeAudio]\tready (" << (m_haveDevice ? "device" : "headless") << ", "
		     << kSampleRate << " Hz, ogg/wav/mp3/flac)" << endl;
		return true;
	}

	void shutdown()
	{
		if (!m_inited) return;
		stopAll();
		if (m_haveDevice) { ma_device_uninit(&m_device); m_haveDevice = false; }
		if (m_ok)
		{
			for (int b = 0; b < kBusCount; ++b) ma_sound_group_uninit(&m_bus[b]);
			ma_engine_uninit(&m_engine);
		}
		m_inited = false; m_ok = false;
	}

	void reset() override
	{
		// World switch / play stop: silence the GAME buses, keep the device + preview.
		std::vector<uint64_t> kill;
		for (auto& kv : m_voices)
			if (kv.second.bus != 2) kill.push_back(kv.first);
		for (uint64_t v : kill) stop(v);
		m_gamePaused = false;
	}

	void update(float dt) override
	{
		if (!m_ok) return;
		m_now += (double)dt;

		// Headless: pull the mix manually at the real-time rate (the tap feeds from Read).
		if (!m_haveDevice)
		{
			float scratch[1024 * kChannels];
			ma_uint64 want = (ma_uint64)((double)dt * kSampleRate);
			if (want > kRing / 2) want = kRing / 2;              // hitch guard
			while (want > 0)
			{
				ma_uint64 chunk = want > 1024 ? 1024 : want;
				ma_uint64 got = 0;
				ma_engine_read_pcm_frames(&m_engine, scratch, chunk, &got);
				if (got == 0) break;
				m_ring.Push(scratch, (ma_uint32)got, kChannels);
				want -= got;
			}
		}

		ReapFinished();
		Analyse(dt);
	}

	void setListener(const float pos[3], const float fwd[3], const float up[3]) override
	{
		if (!m_ok) return;
		ma_engine_listener_set_position(&m_engine, 0, pos[0], pos[1], pos[2]);
		ma_engine_listener_set_direction(&m_engine, 0, fwd[0], fwd[1], fwd[2]);
		ma_engine_listener_set_world_up(&m_engine, 0, up[0], up[1], up[2]);
	}

	uint64_t play(const char* path, const NukeVoiceDesc& d) override
	{
		if (!m_ok || !path || !*path) return 0;

		// Decode mode: Memory pre-decodes (short SFX), Stream decodes on the fly (music).
		// Auto picks by file size — long tracks would stall the game thread if pre-decoded.
		ma_uint32 flags = 0;
		int mode = d.decode;
		if (mode == 0)
		{
			FILE* f = fopen(path, "rb");
			long size = 0;
			if (f) { fseek(f, 0, SEEK_END); size = ftell(f); fclose(f); }
			mode = (size > 1536 * 1024) ? 2 : 1;
		}
		flags |= (mode == 2) ? MA_SOUND_FLAG_STREAM : MA_SOUND_FLAG_DECODE;

		int bus = (d.bus >= 0 && d.bus < kBusCount) ? d.bus : 1;
		Voice v;
		v.snd = new ma_sound();
		if (ma_sound_init_from_file(&m_engine, path, flags, &m_bus[bus], nullptr, v.snd) != MA_SUCCESS)
		{
			cout << "[NukeAudio]\tcan't open: " << path << endl;
			delete v.snd;
			return 0;
		}
		v.bus = bus;
		ma_sound_set_volume(v.snd, d.volume);
		ma_sound_set_pitch(v.snd, d.pitch > 0.01f ? d.pitch : 0.01f);
		ma_sound_set_looping(v.snd, d.loop ? MA_TRUE : MA_FALSE);
		if (d.spatial)
		{
			ma_sound_set_spatialization_enabled(v.snd, MA_TRUE);
			ma_sound_set_positioning(v.snd, ma_positioning_absolute);
			ma_sound_set_position(v.snd, d.pos[0], d.pos[1], d.pos[2]);
			ma_sound_set_attenuation_model(v.snd, ma_attenuation_model_linear);
			ma_sound_set_min_distance(v.snd, d.minDist > 0.01f ? d.minDist : 0.01f);
			ma_sound_set_max_distance(v.snd, d.maxDist > d.minDist ? d.maxDist : d.minDist + 0.1f);
		}
		else
			ma_sound_set_spatialization_enabled(v.snd, MA_FALSE);

		if (m_gamePaused && bus != 2) v.gamePaused = true;      // spawned while paused
		else ma_sound_start(v.snd);

		uint64_t id = ++m_nextVoice;
		m_voices[id] = v;
		cout << "[NukeAudio]	voice " << id << " started (file, bus " << bus << "): " << path << endl;
		return id;
	}

	void stop(uint64_t voice) override
	{
		auto it = m_voices.find(voice);
		if (it == m_voices.end()) return;
		FreeVoice(it->second);
		m_voices.erase(it);
	}

	void stopAll() override
	{
		for (auto& kv : m_voices) FreeVoice(kv.second);
		m_voices.clear();
	}

	void setVoicePaused(uint64_t voice, bool paused) override
	{
		Voice* v = Find(voice);
		if (!v) return;
		v->userPaused = paused;
		ApplyPauseState(*v);
	}

	bool isPlaying(uint64_t voice) override
	{
		Voice* v = Find(voice);
		if (!v) return false;
		if (ma_sound_at_end(v->snd)) return false;
		return true;   // paused voices still "exist" — finished/stopped ones don't
	}

	void setVoiceVolume(uint64_t voice, float vol) override { if (Voice* v = Find(voice)) ma_sound_set_volume(v->snd, vol); }
	void setVoicePitch(uint64_t voice, float p) override    { if (Voice* v = Find(voice)) ma_sound_set_pitch(v->snd, p > 0.01f ? p : 0.01f); }
	void setVoicePos(uint64_t voice, const float pos[3]) override
	{
		if (Voice* v = Find(voice)) ma_sound_set_position(v->snd, pos[0], pos[1], pos[2]);
	}

	float voiceTime(uint64_t voice) override
	{
		Voice* v = Find(voice);
		float t = 0.0f;
		if (v) ma_sound_get_cursor_in_seconds(v->snd, &t);
		return t;
	}
	float voiceLength(uint64_t voice) override
	{
		Voice* v = Find(voice);
		float t = 0.0f;
		if (v) ma_sound_get_length_in_seconds(v->snd, &t);
		return t;
	}
	void voiceSeek(uint64_t voice, float sec) override
	{
		if (Voice* v = Find(voice))
			ma_sound_seek_to_pcm_frame(v->snd, (ma_uint64)((double)(sec < 0 ? 0 : sec) * kSampleRate));
	}

	void setBusVolume(int bus, float vol) override
	{
		if (m_ok && bus >= 0 && bus < kBusCount) ma_sound_group_set_volume(&m_bus[bus], vol);
	}
	float getBusVolume(int bus) override
	{
		return (m_ok && bus >= 0 && bus < kBusCount) ? ma_sound_group_get_volume(&m_bus[bus]) : 1.0f;
	}
	void  setMasterVolume(float vol) override { if (m_ok) ma_engine_set_volume(&m_engine, vol); }
	float getMasterVolume() override          { return m_ok ? ma_engine_get_volume(&m_engine) : 1.0f; }

	void setGamePaused(bool paused) override
	{
		if (paused == m_gamePaused) return;
		m_gamePaused = paused;
		for (auto& kv : m_voices)
		{
			if (kv.second.bus == 2) continue;                   // preview never game-pauses
			kv.second.gamePaused = paused;
			ApplyPauseState(kv.second);
		}
	}

	void getAnalysis(NukeAudioAnalysis& out) override { out = m_analysis; }

	// Packed content (3.2): play from MEMORY — the voice owns a copy of the bytes and a
	// ma_decoder over them (decode-on-the-fly; no temp files, the pak stays the only copy).
	uint64_t playData(const void* bytes, uint64_t size, const NukeVoiceDesc& d) override
	{
		if (!m_ok || !bytes || !size) return 0;
		int bus = (d.bus >= 0 && d.bus < kBusCount) ? d.bus : 1;
		Voice v;
		v.data.assign((const char*)bytes, (size_t)size);
		v.dec = new ma_decoder();
		if (ma_decoder_init_memory(v.data.data(), v.data.size(), nullptr, v.dec) != MA_SUCCESS)
		{
			cout << "[NukeAudio]	can't decode memory clip (" << size << " bytes)" << endl;
			delete v.dec;
			return 0;
		}
		v.snd = new ma_sound();
		if (ma_sound_init_from_data_source(&m_engine, v.dec, 0, &m_bus[bus], v.snd) != MA_SUCCESS)
		{
			ma_decoder_uninit(v.dec); delete v.dec; delete v.snd;
			return 0;
		}
		v.bus = bus;
		ma_sound_set_volume(v.snd, d.volume);
		ma_sound_set_pitch(v.snd, d.pitch > 0.01f ? d.pitch : 0.01f);
		ma_sound_set_looping(v.snd, d.loop ? MA_TRUE : MA_FALSE);
		if (d.spatial)
		{
			ma_sound_set_spatialization_enabled(v.snd, MA_TRUE);
			ma_sound_set_positioning(v.snd, ma_positioning_absolute);
			ma_sound_set_position(v.snd, d.pos[0], d.pos[1], d.pos[2]);
			ma_sound_set_attenuation_model(v.snd, ma_attenuation_model_linear);
			ma_sound_set_min_distance(v.snd, d.minDist > 0.01f ? d.minDist : 0.01f);
			ma_sound_set_max_distance(v.snd, d.maxDist > d.minDist ? d.maxDist : d.minDist + 0.1f);
		}
		else
			ma_sound_set_spatialization_enabled(v.snd, MA_FALSE);
		if (m_gamePaused && bus != 2) v.gamePaused = true;
		else ma_sound_start(v.snd);
		uint64_t id = ++m_nextVoice;
		m_voices[id] = std::move(v);
		cout << "[NukeAudio]	voice " << id << " started (memory, bus " << bus << ", " << size << " bytes)" << endl;
		return id;
	}

private:
	struct Voice
	{
		ma_sound* snd = nullptr;
		int  bus = 1;
		bool userPaused = false;   // script/inspector pause
		bool gamePaused = false;   // global PIE/Player pause
		// Memory playback (packed content): the voice OWNS its bytes + decoder.
		std::string  data;
		ma_decoder*  dec = nullptr;
	};

	void FreeVoice(Voice& v)
	{
		ma_sound_uninit(v.snd);
		delete v.snd;
		if (v.dec) { ma_decoder_uninit(v.dec); delete v.dec; v.dec = nullptr; }
	}

	Voice* Find(uint64_t id)
	{
		auto it = m_voices.find(id);
		return it == m_voices.end() ? nullptr : &it->second;
	}

	void ApplyPauseState(Voice& v)
	{
		if (v.userPaused || v.gamePaused) ma_sound_stop(v.snd);          // keeps the cursor
		else if (!ma_sound_at_end(v.snd)) ma_sound_start(v.snd);
	}

	void ReapFinished()
	{
		for (auto it = m_voices.begin(); it != m_voices.end(); )
		{
			if (ma_sound_at_end(it->second.snd) && !ma_sound_is_looping(it->second.snd))
			{
				FreeVoice(it->second);
				it = m_voices.erase(it);
			}
			else ++it;
		}
	}

	static void DeviceCallback(ma_device* dev, void* out, const void*, ma_uint32 frames)
	{
		MiniAudio* self = (MiniAudio*)dev->pUserData;
		ma_uint64 got = 0;
		ma_engine_read_pcm_frames(&self->m_engine, out, frames, &got);
		if (got < frames)   // engine starved — pad with silence, never leave garbage
			memset((float*)out + got * kChannels, 0, (size_t)(frames - got) * kChannels * sizeof(float));
		self->m_ring.Push((const float*)out, frames, kChannels);
	}

	// ---- DSP analysis: FFT over the freshest window -> onsets / bass / chroma ----------
	void Analyse(float dt)
	{
		NukeAudioAnalysis& A = m_analysis;

		float win[kFFT];
		if (!m_ring.Latest(win, kFFT))
		{
			A = NukeAudioAnalysis{};                             // not enough audio yet
			return;
		}

		// RMS loudness BEFORE windowing (energy), with a slow auto-gain reference.
		double sq = 0.0;
		for (int i = 0; i < kFFT; ++i) sq += (double)win[i] * win[i];
		float rms = (float)std::sqrt(sq / kFFT);
		m_rmsRef = std::fmax(m_rmsRef * (1.0f - 0.10f * dt), rms);   // reference decays ~10%/s
		A.energy = m_rmsRef > 1e-5f ? std::fmin(rms / m_rmsRef, 1.0f) : 0.0f;
		if (rms < 1e-5f)                                          // effective silence — settle to calm
		{
			float k = std::exp(-dt * 6.0f);
			A.kick *= k; A.snare *= k; A.bass *= k; A.energy = 0.0f;
			A.noteStrength *= k;
			for (int i = 0; i < 12; ++i) A.chroma[i] *= k;
			return;
		}

		// Hann window + magnitudes.
		for (int i = 0; i < kFFT; ++i)
		{
			float w = 0.5f - 0.5f * std::cos(6.28318530718f * i / (kFFT - 1));
			win[i] *= w;
		}
		float mag[kFFT / 2];
		m_fft.Magnitudes(win, m_re, m_im, mag);

		// Percussive onsets (banded spectral flux) -> kick / snare envelopes + beat clock.
		m_kick.Step(mag, dt, m_now);
		m_snare.Step(mag, dt, m_now);
		A.kick  = m_kick.env;
		A.snare = m_snare.env;

		// Sustained bass: the PEAK bin ~55..260 Hz (a bass line is one strong partial —
		// a band mean dilutes it 10x) against a slow-decay running peak. GATED on fresh
		// drum onsets: a kick burst floods the same bins for ~80 ms, and skipping those
		// frames is what separates "sustained bass" from percussion.
		if (m_kick.env < 0.5f)
		{
			float bcur = 0.0f;
			for (int b = 2; b <= 11; ++b) bcur = std::fmax(bcur, mag[b]);
			m_bassRef = std::fmax(m_bassRef * (1.0f - 0.08f * dt), bcur);
			float braw = m_bassRef > 1e-6f ? std::fmin(bcur / m_bassRef, 1.0f) : 0.0f;
			A.bass += (braw - A.bass) * std::fmin(dt * 12.0f, 1.0f);   // fast attack, smooth ride
		}

		// Chroma: fold bins onto the 12 pitch classes (C = 0). Only from ~380 Hz up — below
		// that a 2048-point bin (23.4 Hz) is WIDER than a semitone, so low bins can't name a
		// note (a bass line is still heard via its harmonics). Magnitude is split between
		// the two neighbouring pitch classes by the bin's fractional semitone position.
		float ch[12] = {};
		const float binHz = (float)kSampleRate / kFFT;
		for (int b = (int)(380.0f / binHz); b < (int)(2500.0f / binHz); ++b)
		{
			float f = b * binHz;
			float n = 12.0f * std::log2(f / 261.6256f);          // semitones from middle C
			float fl = std::floor(n), fr = n - fl;
			int pcA = (((int)fl) % 12 + 12) % 12, pcB = (pcA + 1) % 12;
			float p = mag[b] * mag[b];                            // power-weighted
			ch[pcA] += p * (1.0f - fr);
			ch[pcB] += p * fr;
		}
		float mx = 0.0f;
		for (int i = 0; i < 12; ++i) mx = std::fmax(mx, ch[i]);
		if (mx > 1e-9f)
			for (int i = 0; i < 12; ++i)
			{
				float v = std::sqrt(ch[i] / mx);                  // perceptual-ish compression
				A.chroma[i] += (v - A.chroma[i]) * std::fmin(dt * 10.0f, 1.0f);
			}
		int   dom = 0; float dmx = 0.0f, dsum = 0.0f;
		for (int i = 0; i < 12; ++i)
		{
			dsum += A.chroma[i];
			if (A.chroma[i] > dmx) { dmx = A.chroma[i]; dom = i; }
		}
		A.dominantNote = dom;
		A.noteStrength = (dsum > 1e-5f) ? std::fmax(0.0f, (dmx - dsum / 12.0f) / (dmx + 1e-5f)) : 0.0f;

		// Beat clock from the kick's inter-onset interval.
		if (m_kick.lastOnset >= 0.0 && m_kick.interval > 0.05f)
		{
			float ph = (float)((m_now - m_kick.lastOnset) / m_kick.interval);
			A.beatPhase = ph - std::floor(ph);
			A.bpm = 60.0f / m_kick.interval;
		}
	}

	bool m_inited = false, m_ok = false, m_haveDevice = false, m_gamePaused = false;
	ma_engine m_engine{};
	ma_device m_device{};
	ma_sound_group m_bus[kBusCount]{};
	std::map<uint64_t, Voice> m_voices;
	uint64_t m_nextVoice = 0;

	TapRing m_ring;
	FFT     m_fft;
	float   m_re[kFFT] = {}, m_im[kFFT] = {};
	OnsetBand m_kick, m_snare;
	float   m_rmsRef = 0.0f, m_bassRef = 0.0f;
	double  m_now = 0.0;
	NukeAudioAnalysis m_analysis;
};
static MiniAudio gAudio;

// ---- Plugin export (unified plugin model) ---------------------------------------------
struct NukeAudioModule : public NUKEModule
{
	NukeAudioModule()
	{
		strcpy(title, "Nuke Audio");
		strcpy(description, "Audio playback + music analysis backed by miniaudio (embedded; ogg/wav/mp3/flac).");
		strcpy(author, "Luastris");
		strcpy(site, "https://luastris.com");
		strcpy(version, "0.1.0.0");
		tags = { "audio", "sound", "music", "miniaudio", "vorbis" };
	}

	const char* provides() override { return "audio"; }
	void*       queryService() override { return static_cast<iAudio*>(&gAudio); }

	void OnLoad() override {}          // AudioSource/AudioListener are ENGINE components — already registered
	void Run(AppInstance* inst) override { instance = inst; stopped = false; }   // driven by World::Render
	bool HasSettings() override { return false; }
	void Settings() override {}
	void Shutdown() override
	{
		gAudio.shutdown();             // loader revoked the "audio" service before calling this
		stopped = true;
	}
};

extern "C" BOOST_SYMBOL_EXPORT NukeAudioModule plugin;
NukeAudioModule plugin;

// stb_vorbis implementation LAST (after miniaudio's implementation, and after every other
// header — its internal macros must not leak into boost/engine code above).
#undef STB_VORBIS_HEADER_ONLY
#include "stb_vorbis.c"
