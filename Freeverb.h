#ifndef FREEVERB_H
#define FREEVERB_H

/*  Stereo Freeverb for Mozzi on ESP32-S3.

    A Schroeder/Moorer-style reverb: 8 damped comb filters in parallel feed
    4 allpass filters in series. This is Jezar at Dreampoint's public-domain
    "Freeverb" algorithm (the basis of most lush, cheap reverbs) - far smoother
    than Mozzi's tiny ReverbTank, and well within the S3's FPU + RAM budget.

    Stereo: the same mono dry input is fed to two independent comb/allpass banks.
    The right bank's delay lines are lengthened by FV_SPREAD samples, so the two
    tails decorrelate and the ambience spreads across the image. The dry signal
    is left mono/centred by the caller, so the width lives only in the tail
    (subtle, and it collapses cleanly to mono).

    The classic delay-line lengths are tuned for 44.1kHz; here they're scaled
    at compile time to MOZZI_AUDIO_RATE (32768Hz on this board), so the room
    proportions stay correct. Uses float maths - fine because Mozzi's
    updateAudio() runs in task context on ESP32 (not a hardware ISR) and the
    S3 has a single-precision FPU.

    processStereo() returns the wet-only signal for each channel; mix it with
    your dry signal in the sketch, exactly like the old mono process().
*/

// scale a 44.1kHz tuning to the actual audio rate, as a compile-time constant
#define FV_LEN(n) ((int)((long)(n) * MOZZI_AUDIO_RATE / 44100L))
#define FV_SPREAD FV_LEN(23) // right-channel delay offset (classic Freeverb stereospread)

class Freeverb {
public:
  Freeverb(){
    // rate-scaled comb/allpass lengths; the right bank is offset by FV_SPREAD
    static const int cl[NUM_COMBS]   = { FV_LEN(1116), FV_LEN(1188), FV_LEN(1277), FV_LEN(1356),
                                         FV_LEN(1422), FV_LEN(1491), FV_LEN(1557), FV_LEN(1617) };
    static const int al[NUM_ALLPASS] = { FV_LEN(556), FV_LEN(441), FV_LEN(341), FV_LEN(225) };
    float* cbL[NUM_COMBS]   = { c0L,c1L,c2L,c3L,c4L,c5L,c6L,c7L };
    float* cbR[NUM_COMBS]   = { c0R,c1R,c2R,c3R,c4R,c5R,c6R,c7R };
    float* abL[NUM_ALLPASS] = { a0L,a1L,a2L,a3L };
    float* abR[NUM_ALLPASS] = { a0R,a1R,a2R,a3R };
    for (int i=0;i<NUM_COMBS;i++){
      combBufL_[i]=cbL[i]; combLenL_[i]=cl[i];
      combBufR_[i]=cbR[i]; combLenR_[i]=cl[i]+FV_SPREAD;
    }
    for (int i=0;i<NUM_ALLPASS;i++){
      apBufL_[i]=abL[i]; apLenL_[i]=al[i];
      apBufR_[i]=abR[i]; apLenR_[i]=al[i]+FV_SPREAD;
    }
    setRoomSize(0.72f); // tail length
    setDamp(0.45f);     // high-frequency damping
    setWet(0.5f);
    mute();
  }

  // process one mono dry sample; write the wet (reverberated) L/R signals
  inline void processStereo(float input, float& outL, float& outR){
    // The +1e-18 is an anti-denormal offset: it keeps the comb/allpass feedback
    // paths from decaying into denormal floats once the input goes quiet, which
    // can otherwise trap the FPU into a slow path and starve the audio callback
    // (heard as the output dropping to silence). It is far too small to hear.
    float in = input * FIXED_GAIN + 1e-18f;
    outL = processChannel(in, combBufL_, combLenL_, combIdxL_, combStoreL_, apBufL_, apLenL_, apIdxL_);
    outR = processChannel(in, combBufR_, combLenR_, combIdxR_, combStoreR_, apBufR_, apLenR_, apIdxR_);
  }

  // 0..1 -> feedback ~0.70..0.999. Tail length ~ delay/(1-feedback), so the top
  // of the range is huge (~minutes) and nearly a freeze; 1.0 would hold forever.
  void setRoomSize(float v){ feedback_ = v * 0.299f + 0.7f; }
  void setDamp(float v){ damp1_ = v; damp2_ = 1.0f - v; }
  void setWet(float v){ wet_ = v; }

  void mute(){
    for (int i=0;i<NUM_COMBS;i++){
      combIdxL_[i]=0; combStoreL_[i]=0.0f; for (int j=0;j<combLenL_[i];j++) combBufL_[i][j]=0.0f;
      combIdxR_[i]=0; combStoreR_[i]=0.0f; for (int j=0;j<combLenR_[i];j++) combBufR_[i][j]=0.0f;
    }
    for (int i=0;i<NUM_ALLPASS;i++){
      apIdxL_[i]=0; for (int j=0;j<apLenL_[i];j++) apBufL_[i][j]=0.0f;
      apIdxR_[i]=0; for (int j=0;j<apLenR_[i];j++) apBufR_[i][j]=0.0f;
    }
  }

private:
  static const int NUM_COMBS   = 8;
  static const int NUM_ALLPASS = 4;
  static constexpr float FIXED_GAIN = 0.015f; // input scaling, keeps levels sane

  // run one comb+allpass bank (a single channel) on the already-scaled input
  inline float processChannel(float in, float** combBuf, int* combLen, int* combIdx,
                              float* combStore, float** apBuf, int* apLen, int* apIdx){
    float out = 0.0f;
    // parallel damped comb filters build up the dense tail
    for (int i=0;i<NUM_COMBS;i++){
      float* buf = combBuf[i];
      int idx = combIdx[i];
      float y = buf[idx];
      combStore[i] = y * damp2_ + combStore[i] * damp1_; // one-pole lowpass in the feedback
      buf[idx] = in + combStore[i] * feedback_;
      if (++idx >= combLen[i]) idx = 0;
      combIdx[i] = idx;
      out += y;
    }
    // series allpass filters smear it into a smooth diffuse decay
    for (int i=0;i<NUM_ALLPASS;i++){
      float* buf = apBuf[i];
      int idx = apIdx[i];
      float bufout = buf[idx];
      float y = bufout - out;          // allpass
      buf[idx] = out + bufout * 0.5f;  // fixed allpass feedback
      if (++idx >= apLen[i]) idx = 0;
      apIdx[i] = idx;
      out = y;
    }
    return out * wet_;
  }

  // delay-line storage, sized at compile time from the audio rate.
  // left bank = classic lengths; right bank = lengths + FV_SPREAD (stereo).
  float c0L[FV_LEN(1116)], c1L[FV_LEN(1188)], c2L[FV_LEN(1277)], c3L[FV_LEN(1356)];
  float c4L[FV_LEN(1422)], c5L[FV_LEN(1491)], c6L[FV_LEN(1557)], c7L[FV_LEN(1617)];
  float a0L[FV_LEN(556)],  a1L[FV_LEN(441)],  a2L[FV_LEN(341)],  a3L[FV_LEN(225)];
  float c0R[FV_LEN(1116)+FV_SPREAD], c1R[FV_LEN(1188)+FV_SPREAD], c2R[FV_LEN(1277)+FV_SPREAD], c3R[FV_LEN(1356)+FV_SPREAD];
  float c4R[FV_LEN(1422)+FV_SPREAD], c5R[FV_LEN(1491)+FV_SPREAD], c6R[FV_LEN(1557)+FV_SPREAD], c7R[FV_LEN(1617)+FV_SPREAD];
  float a0R[FV_LEN(556)+FV_SPREAD],  a1R[FV_LEN(441)+FV_SPREAD],  a2R[FV_LEN(341)+FV_SPREAD],  a3R[FV_LEN(225)+FV_SPREAD];

  float* combBufL_[NUM_COMBS];  int combLenL_[NUM_COMBS];  int combIdxL_[NUM_COMBS];  float combStoreL_[NUM_COMBS];
  float* combBufR_[NUM_COMBS];  int combLenR_[NUM_COMBS];  int combIdxR_[NUM_COMBS];  float combStoreR_[NUM_COMBS];
  float* apBufL_[NUM_ALLPASS];  int apLenL_[NUM_ALLPASS];  int apIdxL_[NUM_ALLPASS];
  float* apBufR_[NUM_ALLPASS];  int apLenR_[NUM_ALLPASS];  int apIdxR_[NUM_ALLPASS];

  float feedback_, damp1_, damp2_, wet_;
};

#endif // FREEVERB_H
