#ifndef FREEVERB_H
#define FREEVERB_H

/*  Mono Freeverb for Mozzi on ESP32-S3.

    A Schroeder/Moorer-style reverb: 8 damped comb filters in parallel feed
    4 allpass filters in series. This is Jezar at Dreampoint's public-domain
    "Freeverb" algorithm (the basis of most lush, cheap reverbs) - far smoother
    than Mozzi's tiny ReverbTank, and well within the S3's FPU + RAM budget.

    The classic delay-line lengths are tuned for 44.1kHz; here they're scaled
    at compile time to MOZZI_AUDIO_RATE (32768Hz on this board), so the room
    proportions stay correct. Uses float maths - fine because Mozzi's
    updateAudio() runs in task context on ESP32 (not a hardware ISR) and the
    S3 has a single-precision FPU.

    process() returns the wet-only signal; mix it with your dry signal in the
    sketch, exactly like ReverbTank.
*/

// scale a 44.1kHz tuning to the actual audio rate, as a compile-time constant
#define FV_LEN(n) ((int)((long)(n) * MOZZI_AUDIO_RATE / 44100L))

class Freeverb {
public:
  Freeverb(){
    // wire each filter to its buffer + (rate-scaled) length
    static const int cl[NUM_COMBS]    = { FV_LEN(1116), FV_LEN(1188), FV_LEN(1277), FV_LEN(1356),
                                          FV_LEN(1422), FV_LEN(1491), FV_LEN(1557), FV_LEN(1617) };
    static const int al[NUM_ALLPASS]  = { FV_LEN(556), FV_LEN(441), FV_LEN(341), FV_LEN(225) };
    float* cb[NUM_COMBS]   = { c0,c1,c2,c3,c4,c5,c6,c7 };
    float* ab[NUM_ALLPASS] = { a0,a1,a2,a3 };
    for (int i=0;i<NUM_COMBS;i++){   combBuf_[i]=cb[i]; combLen_[i]=cl[i]; }
    for (int i=0;i<NUM_ALLPASS;i++){ apBuf_[i]=ab[i];   apLen_[i]=al[i];  }

    setRoomSize(0.72f); // tail length
    setDamp(0.45f);     // high-frequency damping
    setWet(0.5f);
    mute();
  }

  // process one dry input sample, return the wet (reverberated) signal only
  inline float process(float input){
    // The +1e-18 is an anti-denormal offset: it keeps the comb/allpass feedback
    // paths from decaying into denormal floats once the input goes quiet, which
    // can otherwise trap the FPU into a slow path and starve the audio callback
    // (heard as the output dropping to silence). It is far too small to hear.
    float in = input * FIXED_GAIN + 1e-18f;
    float out = 0.0f;
    // parallel damped comb filters build up the dense tail
    for (int i=0;i<NUM_COMBS;i++){
      float* buf = combBuf_[i];
      int idx = combIdx_[i];
      float y = buf[idx];
      combStore_[i] = y * damp2_ + combStore_[i] * damp1_; // one-pole lowpass in the feedback
      buf[idx] = in + combStore_[i] * feedback_;
      if (++idx >= combLen_[i]) idx = 0;
      combIdx_[i] = idx;
      out += y;
    }
    // series allpass filters smear it into a smooth diffuse decay
    for (int i=0;i<NUM_ALLPASS;i++){
      float* buf = apBuf_[i];
      int idx = apIdx_[i];
      float bufout = buf[idx];
      float y = bufout - out;          // allpass
      buf[idx] = out + bufout * 0.5f;  // fixed allpass feedback
      if (++idx >= apLen_[i]) idx = 0;
      apIdx_[i] = idx;
      out = y;
    }
    return out * wet_;
  }

  void setRoomSize(float v){ feedback_ = v * 0.28f + 0.7f; } // 0..1 -> ~0.70..0.98
  void setDamp(float v){ damp1_ = v; damp2_ = 1.0f - v; }
  void setWet(float v){ wet_ = v; }

  void mute(){
    for (int i=0;i<NUM_COMBS;i++){ combIdx_[i]=0; combStore_[i]=0.0f;
      for (int j=0;j<combLen_[i];j++) combBuf_[i][j]=0.0f; }
    for (int i=0;i<NUM_ALLPASS;i++){ apIdx_[i]=0;
      for (int j=0;j<apLen_[i];j++) apBuf_[i][j]=0.0f; }
  }

private:
  static const int NUM_COMBS   = 8;
  static const int NUM_ALLPASS = 4;
  static constexpr float FIXED_GAIN = 0.015f; // input scaling, keeps levels sane

  // delay-line storage, sized at compile time from the audio rate
  float c0[FV_LEN(1116)], c1[FV_LEN(1188)], c2[FV_LEN(1277)], c3[FV_LEN(1356)];
  float c4[FV_LEN(1422)], c5[FV_LEN(1491)], c6[FV_LEN(1557)], c7[FV_LEN(1617)];
  float a0[FV_LEN(556)],  a1[FV_LEN(441)],  a2[FV_LEN(341)],  a3[FV_LEN(225)];

  float* combBuf_[NUM_COMBS];  int combLen_[NUM_COMBS];  int combIdx_[NUM_COMBS];  float combStore_[NUM_COMBS];
  float* apBuf_[NUM_ALLPASS];  int apLen_[NUM_ALLPASS];  int apIdx_[NUM_ALLPASS];

  float feedback_, damp1_, damp2_, wet_;
};

#endif // FREEVERB_H
