#pragma once
#ifdef INCLUDED_FROM_MAIN_INO
#include "math.hpp"
#include <type_traits>
#include <cmath>
#include <math.h>

template<typename Tsrate = Tcoeff, typename Tfreq = Tcoeff, typename Tqual = Tcoeff>
struct fircoeffs
{
  using coeff_type = Tcoeff;
  using f0_type = Tfreq;
  using sr_type = Tsrate;
  using q_type = Tqual;

  union {
    struct {
      union {
        struct {
          coeff_type a0, a1, a2;
        };
        struct {
          coeff_type a[3];
        };
      };
      union {
        struct {
          coeff_type b0, b1, b2;
        };
        struct {
          coeff_type b[3];
        };
      };
    };
    coeff_type raw[6];
  };

  fircoeffs() {
    a0 = 0.0; b0 = 0.0;
    a1 = 0.0; b1 = 0.0;
    a2 = 0.0; b2 = 0.0;
  }

  fircoeffs(const fircoeffs& other) {
    *this = other;
  }

  fircoeffs& operator=(const fircoeffs& other) {
    a0 = other.a0; a1 = other.a1; a2 = other.a2;
    b0 = other.b0; b1 = other.b1; b2 = other.b2;
    return *this;
  }

};

fircoeffs<float> lerp(fircoeffs<float> a, const fircoeffs<float>& b, float x)
{
  x=(x<0.0)?(0.0):(x>1.0)?(1.0):(x);
  for(int i = 0; i < 6; ++i)
    a.raw[i] = b.raw[i] * x + a.raw[i] * (float)((float)1.0 - (float)x);
  return a;
}

fircoeffs<float> lowpass(float Fs, float f0, float Q)
{
  fircoeffs<float> out;
  float w0 = 2.0*PI*f0/Fs;
  float alpha = sin(w0)/(2.0*Q);
  out.b0 =  (1.0 - cos(w0))/2.0;
  out.b1 =   1.0 - cos(w0);
  out.b2 =  (1.0 - cos(w0))/2.0;
  out.a0 =   1.0 + alpha;
  out.a1 =  -2.0*cos(w0);
  out.a2 =   1.0 - alpha;
  return out;
}

struct fircoeffs_smoothed : fircoeffs<float>
{
  float smpos = 1.0;
  float smstep = 1.0 / 50.0;
  fircoeffs<float> smto;
  fircoeffs<float> smfrom;

  fircoeffs_smoothed(fircoeffs<float> initial, float steps = 50.0): fircoeffs<float>(initial), smstep(1.0 / steps), smpos(1.0), smto(initial), smfrom(initial) {}

  explicit fircoeffs_smoothed(float steps = 50.0): fircoeffs_smoothed(fircoeffs<float>(), steps) {}


  fircoeffs_smoothed& smooth_to(fircoeffs<float> new_smto) {
    fircoeffs<float> new_smfrom = lerp(smfrom, smto, smpos);
    smpos = 0.0;
    smto = new_smto;
    smfrom = new_smfrom;
    *this = new_smfrom;
    return *this;
  }

  void step()
  {
    if(smpos < 1.0)
    {
      smpos = smpos + smstep;
      if(smpos > 1.0)
        smpos = 1.0;
      *this = lerp(smfrom, smto, smpos);
    }
  }

};


struct firfilter : public fircoeffs<float>
{
  float x[3][3], y[3][3];

  firfilter(float Fs, float f0, float Q = 1.0):
      fircoeffs<float>(::lowpass(Fs, f0, Q))
  {}

  void set_lpf(float Fs, float f0, float Q = 1.0)
  {
    fircoeffs<float> lpf = ::lowpass(Fs, f0, Q);
    *this = lpf;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        x[i][j] = y[i][j] = 0.0;
  }

  firfilter& operator=(const fircoeffs<float>& other) {
    a0 = other.a0; a1 = other.a1; a2 = other.a2;
    b0 = other.b0; b1 = other.b1; b2 = other.b2;
    return *this;
  }

  void filter(CRGB cin, CRGB &cout) {
    for(int j = 0; j<3; j++) {
      float *xn = x[j], *yn = y[j];
      xn[2] = xn[1]; xn[1] = xn[0]; xn[0] = cin.raw[j];
      yn[2] = yn[1]; yn[1] = yn[0]; 
      yn[0] = (b0/a0)*xn[0] + (b1/a0)*xn[1] + (b2/a0)*xn[2]
                      - (a1/a0)*yn[1] - (a2/a0)*yn[2];
      cout.raw[j] = fmin(0xffff, fmax(0x0000, yn[0]));
    }
  }
};

template<int N>
struct firarray : public fircoeffs_smoothed
{
  float x[N*3][3], y[N*3][3];

  firarray(float Fs, float f0, float Q = 1.0):
      fircoeffs_smoothed(lowpass(Fs, f0, Q))
  {
  }

  void smooth_to_lpf(float Fs, float f0, float Q = 1.0)
  {
    smooth_to(lowpass(Fs, f0, Q));
  }

  void filter(CRGB cin[N], CRGB cout[N]) {
    step();
    for(int i = 0; i<N; i++) {
      for(int j = 0; j<3; j++) {
        float *xn = x[i*3+j], *yn = y[i*3+j];
        xn[2] = xn[1]; xn[1] = xn[0]; xn[0] = cin[i].raw[j];
        yn[2] = yn[1]; yn[1] = yn[0]; 
        yn[0] = (b0/a0)*xn[0] + (b1/a0)*xn[1] + (b2/a0)*xn[2]
                        - (a1/a0)*yn[1] - (a2/a0)*yn[2];
        cout[i].raw[j] = fmin(0xff, fmax(0x00, yn[0]));
      }
    }
  }
};

#endif INCLUDED_FROM_MAIN_INO
