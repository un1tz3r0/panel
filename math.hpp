#ifdef INCLUDED_FROM_MAIN_INO

#pragma once

#include <inttypes.h>
#include <cmath>
#include <math.h>
//#include <type_traits>
//#include <array>
#include <vector>

/*
template<typename T, typename U, typename = std::enable_if<
    std::is_unsigned<T>::value && std::is_floating_point<U>::value
  >::type>
constexpr U convert_helper(T v) { return (U)v / (U)std::numeric_limits<T>::max() }

template<typename T, typename U, typename = void, typename = std::enable_if<
    std::is_floating_point<T>::value && std::is_integer<U>::value
  >::type>
*/

// ---------------------------------------------------------------------------

template<typename T> //, typename std::enable_if<std::is_arithmetic<T>::value>::type>
T abs(T x)
{
  if(x < 0.0)
    return -x;
  else
    return x;
}

template<typename T> T declval();

template<typename T> void swap(T& x, T& y)
{
  T z = x;
  x = y;
  y = z;
}

#if 0 // BROKEN

template<typename T, typename U, typename... Vs>
auto min(T a, U b, Vs... cs) -> decltype( min(declval<bool>() ? declval<T>() : declval<U>(), declval<Vs>()...) )
{
  return min((a < b) ? a : b, cs...);
}

template<typename T, typename U, typename... Vs>
auto max(T a, U b, Vs... cs) -> decltype( max(declval<bool>() ? declval<T>() : declval<U>(), declval<Vs>()...) )
{
  return max((a < b) ? b : a, cs...);
}

#endif // 0 // BROKEN

template<typename T, typename U>
auto min(T a, U b) -> decltype( true ? a : b )
{
  return (a < b) ? a : b;
}

template<typename T, typename U>
auto max(T a, U b) -> decltype( true ? a : b )
{
  return (a < b) ? b : a;
}

/*template<typename T>
auto min(T a) -> T
{
  return a;
}

template<typename T>
auto max(T a) -> T
{
  return a;
}*/

template<typename T, typename U=T, typename V=U>
auto clip(T x, U l=0.0, V h=1.0) -> decltype(declval<bool>() ? declval<T>() : declval<bool>() ? declval<U>() : declval<V>())
{
  if(l > h)
    return x < h ? h : x > l ? l : x;
  else
    return x < l ? l : x > h ? h : x;
}

float lin_scale(float x, float xa, float xb, float ya, float yb)
{
  return ((x - xa) / (xb - xa)) * (yb - ya) + ya;
}

template<typename Tfrom, typename Tto>
Tto scale(Tfrom x, Tfrom xa, Tfrom xb, Tto ya, Tto yb)
{
  return ((x - xa) * (yb - ya)) / (xb - xa) + ya;
}

float log_scale(float x, float xa, float xb, float yl, float yh, float ys)
{
  return exp( ((x - xa) / (xb - xa)) * (log(ys + yh - yl) - log(ys)) + log(ys)) + yl - ys;
}

// --------------------------------------------------------------------

float rand16f()
{
  return (float)((float)random16() - (float)0x7fff) / (float)0x7fff;
}

// -------------------------------------------------------------------------

/*
template<class T, typename = typename std::enable_if<std::is_floating_point<T>::value>::type>
T gaussian (T x, T mu, T sigma) {
  return std::exp( -(((x-mu)/(sigma))*((x-mu)/(sigma)))/2.0 );
}
*/


namespace std
{
  constexpr auto pi() -> double //decltype(std::atan(1.0) * 4.0)
  {
    return 3.141592653589793115997963468544185161590576171875;
    //return std::atan(1.0) * 4.0;
  }
}

// ------------------------------------------------------------------------------

float wrap(float x, float m)
{
  return fmod((x < 0.0) ? ((float)m - (float)fmod(- (float)x, m)) : (x), m);
}

double wrap(double x, double m)
{
  return fmod((x < 0.0) ? ((double)m - (double)fmod(- (double)x, m)) : (x), m);
}

//template<typename T, typename = std::enable_if<std::is_integral<T>::value>::type>
signed int wrap(signed int x, signed int m)
{
  return ((x<0)?(m-(-x%m)):(x))%m;
}

//template<typename T, typename = void, std::enable_if<std::is_integral<T>::value, std::enable_if<std::is_unsigned<T>::value>::type>::type>
unsigned int wrap(unsigned int x, unsigned int m)
{
  return x % m;
}


// --------------------------------------------------------------------
/*
template<typename T, typename U>
struct common_type_helper
{
  using type = decltype( std::declval<bool>() ? std::declval<T>() : std::declval<U>() );
};

template<typename T, typename Us...>
struct common_type
{
  using type = typename common_type_helper< T, common_type< Us... >::type >::type;
};

template<typename T>
struct common_type
{
  using type = T;
};
*/ 

CRGB lerp(CRGB a, CRGB b, float x)
{
  if(x < 0.0)
    x = 0.0;
  if(x > 1.0)
    x = 1.0;
  return a.lerp16(b, (int)(x * (float)0xffff));
}

CRGB lerp(CRGB a, CRGB b, uint8_t x)
{
  return a.lerp8(b, x);
}

uint8_t lerp(uint8_t a, uint8_t b, uint8_t x)
{
  return ((a * (255-x)) + (b * x)) >> 8; // * (float)0xffff));
}

uint8_t lerp(uint8_t a, uint8_t b, float x)
{
  if(x < 0.0)
    x = 0.0;
  if(x > 1.0)
    x = 1.0;
  return (float)(((float)a * (float)((float)1.0 - (float)x)) + (float)((float)b * (float)x)); // * (float)0xffff));
}

float lerp(float a, float b, float x)
{
  if(x < 0.0)
    x = 0.0;
  if(x > 1.0)
    x = 1.0;
  return (float)(((float)a * (float)((float)1.0 - (float)x)) + (float)((float)b * (float)x)); // * (float)0xffff));
}


/*
CRGB add(CRGB a, CRGB b, float x)
{
  if(x < 0.0)
    x = 0.0;
  if(x > 1.0)
    x = 1.0;
  return CRGB(
    MIN(255, a.r + (float)b.r * (float)x),
    MIN(255, a.g + (float)b.g * (float)x),
    MIN(255, a.b + (float)b.b * (float)x)
  );
}
*/

/*
void alphablend(CRGB &Ca, float &Aa, CRGB Cb, float Ab)
{
  Aa = clip(Aa);
  Ab = clip(Ab);
  float Ao = (float)1.0 - (float)((float)1.0 - (float)Aa) * (float)((float)1.0 - (float)Ab);
  CRGB Co = CRGB(
      (((float)Ca.r * (float)(Aa * (float)1)) + ((float)Cb.r * (float)( (float)((float)((float)1.0 - (float)Aa) * (float)Ab) * (float)1 ))) / (float)((float)Ao * (float)1),
      (((float)Ca.g * (float)(Aa * (float)1)) + ((float)Cb.g * (float)( (float)((float)((float)1.0 - (float)Aa) * (float)Ab) * (float)1 ))) / (float)((float)Ao * (float)1),
      (((float)Ca.b * (float)(Aa * (float)1)) + ((float)Cb.b * (float)( (float)((float)((float)1.0 - (float)Aa) * (float)Ab) * (float)1 ))) / (float)((float)Ao * (float)1)
    );
  
  Ca = Co;
  Aa = Ao;
}
*/

// -------------------------------------------------------------------------

class gaussian_blur
{
public:
  std::vector<float> kernel;
  float radius;

  gaussian_blur()
  {
    radius = 0.0;
    kernel.resize(0);
  }

  ~gaussian_blur()
  {
  }

  void set_radius(float newradius)
  {
    double sum = 0.0;
    double sigma = 1.0;
    double r, s = 2.0 * sigma * sigma;

    kernel.resize((int)ceil(newradius)+1);
    radius = newradius;
    for(int i = 0; i <= ceil(radius); i++)
    { 
      r = i;
      kernel[i] = (exp(-(r*r)/s))/(std::pi()*s);
      sum += kernel[i] * ((i > 0) ? (2.0) : (1.0));
    }

    for(int i = 0; i <= ceil(radius); i++)
    {
      kernel[i] = kernel[i] / sum;
    }
  }

  explicit gaussian_blur(float r)
  {
    set_radius(r);
  }

  void apply(size_t size, const CRGB *inarr, CRGB *outarr)
  {
    int ksize = ceil(radius);
    for(int i = 0; i < size; i++)
    {
      for(int w = 0; w < 3; w++)
      {
        float outv = (float)(inarr[i].raw[w]) * (float)kernel[0];
        for(int j = 1; j <= ksize; j++) {
          outv = (float)outv + (float)((float)inarr[(int)max(0, i-j)].raw[w] + (float)inarr[(int)min((int)size-1, (int)(i+j))].raw[w]) * (float)kernel[j];
        }
        outarr[i].raw[w] = (float)outv;
      }
    }
  }

};

// ------------------------------------------------------------

class spatial_boxblur
{
public:
  spatial_boxblur(int _passes): passes(_passes) {}

  void set_passes(int _passes) { passes = _passes; }
  int get_passes() const { return passes; }

  void apply(int count, CRGB *buf)
  {
    for(int pass = 0; pass < passes; pass++)
    {
      CRGB o1, o0;
      uint16_t accum[3];

      // prefill first pixel of output delay line, and load first two pixels into the accumulator
      for(int j = 0; j < 3; j++)
      {
        o0.raw[j] = (buf[0].raw[j] * 2 + buf[1].raw[j]) / 3;
        accum[j] = buf[0].raw[j] + buf[1].raw[j];
      }

      // process pixels in-place by delaying overwriting by one pixel, so that 
      // p[x-1] is changed after calculating avg(p[x-1], p[x], p[x+1])
      for(int i = 1; i < count-1; i++)
      {
        for(int j = 0; j < 3; j++)
        {
          o1.raw[j] = o0.raw[j]; // move the previous output to the end of the delay line
          accum[j] += buf[i+1].raw[j]; // accumulate p[x+1], accum now holds p[x-1] + p[x] + p[x+1]
          o0.raw[j] = accum[j]/3; // put average of accumulator at the front of the delay line
          accum[j] -= buf[i-1].raw[j]; // un-accumulate p[x-1], accum now only holds p[x] + p[x+1]...
          buf[i-1].raw[j] = o1.raw[j]; // write delayed output from last iteration to p[x-1]
          // the accumulator now holds p[x] + p[x+1], which when x++ becomes p[x-1] + p[x]
        }
      }

      // calculate p[N] and write p[N-1] in delay line from last loop iteration 
      for(int j = 0; j < 3; j++)
      {
        accum[j] += buf[count-1].raw[j];
        buf[count-2].raw[j] = o0.raw[j];
        buf[count-1].raw[j] = accum[j]/3;
      }
    }
  }

private:
  int passes;
};

// --------------------------------------------------------------------------------------

class temporal_expblur
{
public:
  temporal_expblur(size_t _size, float _weight): size(_size), weight(_weight)
  {
    accum.resize(size*3);
    first = true;
  }

  void set_weight(float _w)
  {
    weight = _w;
  }

  float get_weight() const
  {
    return weight;
  }

  void apply(size_t so, CRGB *io)
  {
    if(so != size)
    {
      accum.resize(so);
      size = so;
      first = true;
    }

    for(int x = 0; x < size; x++)
    {
      for(int j = 0; j < 3; j++)
      {
        accum[x*3+j] = lerp((float)io[x].raw[j], accum[x*3+j], (first)?(0.0):(weight));
        io[x].raw[j] = accum[x*3+j];
      }
    }

    first = false;
  }

private:
  size_t size;
  float weight;
  bool first;
  std::vector<float> accum;
};

// --------------------------------------------------------------------------------------------------

#if 0 // kinda half done heh

// piecewise linear interpolation, arbitrary number of points
template<typename Tx, typename Ty>
class piecewise_linear
{
  using key_type = Tx;
  using value_type = Ty;
  using point_type = std::pair<Tx, Ty>;

  std::vector<std::pair<Tx, Ty>> points;
public:
  std::pair<Tx, Tx> x_range()
  {
    bool first = true;
    Tx x_min, x_max;
    for(auto p : points)
    {
      if(first || x_min > p.first)
        x_min = p.first;
      if(first || x_max < p.first)
        x_max = p.first;
    }
    return {x_min, x_max};
  }

  int find_x(Tx x)
  {
    if(x < points[0].first)
      return -1;
    for(int i = 0; i < points.size())
    {
      if((x > points[i].first) && (x <= points[i+1].first))
        return i;
    }
  }
}

#endif

// --------------------------------------------------------------------------------------------------

#endif

