#include <EEPROM.h>
//#include <WProgram.h>
//#include <Encoder.h>
//#include <ADC.h>
#include <math.h>

#include <FastLED.h>
// @isystem "C:\Users\Victor Condino\Program Files\arduino-1.6.7\hardware\teensy\avr\cores\teensy3\*"
// why the hell fastled's header insists on leaking preprocessor macros named min and max is beyond me
#undef min
#undef max

//#include <StandardCplusplus.h>

#define INCLUDED_FROM_MAIN_INO
#include "math.hpp"
//#include "render.hpp"
//#include "ui.hpp"
//#include "firfilter.hpp"
#undef INCLUDED_FROM_MAIN_INO

#include "preferences.hpp"

// ---------------------------------------------------------------------
#if 0 //def __MK20DX256__

extern "C"
{
  int _kill(pid_t p, int s)
  {
    return -1;
  }

  int _getpid()
  {
    return -1;
  }
};

#endif

// ---------------------------------------------------------------------

float time_factor = 1.0;

double get_ms_time()
{
  static double last_actual_ms = 0.0, last_reported_ms = 0.0;
  uint32_t ms = millis();
  
  double elapsed_ms = (double)ms - (double)last_actual_ms;
  last_actual_ms = ms;
  
  last_reported_ms = (double)last_reported_ms + (double)elapsed_ms * (double)time_factor;

  return last_reported_ms;
}

// ---------------------------------------------------------------------


/**
 * @brief      Perform alpha compositing of one pixel onto another
 *
 *             Composites the color and alpha of layer 2 onto layer 1's color
 *             and alpha.
 *
 * @param[in]  src_color  The source color
 * @param[in]  src_alpha  The source alpha
 * @param      dst_color  The destination/output color
 * @param      dst_alpha  The destination/output color
 * @return     The blended result color and alpha are returned in place of the dst_color and dst_alpha values.
 */

void alphablend(const CRGB& src_color, const uint8_t& src_alpha, CRGB &dst_color, uint8_t &dst_alpha)
{
  // short-circuit trivial cases
  if(src_alpha == 0) return; // src fully transparent, leave dest unmodified
  if(src_alpha == 0xff || dst_alpha == 0x0) { dst_alpha = src_alpha; dst_color = src_color; return; } // fully opaque
  if(dst_alpha == 0xff) { dst_color = lerp(dst_color, src_color, src_alpha); return; } // dest opaque, no need to calc alpha
  // non-trivial case, fixed-point math on N-bit fractional values uses
  // 3N-bit intermediate values in the R, G and B component calculations
  uint8_t na = 0xff - dst_alpha;
  uint32_t fa = 0xff * dst_alpha, fb = na * src_alpha;
  uint32_t Ao = 0xffff - na * (0xff - src_alpha);
  CRGB Co = CRGB(
      (dst_color.r * fa + src_color.r * fb) / Ao,
      (dst_color.g * fa + src_color.g * fb) / Ao,
      (dst_color.b * fa + src_color.b * fb) / Ao
    );
  dst_color = Co;
  dst_alpha = Ao >> 8;
}

/**
 * @brief      Non-inplace (copying) version of @ref alphablend
 *
 *             When called with a third color and alpha, by reference, the src
 *             and dest color and alpha may be rvalues and the result will be
 *             returned in these last two by-reference arguments without
 *             modifying the source or dest. Note that it is safe to pass the
 *             same instance to dst and out, because dst is passed by value, and
 *             is therefore copy-constructed from the de
 *             before delegating to the in-place @ref alphablend.
 *
 * @param[in]  Csrc  The source color
 * @param[in]  Asrc  The source alpha
 * @param[in]  Cdst  The destination color
 * @param[in]  Adst  The destination alpha
 * @param      Cout  The resulting color
 * @param      Aout  The resulting alpha
 */

void alphablend(const CRGB& Csrc, const uint8_t& Asrc, CRGB Cdst, uint8_t Adst, CRGB& Cout, uint8_t &Aout)
{
  alphablend(Csrc, Asrc, Cdst, Adst);
  Aout = Adst;
  Cout = Cdst;
}

/**
 * @brief      Renders a gradient fill with alpha channel into a buffer,
 *             overlaying existing pixels
 *
 * @param[in]  c0    Starting color
 * @param[in]  a0    Starting alpha
 * @param[in]  c1    Ending color
 * @param[in]  a1    Ending alpha
 * @param[in]  x0    Starting position, fractional positions are anti-aliased
 * @param[in]  x1    Ending position
 * @param[in]  n     The size of the destination buffer
 * @param      co    The destination pixel RGB buffer
 * @param      ao    The destination pixel alpha buffer
 */

/*
[ 0 ][ 1 ][ 2 ][ 3 ][ 4 ] <-- pixels
   [        ]
0.25  1.0  0.5  2.5
gradient 0.75 to 2.5
*/

void render_gradient(CRGB c0, float a0, CRGB c1, float a1, float x0, float x1, int n, CRGB *co, uint8_t *ao)
{
  if(x0 > x1)
  {
    swap(c0, c1);
    swap(a0, a1);
    swap(x0, x1);
  }

  // if whole gradient is being rendered into less than two pixels, then adjust alpha
  float aadj = (float)1.0;
  if((float)((float)x1 - (float)x0) < (float)2.0)
  {
    aadj = (float)((float)x1 - (float)x0) / (float)2.0;
  }
  for(int i = clip<float>(floor(x0-(float)1.0), 0.0, (float)n-(float)1.0); i <= clip<float>(ceil(x1+(float)1.0), (float)0, (float)n-(float)1); i++)
  {
    if(((float)i >= x0) && ((float)i <= x1))
    {
      float u = (float)aadj * (float)((float)((float)i - (float)x0) / (float)((float)x1 - (float)x0));
      alphablend(lerp(c0, c1, u), lerp(a0, a1, u), co[i], ao[i]);
    }
    else if((float)i < x0)
    {
      float u = (float)1.0 - (float)clip<float>((float)x0 - (float)i, 0.0, 1.0);
      alphablend(c0, (int)((float)a0 * (float)u *(float)aadj), co[i], ao[i]);
    }
    else if((float)i > x1)
    {
      float u = (float)1.0 - (float)clip<float>((float)i - (float)x1, 0.0, 1.0);
      alphablend(c1, (int)((float)a1 * (float)u * (float)aadj), co[i], ao[i]);
    }
  }
}

/*
void render_gradient_spline(std::vector<std::pair<CRGB, float>> _colors, float _curve, float x0, float x1, int n, CRGB *co, uint8_t *ao)
{
  if(x0 > x1)
  {
    swap(c0, c1);
    swap(a0, a1);
    swap(x0, x1);
  }

  for(int i = clip<float>(floor(x0-(float)1.0), 0.0, (float)n-(float)1.0); i <= clip<float>(ceil(x1+(float)1.0), (float)0, (float)n-(float)1); i++)
  {
    if(((float)i >= x0) && ((float)i <= x1))
    {
      float u = (float)((float)i - (float)x0) / (float)((float)x1 - (float)x0);
      alphablend(lerp(c0, c1, u), lerp(a0, a1, u), co[i], ao[i]);
    }
    else if((float)i < x0)
    {
      float u = (float)1.0 - (float)clip<float>((float)x0 - (float)i, 0.0, 1.0);
      alphablend(c0, (int)((float)a0 * (float)u), co[i], ao[i]);
    }
    else if((float)i > x1)
    {
      float u = (float)1.0 - (float)clip<float>((float)i - (float)x1, 0.0, 1.0);
      alphablend(c1, (int)((float)a1 * (float)u), co[i], ao[i]);
    }
  }
}
*/


float ease(float x, float z)
{
  if(z > 0.0)
  {
    float p = (float)1.0/(float)z;
    //return x;
  //  if(x*2.0 > 1.0)
      return (float)1.0 - (float)pow((float)1.0 - (float)x, p);
  }
  else
  {
      return pow(x, (float)1.0/(float)-z);
  }
}

float noise(float l, float h)
{
  return (float)((float)random16() / (float)0x0ffff + (float)random16() / (float)0x0ffff0000) * (float)((float)h-(float)l) + (float)l;
}



/* ====================================================================
 * @end-file */

#define NUM_LEDS (96*3)

// led output array
CRGB leds[NUM_LEDS];
uint8_t led_alpha[NUM_LEDS];

  // pinwheel spiral parameters
  static float hue_lfo_period = (float)30.0/(float)1.0;
  static float spiral_arm_hue_offs = (float)1/(float)8;
  static float spiral_arm_hue_fade = (float)1/(float)3;
  static float spiral_period = 1.0;
  static int spiral_arms = random8() % 4 + 1;
  static float spiral_twists = (float)(random8() % 11 + 1) / 3.0;
  static float spiral_arm_sat_begin = 1.0;
  static float spiral_arm_sat_end = 0.5;
  static float val_ring_size = 0.5;
  static float val_ring_period = 0.5;
  static float val_ring_intensity = 0.1;
  static float val_ring_arm_offset = 0.5;
  static float val_ring_slope = 0.5;


prefs_cli<decltype(Serial2)> prefs_bt(Serial2, 0xdeadbeef, {
  /*preference("pin8", &get_pin_8, &set_pin_8, false, true),
  preference("baud", &get_baud_rate, &set_baud_rate, 115200, 1200, 115200, true),*/
  preference("hue_lfo_period", hue_lfo_period, 30.0, 0.1, 100.0, true),
  preference("spiral_arm_hue_offs", spiral_arm_hue_offs, 0.125, 0.0, 1.0, true),
  preference("spiral_arm_hue_fade", spiral_arm_hue_fade, 0.333, 0.0, 1.0, true),
  //preference("spiral_rate", spiral_rate, -0.001, -0.01, 0.01, true),
  preference("spiral_period", spiral_period, 1.0, 0.1, 5.0, true),
  preference("spiral_arms", spiral_arms, 3, 0, 8, true),
  preference("spiral_twists", spiral_twists, 2.0, 0.01, 15.0, true),
  preference("spiral_arm_sat_begin", spiral_arm_sat_begin, 1.0, 0.0, 1.0, true),
  preference("spiral_arm_sat_end", spiral_arm_sat_end, 0.5, 0.0, 1.0, true),
  
  preference("val_ring_size", val_ring_size, 0.5, 0.01, 2.0, true),
  preference("val_ring_period", val_ring_period, 0.5, -5.0, 5.0, true),
  preference("val_ring_intensity", val_ring_intensity, 0.1, 0.0, 1.0, true),
  preference("val_ring_arm_offset", val_ring_arm_offset, 0.1, 0.0, 1.0, true),
  preference("val_ring_slope", val_ring_slope, 0.5, 0.0, 1.0, true)

});

//prefs_cli<decltype(Serial)> prefs_usb(Serial, prefs_bt);

float stripes(float x, float phase, float size, float slope)
{
  float y = (fabs(fmod(phase*2.0 + (float)x/size, 1.0) - 0.5) * 4.0 - 1.0) / slope;
  if(y < -1.0) y = -1.0;
  if(y > 1.0) y = 1.0;
  return y/2.0 + 0.5;
}

void blink()
{
 static bool foo = false;
 foo = !foo;
 pinMode(13, OUTPUT);
 digitalWriteFast(13, foo); 
}

void loop()
{
  double ms = millis();
  static double last_ms = 0.0;
  double elapsed = ms - last_ms;
  last_ms = ms;

  prefs_bt.loop();

  float val_ring_rate = (val_ring_period != 0.0) ? ((float)0.001/(float)val_ring_period) : (0.0);
  float spiral_rate = (spiral_period != 0.0) ? ((float)0.001/(float)spiral_period) : (0.0);
  float hue_lfo_rate = (hue_lfo_period != 0.0) ? ((float)0.001/(float)hue_lfo_period) : (0.0);
  static float val_ring_pos = 0.0;
  static float spiral_pos = 0.0;
  static float hue_lfo_pos = 0.0;
  val_ring_pos = wrap(val_ring_pos + elapsed * val_ring_rate, 1.0);
  spiral_pos = wrap(spiral_pos + elapsed * spiral_rate, (float)spiral_arms*2.0);
  hue_lfo_pos = wrap(hue_lfo_pos + elapsed * hue_lfo_rate, 1.0);

  static long frames = 0;
  static long last_frames = 0;
  static const double status_interval = 1000.0;
  static double last_status = 0.0;
  frames ++;
  if(last_status + status_interval < ms)
  {
    Serial2.print("frame #");
    Serial2.print(frames, DEC);
    Serial2.print(", time ");
    Serial2.print(ms / 1000.0, 3);
    Serial2.print("s, ");
    Serial2.print((double)(frames - last_frames) / ((ms - last_status)/1000.0), 2);
    Serial2.println(" fps");
    last_status = ms;
    last_frames = frames;
  }

  // how do we compute pinwheels at 30+ fps on this little guy? lookup tables, baby

  static float theta_lut[NUM_LEDS];
  static float r_lut[NUM_LEDS];
  static bool init_luts = true;

  if(init_luts)
  {
    init_luts = false;
    for(int i = 0; i < NUM_LEDS; i++)
    {
      int xi = i % 32;
      if(((i % 96) >= 32) && ((i % 96) < 64))
        xi = 31 - xi;
      int yi = i / 32;
      float x = (float)(xi - 16) / (float)16;
      float y = (float)(yi - 4) / (float)16;

      theta_lut[i] = atan2(x, y)/std::pi();
      r_lut[i] = sqrt(x*x + y*y);
    }    
  }

  // once we've done all the square roots and arc tangents, and cached the results, the rest is basically just some lerps

  for(int i = 0; i < NUM_LEDS; i++)
  {
    float theta = theta_lut[i];
    float r = r_lut[i];

    float w = (int)wrap(2.0 * ((float)spiral_pos + (float)theta * (float)spiral_arms + (float)r * (float)spiral_twists), (float)spiral_arms*2.0) / 2;

    float val = (float)1.0 - fabs((float)wrap((float)spiral_pos + (float)theta * (float)spiral_arms + (float)r * (float)spiral_twists, 1.0) - (float)0.5) * 2.0;
    float ring_val = (float)1.0 - (float)stripes((float)r, (float)w * (float)((float)val_ring_arm_offset / (float)spiral_arms) + (float)val_ring_pos, val_ring_size, val_ring_slope) * (float)val_ring_intensity;
    val = (float)val * (float)ring_val;
    float hue = wrap((float)spiral_arm_hue_fade * (float)r + (float)hue_lfo_pos + (float)w * (float)spiral_arm_hue_offs, 1.0);
    float sat = clip((float)((float)1.0 - (float)r) * (float)spiral_arm_sat_begin + (float)((float)r * (float)spiral_arm_sat_end), 0.0, 1.0);
    
    leds[i] = CHSV(hue * (float)255.0, sat * (float)255.0, val * (float)255.0);
    //CHSV(noise(0, 255), noise(0, 32), noise(255 - 16, 255));
    led_alpha[i] = 255.0;
  }

  //static long frames = 0;
  //frames ++;
  //if(frames % 500 == 0)
    //Serial1.println(frames);

  LEDS.show();
}

/*
void particle_loop()
{
    double ms = millis();
    for(int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0,0,0);
      led_alpha[i] = 0;
    }
    psys.render(ms, NUM_LEDS, leds, led_alpha);
    LEDS.show();
}
*/

void setup()
{
  // initialize the x/y and time values
  uint16_t eeseed;
  EEPROM.get(252, eeseed);
  EEPROM.put(252, ++eeseed);

  random16_set_seed(8934 + eeseed);
  random16_add_entropy(analogRead(3));

  if(!prefs_bt.load_prefs())
  {
    prefs_bt.reset_prefs();
    prefs_bt.save_prefs();
  }

  Serial2.begin(115200);
  Serial.begin(115200);

  //pinMode(9, INPUT);
  //pinMode(10, OUTPUT);
  pinMode(8, OUTPUT);

  //pinMode(2, OUTPUT); // KEY --> HC-05/6
  //pinMode(3, INPUT);  // STATE <-- HC-05/6

  /*digitalWriteFast(2, HIGH);
  blink();delay(500);blink();delay(1500);
  Serial1.print("AT+BAUD4\r\n");
  blink();delay(500);blink();delay(1500);
  Serial1.print("AT\r\n");
  blink();delay(500);blink();delay(1500);
  digitalWriteFast(2, LOW);
  blink();delay(500);blink();delay(1500);

  Serial.begin(9600);*/
  //delay(1000);
  //Serial.println("setting up ui!");
  //delay(2000);
  
  //ui_setup();
  delay(1000);

  Serial.println("initing leds!");

  delay(1000);

  //three banks of three rows each of 32 pixels per row, with the middle row of each bank reversed
  LEDS.addLeds<WS2812,11,GRB>(&leds[96*0], 96);
  LEDS.addLeds<WS2812,14,GRB>(&leds[96*1], 96);
  LEDS.addLeds<WS2812,6,GRB>(&leds[96*2], 96);

  //shuffle_n = reboot_shuffle_setup(255);

  LEDS.setBrightness(128);
  LEDS.show();

  Serial.println("leds ok, exiting setup()");

}
