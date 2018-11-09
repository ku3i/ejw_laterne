/* Lampe für EJW */

#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <EEPROM.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

struct RGBColor { uint8_t r, g, b; };
struct HSVColor { uint8_t h, s, v; };

namespace colors {
	const RGBColor red          = {255,  32,   0};
	const RGBColor orange       = {255,  64,   0};
	const RGBColor yellow       = {255, 128,   0};
	const RGBColor yellow_green = {128, 255,   0};
}

/* constants */
const unsigned int delay_ms = 1;
const unsigned int update_rate = 1;

const unsigned int number_of_neopixels = 17;
const unsigned int pin_neopixel = 6;
const unsigned int pin_button   = 10;
const unsigned int pin_encode_A = 11;
const unsigned int pin_encode_B = 12;
const unsigned int pin_red_led  = 13;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel( number_of_neopixels
                                            , pin_neopixel
                                            , NEO_RGB + NEO_KHZ400 );

bool mode                = true;
bool button_pressed      = false;
bool last_button_pressed = false;

unsigned int cycles  = 0;
unsigned int special = 0;

int program_set = 0;
const int MAX_PROG = 8;
unsigned program_changed = 0;
const unsigned PROG_STABLE = 10000;

const unsigned prog_addr = 42;

/* SO(2)-Oscillator */
float x1 = 0.01;
float x2, y1, y2 = 0.0;

const float w11 = +1.01;
const float w12 = +0.13;
const float w21 = -0.13;
const float w22 = +1.01;

/* for random weights */
float W0 [number_of_neopixels];
float W1 [number_of_neopixels];

float brightness = 1.0;

HSVColor multi = {0, 255, 255};
uint8_t color_wheel = 0;

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pixels.begin(); // This initializes the NeoPixel library.
  pinMode(pin_red_led, OUTPUT);

  /* rotary encoder switches */
  pinMode(pin_button,   INPUT_PULLUP); // button
  pinMode(pin_encode_A, INPUT_PULLUP); // pin_A
  pinMode(pin_encode_B, INPUT_PULLUP); // pin_B

  for (unsigned int i = 0; i < number_of_neopixels; ++i)
      pixels.setPixelColor(i, pixels.Color(64,0,64));
  pixels.show();                            // update neopixels

	randomSeed(analogRead(0));
	for (unsigned i = 0; i < number_of_neopixels; ++i)
	{
		//W0[i] = random(0,256) > 128 ? -1:1;
		//W1[i] = random(0,256) > 128 ? -1:1;
		W0[i] = (random(0,256) - 128) / 128.f;
		W1[i] = (random(0,256) - 128) / 128.f;
	}

	program_set = EEPROM.read(prog_addr);
	brightness  = EEPROM.read(prog_addr + 1) / 255.f;
	program_changed = PROG_STABLE;
}


bool check_button() {
	static int hyst = 0;
	bool val = 1 - digitalRead(pin_button);
	hyst += (val) ? 1 : -1;
	hyst = constrain(hyst, 0, 9);
	return (hyst > 4);
}
inline float map01(float value) { return constrain(0.5f * value + 0.5f, 0.f, 1.f); }

inline void diffuse_glow(unsigned index, RGBColor const& col, float in0, float in1) {
	const uint8_t r8 = col.r * brightness * map01(in0*W0[index]);
	const uint8_t g8 = col.g * brightness * map01(in1*W1[index]);
	const uint8_t b8 = col.b * brightness * map01((W0[index]*in0 + W1[index]*in1) / 2);
	pixels.setPixelColor(index, pixels.Color(r8, g8, b8));
}

void diffuse_glow_all(RGBColor const& col, float in0, float in1) {
	for (unsigned int i = 0; i < number_of_neopixels; ++i) {
		diffuse_glow(i, col, in0, in1);
	}
}

void set_all_pixels_to(float r, float g, float b)
{
	uint8_t r8 = (uint8_t) 255 * constrain(r, 0.0, 1.0) * brightness;
	uint8_t g8 = (uint8_t) 255 * constrain(g, 0.0, 1.0) * brightness;
	uint8_t b8 = (uint8_t) 255 * constrain(b, 0.0, 1.0) * brightness;

	for (unsigned int i = 0; i < number_of_neopixels; ++i)
		pixels.setPixelColor(i, pixels.Color(r8, g8, b8));
}

/* Note: this method should only be called once a cycle */
int check_rotation() {

  static unsigned int last_A = 0;
  int val = 0;

  /* read the encoder switches */
  unsigned int encode_A = digitalRead(pin_encode_A);
  unsigned int encode_B = digitalRead(pin_encode_B);

  if (!encode_A && last_A) // check if A has gone from high to low
  {
    if (encode_B) // clockwise
      val = 1;
    else // counter clockwise
      val = -1;
  }
  last_A = encode_A;      // store reading of A

  return val;
}

void update_mode()
{
	static int last_program = -1;
	static float last_brightness = brightness;
	/* read inputs */
	last_button_pressed = button_pressed;
	button_pressed = check_button();
	digitalWrite(pin_red_led, button_pressed); // set onboard LED

	if (mode) {
		last_program = program_set;
		program_set += check_rotation();
		if (program_set > MAX_PROG) program_set = 0;
		if (program_set < 0) program_set = MAX_PROG;
	} else {
		last_brightness = brightness;
		brightness -= 0.03125 * check_rotation(); // 32 steps
		brightness = constrain(brightness, 0.0, 1.0);
	}

	if (button_pressed && !last_button_pressed)
		mode = !mode;

	delay(delay_ms);

	if ((last_program != program_set) or (last_brightness != brightness))
		program_changed = 0;
	else
		program_changed += (program_changed < PROG_STABLE ) ? 1 : 0; // full after 10 sek.

	/* after 10 sec. save settings to eeprom */
	if (program_changed == PROG_STABLE-1 ) {
		EEPROM.write(prog_addr, program_set);
		EEPROM.write(prog_addr + 1, static_cast<uint8_t>(brightness*255));
		digitalWrite(pin_red_led, 1);
		delay(1000);
		digitalWrite(pin_red_led, 0);
	}
}


void loop()
{
	for (unsigned i = 0; i < 10; ++i) update_mode();

	if (cycles % update_rate == 0)
	{
		switch (program_set)
		{
			/* set new colors for neopixels */
			case 0: diffuse_glow_all(colors::red         , y1, y2); break; // red
			case 1: diffuse_glow_all(colors::orange      , y1, y2); break; // orange
			case 2: diffuse_glow_all(colors::yellow      , y1, y2); break; // yellow

			case 3: diffuse_glow_all(colors::yellow_green, y1, y2);
			        diffuse_glow(special, {255, 0, 192}, y1, y2); break; // yellow-green + violet blossom

			case 4: diffuse_glow_all( {32, 255, 0}, y1, y2); break; // green

			case 5: set_all_pixels_to( 1.00f*map01(y1), 0.00f          , 0.80f*map01(y2) ); break; // violet
			case 6: set_all_pixels_to( 0.00f          , 1.00f*map01(y1), 1.00f*map01(y2) ); break; // cyan
			case 7: set_all_pixels_to( 1.00f*map01(y1), 1.00f*map01(y2), 1.00f*map01(y2) );        // white
			        pixels.setPixelColor(special, pixels.Color(255, 255, 255)); break;

			case 8: diffuse_glow_all(HSV2RGB(multi)      , y1, y2); break; // multicolor

			default: break;
		}

		if (cycles % 100 == 0) {
			special = random(0,number_of_neopixels);
			color_wheel += 1;
			multi.h = color_wheel;
		}

		/* write outputs, update neopixels */
		pixels.show();

		if (mode) {
			y1 = tanh(w11*x1 + w12*x2);
			y2 = tanh(w21*x1 + w22*x2);
			x1 = y1;
			x2 = y2;
		}
	} /* (cycles % update_rate == 0) */

	++cycles;
}

RGBColor HSV2RGB(HSVColor hsv)
{
    RGBColor rgb;
    uint8_t region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

HSVColor RGB2HSV(RGBColor rgb)
{
    HSVColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}
