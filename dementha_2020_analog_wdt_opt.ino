// use 1mhz compiled (leave whatever the default clock speed is, which I am to believe is 8mhz prescaled to 1mhz)
// but should lower BOD voltage
// and enable BOD in the sleep code


#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#define P0 7 // PA7
#define P1 6 // PA6
#define P2 5 // PA5
#define P3 4 // PA4
#define P4 3 // physical 2, PA3
#define P5 2 // physical 3, PA2
#define P6 1 // physical 4, PA1
#define P7 0 // physical 5, PA0

#define NUM_LEDS 39

// can pack this into less memory. since each is only 4 bits.
typedef struct Note {
  const uint8_t cathode: 4;
    const uint8_t anode: 4;
  } Note;


  const Note LED[NUM_LEDS] = {
  // top
  {P7, P6},
  {P7, P5},
  {P5, P7},
  {P3, P7},
  {P7, P3},
  {P1, P3},
  {P3, P1},
  {P1, P4},
  {P4, P1},
  {P1, P0},
  {P2, P4},
  {P4, P2},
  {P1, P2},
  {P2, P1},


  {P2, P3},

  {P3, P2},
  {P2, P0},
  {P0, P2},
  {P3, P0},

  // tip
  {P0, P3},

  // bottom
  {P4, P0},
  {P0, P4},
  {P3, P4},
  {P4, P3},
  {P3, P5},
  {P5, P3},
  {P4, P5},
  {P5, P4},
  {P4, P6},
  {P6, P4},
  {P5, P6},
  {P0, P5},
  {P5, P0},
  {P6, P5},
  {P0, P6},
  {P6, P0},
  {P0, P7},
  {P7, P0},
  {P6, P7},
};




//#define BRIGHTNESS_SIZE NUM_LEDS

//uint8_t brightness[BRIGHTNESS_SIZE] = { 0 };

#define BRIGHTNESS_SIZE ((NUM_LEDS / 2) + (NUM_LEDS & 1))

uint8_t brightness[BRIGHTNESS_SIZE] = { 0 };

void setBrightness(uint8_t pix, uint8_t new_brightness) {
  uint8_t& old_brightness = brightness[pix >> 1];
  if (pix & 1) {
    old_brightness = (old_brightness & 0b11110000) | (new_brightness & 0b1111);
  } else {
    old_brightness = (old_brightness & 0b00001111) | (new_brightness << 4);
  }
}

uint8_t getBrightness(uint8_t pix) {
  if (pix & 1) {
    return brightness[pix >> 1] & 0b1111;
  } else {
    return brightness[pix >> 1] >> 4;
  }
}


void resetBrightness() {
  memset(brightness, 0, BRIGHTNESS_SIZE * sizeof(brightness[0])); // is this or can this be precompiled?
}



const uint8_t INT0_LOW = 0;
const uint8_t INT0_CHANGE = bit(ISC00);
const uint8_t INT0_FALLING = bit(ISC01);
const uint8_t INT0_RISING = bit(ISC00) | bit(ISC01);
const uint8_t INT0_CLEAR = bit(ISC00) | bit(ISC01);




EMPTY_INTERRUPT(WDT_vect);

EMPTY_INTERRUPT(INT0_vect);

//volatile bool gotTap = false;
//ISR(INT0_vect) {
//  cbi(GIMSK, INT0);
//  gotTap = true;
//}

#define NO_WATCHDOG -1


#define BODCR _SFR_IO8(0x30)
#define BODSE 0
#define BODS 1

void Sleep_now(int8_t timeout) {
  DDRA = 0;
  PORTA = 0xff;

  cli();
  wdt_reset();                  // reset watchdog timer

  if (timeout != NO_WATCHDOG) {
    MCUSR &= ~(1 << WDRF);        // clear reset flag
    WDTCSR = (1 << WDE) | (1 << WDCE); // enable watchdog
    WDTCSR = (1 << WDIE) | timeout; // watchdog interrupt instead of reset
    //+reset, timeout can be 15,30,60,120,250,500ms or 1,2,4,8s
  }

  setInt0Mode(INT0_LOW);
  //  if ((PINB & _BV(PINB2)) == HIGH) {
  //  sbi(GIMSK, INT0);                      //enable INT0
  //  }


  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // set type of sleepmode
  power_all_disable();                  // disable unneeded loads
  wdt_reset();                          // timer should start at zero
  sleep_enable();                       // approach sleep mode
  sbi(GIMSK, INT0);
  BODCR = (1 << BODSE) | (1 << BODS); //Disable BOD, step 1
  BODCR = (1 << BODS);                //Second step

  sei();
  sleep_mode();                         // enter sleep mode (confirm)
  sleep_disable();                      // entrance point when woken up
  cbi(GIMSK, INT0);
  power_all_enable();                   // re-enable the loads
}




void glimmer(uint8_t current) {
  resetBrightness();

  if (current < 5) {

    brightness[0] = brightness[15] = brightness[24] = brightness[39] = (current & 1) ? 15 : 5;

  } else {
    current -= 5;

    uint16_t colCurrent = (current * 10) / 16;

    setOffset(current, 0, 12, 0, 15);
    setOffset(48, -colCurrent - 1, 12, 39, 47);

    setOffset(colCurrent, 15, 12, 15, 24);
    setOffset(15, -current - 1, 12, 0, 14);

    setOffset(current, 24, 12, 24, 39);
    setOffset(24, -colCurrent - 1, 12, 15, 24);

    setOffset(colCurrent, 39, 12, 39, 47);
    setOffset(39, -current - 1, 12, 24, 39);

  }

}



void vegasFanBalanced(uint8_t current) {

  setOffset(current, 0, 15, 0, NUM_LEDS - 1);
  setOffset(current, -1, 8, 0, NUM_LEDS - 1);
  setOffset(current, -2, 1, 0, NUM_LEDS - 1);
  setOffset(current, -3, 1, 0, NUM_LEDS - 1);
  setOffset(current, -4, 0, 0, NUM_LEDS - 1);

  setOffset(NUM_LEDS - 1, -((int8_t)current), 15, 0, NUM_LEDS - 1);
  setOffset(NUM_LEDS, -((int8_t)current), 8, 0, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 1, -((int8_t)current), 1, 0, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 2, -((int8_t)current), 1, 0, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 3, -((int8_t)current), 0, 0, NUM_LEDS - 1);


  //void setOffset(int8_t pix, int8_t offset, uint8_t val, uint8_t min, uint8_t max) {
  //  if (pix + offset >= min && pix + offset <= max) {
  //    brightness[pix + offset] = val;
  //  }
  //}


}


void vegasFan(uint8_t current) {
  setOffset(current, -1, 15, 0, NUM_LEDS / 2);
  setOffset(current, -2, 8, 0, NUM_LEDS / 2);
  setOffset(current, -3, 1, 0, NUM_LEDS / 2);
  setOffset(current, -4, 1, 0, NUM_LEDS / 2);
  setOffset(current, -5, 0, 0, NUM_LEDS / 2);

  setOffset(NUM_LEDS + 6, -((int8_t)current), 15, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 7, -((int8_t)current), 8, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 8, -((int8_t)current), 1, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 9, -((int8_t)current), 1, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 10, -((int8_t)current), 0, NUM_LEDS / 2, NUM_LEDS - 1);
}


void vegasFan2(uint8_t current) {
  setOffset(current, -6, 15, 0, NUM_LEDS / 2);
  setOffset(current, -7, 8, 0, NUM_LEDS / 2);
  setOffset(current, -8, 1, 0, NUM_LEDS / 2);
  setOffset(current, -9, 1, 0, NUM_LEDS / 2);
  setOffset(current, -10, 0, 0, NUM_LEDS / 2);

  setOffset(NUM_LEDS - 1, -((int8_t)current), 15, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS, -((int8_t)current), 8, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 1, -((int8_t)current), 1, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 2, -((int8_t)current), 1, NUM_LEDS / 2, NUM_LEDS - 1);
  setOffset(NUM_LEDS + 3, -((int8_t)current), 0, NUM_LEDS / 2, NUM_LEDS - 1);
}


void vegasRound(uint8_t current) {
  //  setOffset(current, 0, 15, 0, NUM_LEDS - 1);
  //  setOffset(current, -1, 8, 0, NUM_LEDS - 1);
  //  setOffset(current, -2, 1, 0, NUM_LEDS - 1);
  //  setOffset(current, -3, 1, 0, NUM_LEDS - 1);
  //  setOffset(current, -4, 0, 0, NUM_LEDS - 1);
  //
  setOffset(current, -1, 15, 0, NUM_LEDS - 1 );
  setOffset(current, -2, 8, 0, NUM_LEDS - 1 );
  setOffset(current, -3, 1, 0, NUM_LEDS - 1 );
  setOffset(current, -4, 1, 0, NUM_LEDS - 1 );
  setOffset(current, -5, 0, 0, NUM_LEDS - 1 );
}

void vegasRound2(uint8_t current) {
  setOffset(current, 0, (current & 1) ? 15 : 2, 0, NUM_LEDS - 1);
  setOffset(current, -1, (current & 1) ? 8 : 2, 0, NUM_LEDS - 1);
  setOffset(current, -2, 1, 0, NUM_LEDS - 1);
  setOffset(current, -3, (current & 1) ? 1 : 0, 0, NUM_LEDS - 1);
  setOffset(current, -4, 0, 0, NUM_LEDS - 1);
}

void currentDiagnostic(uint8_t current) {
  resetBrightness();
  for (uint8_t i = 0; i < 8; ++i) {
    setBrightness(i, (current & (1 << i)) ? 15 : 0);
  }
}



void twinkle(uint8_t current) {
  //  resetBrightness();
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    setBrightness(i, getBrightness(i) >> 3);
  }
  static uint8_t px;
  if (current == 0) {
    px = random8(NUM_LEDS - 1);
  } else {
    px += random(NUM_LEDS >> 2, (NUM_LEDS >> 2) + (NUM_LEDS >> 1));
    if (px >= NUM_LEDS) {
      px -= NUM_LEDS;
    }
  }

  uint8_t masterBright = triwave8(current << 3);
  uint8_t prevPix = (px == 0) ? NUM_LEDS - 1 : px - 1;
  uint8_t nextPix = (px == NUM_LEDS - 1) ? 0 : px + 1;

  setBrightness(px, masterBright >> 4); // scale8(cubicwave8(current * 4), 10) + 3; // random(0, 15);
  if (random8() & 1) {
    setBrightness(nextPix, masterBright >> 6);
  } else {
    setBrightness(prevPix, masterBright >> 6);
  }
  // brightness[random(0, 47)] = random(0, 15);
}



#define NUM_Q 5
#define update_per_step 1

// below are automatically global
int8_t pix_id[NUM_Q] = { -1 };
uint8_t pix_str[NUM_Q] = { 0 };


uint8_t currentPixel = 0;
uint8_t currentStrength = 0;

void sparkle(uint8_t current) {
  uint8_t i;
  static  bool alt = false;

  if (currentStrength >= 4 || current == 0) {
    //    setBrightness(currentPixel, 0);
    if (alt) {
      currentPixel = (NUM_LEDS - 1) - currentPixel;
    } else {
      currentPixel = random8(NUM_LEDS - 1);
    }
    alt = !alt;
    currentStrength = 0;


    for (i = 0; i < NUM_Q - 1; ++i) {
      pix_str[i] = pix_str[i + 1];
      pix_id[i] = pix_id[i + 1];
    }


    pix_id[NUM_Q - 1] = currentPixel;
  }

  resetBrightness();

  setBrightness(currentPixel, triwave8(currentStrength++ << 4) >> 4);

  for (i = 0; i < NUM_Q; ++i) {
    if (pix_id[i] != -1) {
      setBrightness(pix_id[i], triwave8(pix_str[i] << 4) >> 4); // quadwave8(pix_str[i]) >> 4; // also try cubic
      if (pix_str[i] <= 16) {
        pix_str[i] += 1;
      }
    }
  }
}



typedef void (*anim_tick_update_f)(uint8_t);
struct animation_t {
  const uint16_t frameEvery; // how often to call the frame update
  const anim_tick_update_f frameUpdate; // how to update the frame
  const uint16_t doneAfter; // done after this many frames
  const uint8_t onBudget; // expect for leds to be on this many us
};


struct animation_t vegas_fan_balanced_anim  = {
  .frameEvery = 1, .frameUpdate = vegasFanBalanced, .doneAfter = 40, .onBudget = (4 + 2 + 1 + 1) * 2
};

struct animation_t vegas_fan_anim  = {
  .frameEvery = 1, .frameUpdate = vegasFan, .doneAfter = 32, .onBudget = (4 + 2 + 1 + 1) * 2
};

struct animation_t vegas_fan_anim2  = {
  .frameEvery = 1, .frameUpdate = vegasFan2, .doneAfter = 20, .onBudget = (4 + 2 + 1 + 1) * 2
};

struct animation_t vegas_round_anim  = {
  .frameEvery = 1, .frameUpdate = vegasRound, .doneAfter = 54, .onBudget = 4 + 2 + 1 + 1
};

struct animation_t vegas_round_anim2  = {
  .frameEvery = 2, .frameUpdate = vegasRound2, .doneAfter = 54, .onBudget = 4 + 2 + 1 + 1
};

struct animation_t glimmer_anim  = {
  .frameEvery = 1, .frameUpdate = glimmer, .doneAfter = 24, .onBudget = 0
};

struct animation_t twinkle_anim  = {
  .frameEvery = 2, .frameUpdate = twinkle, .doneAfter = 32, .onBudget = 0
};

struct animation_t sparkle_anim  = {
  .frameEvery = 1, .frameUpdate = sparkle, .doneAfter = 64, .onBudget = 0
};


struct animation_t *anim = &vegas_round_anim;
struct animation_t *anim2 = &sparkle_anim;

uint16_t getTimeSpent(void) {
  uint16_t timeSpent = 0;

  for (uint8_t led_ix = 0; led_ix < NUM_LEDS; ++led_ix) {
    uint8_t bright = getBrightness(led_ix);
    if (bright >= 12) {
      timeSpent += 4 * bright;
    } else if (brightness[led_ix] >= 8) {
      timeSpent += 2 * bright;
    } else {
      timeSpent += 1 * bright;
    }
  }
  return timeSpent / 16;
}

void finalSparkle() {
  static uint8_t offset = 0;
  int8_t q[] = { -1, -1};
  for (uint8_t i = 0; i < 10; ++i) {
    q[0] = random8(NUM_LEDS >> 2) + offset;
    offset += NUM_LEDS >> 2;
    if (offset > NUM_LEDS) {
      offset = 0;
    }

    for (uint8_t l = 0; l < 2; ++l) {
      uint8_t p = q[l];
      if (p == -1) {
        continue;
      }
      for (uint8_t j = 24; j > 0; --j) {


        for (uint8_t k = 0; k < 24; ++k) {
          if (k > j) {
            pix(p);
          } else {
            resetPix();
          }
          _delay_us(20);
        }
        resetPix();

        //      _delay_ms(1);
      }
    }

    q[1] = 38 - q[0];
    uint8_t delay_time = random8(10) + 5;
    for (uint8_t j = 0; j < delay_time; ++j) {
      _delay_ms(1);
    }
  }
}

bool repeat(void) {
  // needed: update rate, update function, know when finished

  static uint8_t refreshCycle;
  static uint16_t cycles;
  static uint16_t frame;
  static uint16_t onTime;

  if (frame == 0) {
    anim->frameUpdate(frame++);
    onTime = getTimeSpent();
  }

  if (refreshCycle++ == 10) {
    refreshCycle = 0;
    resetPix();
    // TODO: delay for (anim->onRate - onTime)
    _delay_us(180); // reduce me for higher efficiency leds.
    // TODO: longer wait if everything wasn't lit as long as expected.
    if (++cycles == anim->frameEvery) {
      cycles = 0;
      if (frame == anim->doneAfter) {
        cycles = frame = 0;
        //        finalSparkle();
        return false;
      } else {
        anim->frameUpdate(++frame);
        onTime = getTimeSpent();
      }
    }
  }

  for (uint8_t led_ix = 0; led_ix < NUM_LEDS; ++led_ix) {
    //    uint8_t refreshIx = (refreshCycle ^ led_ix ^ 111) & 15;
    uint8_t bright = getBrightness(led_ix);
    if (bright > refreshCycle) {

      resetPix();
      _delay_us(500); // 90 looks the best but leds are too bright
      pix(led_ix);

      if (bright >= 12) {
        _delay_us(4);
      } else if (bright >= 8) {
        _delay_us(2);
      } else {
        _delay_us(1);
      }

    }
  }

  resetPix();
  _delay_us(2);

  return true;
}


inline void setInt0Mode(uint8_t mode) {
  MCUCR = (MCUCR & ~INT0_CLEAR) | mode;
}


void resetPix() {
  DDRA = 0;
  PORTA = 0;
}

void pix(uint8_t sinkPin, uint8_t sourcePin) {
  PORTA = 0;
  DDRA = (1 << sinkPin) | (1 << sourcePin);
  PORTA = (1 << sourcePin);
}

void pix(int8_t pinIx) {
  if (pinIx < 0 || pinIx > NUM_LEDS) {
    resetPix();
  } else {
    pix(LED[pinIx].cathode, LED[pinIx].anode);
  }
}

void setOffset(int8_t pix, int8_t offset, uint8_t val, uint8_t min, uint8_t max) {
  if (pix + offset >= min && pix + offset <= max) {
    setBrightness(pix + offset, val);
  }
}

inline uint8_t scale8(uint8_t i, uint8_t scale) {
  return (((uint16_t)i) * (1 + (uint16_t)(scale))) >> 8;
}

#define FASTLED_RAND16_2053  ((uint16_t)(2053))
#define FASTLED_RAND16_13849 ((uint16_t)(13849))
#define APPLY_FASTLED_RAND16_2053(x) (x << 11) + (x << 2) + x

/// random number seed
uint16_t rand16seed;// = RAND16_SEED;

/// Generate an 8-bit random number
uint8_t random8()
{
  rand16seed = APPLY_FASTLED_RAND16_2053(rand16seed) + FASTLED_RAND16_13849;
  // return the sum of the high and low bytes, for better
  //  mixing and non-sequential correlation
  return (uint8_t)(((uint8_t)(rand16seed & 0xFF)) +
                   ((uint8_t)(rand16seed >> 8)));
}

uint8_t random8(uint8_t lim) {
  uint8_t r = random8();
  r = (r * lim) >> 8;
  return r;
}

uint8_t triwave8(uint8_t in)
{
  if ( in & 0x80) {
    in = 255 - in;
  }
  uint8_t out = in << 1;
  return out;
}

uint8_t chooseSide() {
  static uint8_t lastSide = 0;
  return (lastSide ^= 1);
}

#define WATCHDOG_TIME WDTO_2S
int main(void) {

  wdt_reset();
  wdt_disable();

  bool wasReset = MCUSR & bit(WDRF);

  MCUSR = 0; // this register contains whether we reset due to watchdog, brown-out or power-on reset

  wdt_enable(WATCHDOG_TIME);


  // enable pullups
  PORTB |=
    // input pin
    _BV(PORTB2)

    // unused pins
    | _BV(PORTB1) | _BV(PORTB0);

  ADCSRA &= ~(1 << ADEN);   //turn off ADC
  ACSR |= _BV(ACD);         // disable analog comparator

  resetPix();

  uint8_t i = 0;

  while (1) {
    if (wasReset || !repeat()) {
      wdt_reset();
      wdt_disable();
      Sleep_now(NO_WATCHDOG);
      wdt_enable(WATCHDOG_TIME);

      if (!wasReset && chooseSide()) {
        anim = &twinkle_anim;
      } else {
        anim = &vegas_round_anim;
      }

      wasReset = false;
    }
  }
}
