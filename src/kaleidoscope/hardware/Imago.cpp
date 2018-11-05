/* -*- mode: c++ -*-
 * Kaleidoscope-Hardware-Imago -- Keyboardio Imago hardware support for Kaleidoscope
 * Copyright (C) 2018  Keyboard.io, Inc
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU General Public License as 
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef ARDUINO_AVR_IMAGO

#include <Kaleidoscope.h>
#include <KeyboardioHID.h>
#include <avr/wdt.h>

static bool do_scan_ = true;

ISR(TIMER1_OVF_vect) {
  do_scan_ = true;
}

namespace kaleidoscope {
namespace hardware {

uint16_t Imago::previousKeyState_[matrix_rows] = {0} ;
uint16_t Imago::keyState_[matrix_rows] = {0};
uint16_t Imago::masks_[matrix_rows];

uint8_t Imago::debounce_matrix_[matrix_rows][matrix_columns];
uint8_t Imago::debounce = 3;

#define PORT_ROWS PORTF
#define DDR_ROWS DDRF
#define PIN_ROWS PINF


#define PIN_ROW0 _BV(6)
#define PIN_ROW1 _BV(5)
#define PIN_ROW2 _BV(4)
#define PIN_ROW3 _BV(1)
#define PIN_ROW4 _BV(0)

static constexpr uint8_t row_pins[] = {_BV(6),_BV(5), _BV(4), _BV(1), _BV(0)};

#define ROW_PINS (PIN_ROW0| PIN_ROW1| PIN_ROW2| PIN_ROW3| PIN_ROW4)


/* For future use
 static constexpr colpins[matrix_columns] =    {PINB,PINB,PINE,PINC,PINC,PINB,PINB,PINB,PIND,PIND,PIND,PIND,PIND,PIND,PINE,PINF};
static constexpr colpinbits[matrix_columns] = {2,7,2,7,6,6,5,4,7,6,4,5,3,2,6,7};
 */

#define COLPINS_PORTB ( _BV(2)|_BV(4)|_BV(5)|_BV(6)|_BV(7))
#define COLPINS_PORTC (_BV(6)|_BV(7))
#define COLPINS_PORTD (_BV(2)| _BV(3)|_BV(4)|_BV(5)|_BV(6)|_BV(7))
#define COLPINS_PORTE (_BV(2)|_BV(6))
#define COLPINS_PORTF (_BV(7))

#define PIN_COL0 PINB
#define PINBIT_COL0 _BV(2)

#define PIN_COL1 PINB
#define PINBIT_COL1 _BV(7)

#define PIN_COL2 PINE
#define PINBIT_COL2 _BV(2)

#define PIN_COL3 PINC
#define PINBIT_COL3 _BV(7)

#define PIN_COL4 PINC
#define PINBIT_COL4 _BV(6)

#define PIN_COL5 PINB
#define PINBIT_COL5 _BV(6)

#define PIN_COL6 PINB
#define PINBIT_COL6 _BV(5)

#define PIN_COL7 PINB
#define PINBIT_COL7 _BV(4)

#define PIN_COL8 PIND
#define PINBIT_COL8 _BV(7)

#define PIN_COL9 PIND
#define PINBIT_COL9 _BV(6)

#define PIN_COL10 PIND
#define PINBIT_COL10 _BV(4)

#define PIN_COL11 PIND
#define PINBIT_COL11 _BV(5)

#define PIN_COL12 PIND
#define PINBIT_COL12 _BV(3)

#define PIN_COL13 PIND
#define PINBIT_COL13 _BV(2)

#define PIN_COL14 PINE
#define PINBIT_COL14 _BV(6)

#define PIN_COL15 PINF
#define PINBIT_COL15 _BV(7)


void Imago::setup(void) {
  wdt_disable();
  delay(100);

  PORT_ROWS &= ~(ROW_PINS);

  DDR_ROWS |= ROW_PINS;

  // Initialize columns
  DDRB &= ~(COLPINS_PORTB);
  PORTB |= (COLPINS_PORTB);

  DDRC &= ~(COLPINS_PORTC);
  PORTC |= (COLPINS_PORTC);

  DDRD &= ~(COLPINS_PORTD);
  PORTD |= (COLPINS_PORTD);
  
  DDRE &= ~(COLPINS_PORTE);
  PORTE |= (COLPINS_PORTE);

  DDRF &= ~(COLPINS_PORTF);
  PORTF |= (COLPINS_PORTF);

/* Set up Timer1 for 500usec */
  TCCR1B = _BV(WGM13);
  TCCR1A = 0;

  const uint32_t cycles = (F_CPU / 2000000) * 500;

  ICR1 = cycles;
  TCCR1B = _BV(WGM13) | _BV(CS10);
  TIMSK1 = _BV(TOIE1);
}

void Imago::toggleRow(uint8_t row) {
   PORT_ROWS ^= row_pins[row];
}

uint16_t Imago::readCols() {
  return 
(PIN_COL0 & (PINBIT_COL0) ? 0 : (_BV(0))) |
(PIN_COL1 & (PINBIT_COL1) ? 0 : (_BV(1))) |  
(PIN_COL2 & (PINBIT_COL2) ? 0 : (_BV(2))) |  
(PIN_COL3 & (PINBIT_COL3) ? 0 : (_BV(3))) |  
(PIN_COL4 & (PINBIT_COL4) ? 0 : (_BV(4))) |  
(PIN_COL5 & (PINBIT_COL5) ? 0 : (_BV(5))) |  
(PIN_COL6 & (PINBIT_COL6) ? 0 : (_BV(6))) |  
(PIN_COL7 & (PINBIT_COL7) ? 0 : (_BV(7))) |  
(PIN_COL8 & (PINBIT_COL8) ? 0 : (_BV(8))) |  
(PIN_COL9 & (PINBIT_COL9) ? 0 : (_BV(9))) |  
(PIN_COL10 & (PINBIT_COL10) ? 0 : (_BV(10))) |  
(PIN_COL11 & (PINBIT_COL11) ? 0 : (_BV(11))) |  
(PIN_COL12 & (PINBIT_COL12) ? 0 : (_BV(12))) |  
(PIN_COL13 & (PINBIT_COL13) ? 0 : (_BV(13))) |  
(PIN_COL14 & (PINBIT_COL14) ? 0 : (_BV(14))) |  
(PIN_COL15 & (PINBIT_COL15) ? 0 : (_BV(15)))
;

}

void Imago::readMatrixRow(uint8_t current_row) {
  uint16_t mask, cols;

  previousKeyState_[current_row] = keyState_[current_row];

  mask = debounceMaskForRow(current_row);

  toggleRow(current_row);
  cols = (readCols() & mask) | (keyState_[current_row] & ~mask);
  toggleRow(current_row);
  debounceRow(cols ^ keyState_[current_row], current_row);
  keyState_[current_row] = cols;
}

void Imago::readMatrix() {
  do_scan_ = false;
  for (uint8_t current_row = 0; current_row < matrix_rows; current_row++) {
    readMatrixRow(current_row);
  }
}

void Imago::actOnMatrixScan() {
  for (byte row = 0; row < matrix_rows; row++) {
    for (byte col = 0; col < matrix_columns; col++) {
      uint8_t keyState = (bitRead(previousKeyState_[row], col) << 0) |
                         (bitRead(keyState_[row], col) << 1);
      if (keyState) {
        handleKeyswitchEvent(Key_NoKey, row, col, keyState);
      }
    }
    previousKeyState_[row] = keyState_[row];
  }
}

void Imago::scanMatrix() {
  if (!do_scan_)
    return;

  readMatrix();
  actOnMatrixScan();
}

void Imago::maskKey(byte row, byte col) {
  if (row >= matrix_rows || col >= matrix_columns)
    return;

  bitWrite(masks_[row], col, 1);
}

void Imago::unMaskKey(byte row, byte col) {
  if (row >= matrix_rows || col >= matrix_columns)
    return;

  bitWrite(masks_[row], col, 0);
}

bool Imago::isKeyMasked(byte row, byte col) {
  if (row >= matrix_rows || col >= matrix_columns)
    return false;

  return bitRead(masks_[row], col);
}


void Imago::detachFromHost() {
  UDCON |= (1 << DETACH);
}

void Imago::attachToHost() {
  UDCON &= ~(1 << DETACH);
}

bool Imago::isKeyswitchPressed(byte row, byte col) {
  return (bitRead(keyState_[row], col) != 0);
}

bool Imago::isKeyswitchPressed(uint8_t keyIndex) {
  keyIndex--;
  return isKeyswitchPressed(keyIndex / matrix_columns, keyIndex % matrix_columns);
}

uint8_t Imago::pressedKeyswitchCount() {
  uint8_t count = 0;

  for (uint8_t r = 0; r < matrix_rows; r++) {
    count += __builtin_popcount(keyState_[r]);
  }
  return count;
}

uint16_t Imago::debounceMaskForRow(uint8_t row) {
  uint16_t result = 0;

  for (uint16_t c = 0; c < matrix_columns; ++c) {
    if (debounce_matrix_[row][c]) {
      --debounce_matrix_[row][c];
    } else {
      result |= (1 << c);
    }
  }
  return result;
}

void Imago::debounceRow(uint16_t change, uint8_t row) {
  for (uint16_t i = 0; i < matrix_columns; ++i) {
    if (change & (1 << i)) {
      debounce_matrix_[row][i] = debounce;
    }
  }
}

}
}

HARDWARE_IMPLEMENTATION KeyboardHardware;
kaleidoscope::hardware::Imago &Imago = KeyboardHardware;

#endif
