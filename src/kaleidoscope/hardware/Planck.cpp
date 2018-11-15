/* -*- mode: c++ -*-
 * Kaleidoscope-Hardware-Planck -- OLKB Planck hardware support for Kaleidoscope
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

#ifdef ARDUINO_AVR_PLANCK

#include <Kaleidoscope.h>
#include <KeyboardioHID.h>
#include <avr/wdt.h>


ISR(TIMER1_OVF_vect) {
  do_scan_ = true;
}


static uint8_t Kaleidoscope::hardware::Planck::matrix_row_pins[] = { PIN_D0, PIN_D5, PIN_B5, PIN_B6 };
static uint8_t Kaleidoscope::hardware::Planck::matrix_col_pins[] = { PIN_F1, PIN_F0, PIN_B0, PIN_C7, PIN_F4, PIN_F5, PIN_F6, PIN_F7, PIN_D4, PIN_D6, PIN_B4, PIN_D7 };
    static constexpr uint8_t Kaleidoscope::hardware::Planck::matrix_columns = sizeof(matrix_col_pins);
    static constexpr uint8_t Kaleidoscope::hardware::Planck::matrix_rows = sizeof(matrix_row_pins);
    static constexpr uint8_t Kaleidoscope::hardware::Planck::led_count = 0;

    static uint16_t Kaleidoscope::hardware::Planck::previousKeyState_[matrix_rows];
    static uint16_t Kaleidoscope::hardware::Planck::keyState_[matrix_rows];
    static uint16_t Kaleidoscope::hardware::Planck::masks_[matrix_rows];

    static uint8_t Kaleidoscope::hardware::Planck::debounce_matrix_[matrix_rows][matrix_columns];


HARDWARE_IMPLEMENTATION KeyboardHardware;
kaleidoscope::hardware::Planck &Planck = KeyboardHardware;

#endif
