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

#pragma once

#include <Arduino.h>
#include <avr/wdt.h>
#include "Kaleidoscope-HIDAdaptor-KeyboardioHID.h"
#include "kaleidoscope/macro_helpers.h"
#include "kaleidoscope/hardware/avr/pins_and_ports.h"
#include "kaleidoscope/Kaleidoscope.h"
#include "kaleidoscope/key_defs.h"
#include "kaleidoscope/key_events.h"


struct cRGB {
  uint8_t r, g, b;
};

#define CRGB(r,g,b) (cRGB){b, g, r}

static bool do_scan_ = true;


namespace kaleidoscope {
namespace hardware {

class ATMegaKeyboard {
  public:
    ATMegaKeyboard(void) {}
    static uint8_t matrix_col_pins[];
    static uint8_t matrix_row_pins[];
    static uint8_t matrix_columns;// = sizeof(matrix_col_pins);
    static uint8_t matrix_rows; // = sizeof(matrix_row_pins);
    static uint8_t led_count;
    void syncLeds(void) {}
    void setCrgbAt(uint8_t row, byte col, cRGB color) {}
    void setCrgbAt(int8_t i, cRGB crgb) {}
    cRGB getCrgbAt(int8_t i) {
        return CRGB(0, 0, 0);
    }
    int8_t getLedIndex(uint8_t row, byte col) {
        return -1;
    }

    void scanMatrix(void) {
        if (!do_scan_)
            return;

        do_scan_ = false;

        readMatrix();
        actOnMatrixScan();
    }


    void readMatrix(void) {

        for (uint8_t current_row = 0; current_row < matrix_rows; current_row++) {
            uint16_t mask, cols;

            previousKeyState_[current_row] = keyState_[current_row];

            mask = debounceMaskForRow(current_row);

            OUTPUT_TOGGLE(matrix_row_pins[current_row]);
            cols = (readCols() & mask) | (keyState_[current_row] & ~mask);
            OUTPUT_TOGGLE(matrix_row_pins[current_row]);
            debounceRow(cols ^ keyState_[current_row], current_row);
            keyState_[current_row] = cols;


        }
    }
    void actOnMatrixScan() {
        for (int8_t row = 0; row < matrix_rows; row++) {
            for (int8_t col = 0; col < matrix_columns; col++) {
                uint8_t keyState = (bitRead(previousKeyState_[row], col) << 0) |
                                   (bitRead(keyState_[row], col) << 1);
                if (keyState) {
                    handleKeyswitchEvent(Key_NoKey, row, col, keyState);
                }
            }
            previousKeyState_[row] = keyState_[row];
        }
    }
    void setup(void) {
        wdt_disable();


        for (uint8_t i = 0; i < matrix_columns; i++) {
            DDR_INPUT(matrix_col_pins[i]);
            ENABLE_PULLUP(matrix_col_pins[i]);
        }

        for (uint8_t i = 0; i < matrix_rows; i++) {
            DDR_OUTPUT(matrix_row_pins[i]);
            OUTPUT_HIGH(matrix_row_pins[i]);
        }

        /* Set up Timer1 for 500usec */
        TCCR1B = _BV(WGM13);
        TCCR1A = 0;

        const uint32_t cycles = (F_CPU / 2000000) * 500;

        ICR1 = cycles;
        TCCR1B = _BV(WGM13) | _BV(CS10);
        TIMSK1 = _BV(TOIE1);
    }

    /** Detaching from / attaching to the host.
     *
     * These two functions should detach the device from (or attach it to) the
     * host, preferably without rebooting the device. Their purpose is to allow
     * one to do some configuration inbetween, so the re-attach happens with
     * different properties. The device remains powered between these operations,
     * only the connection to the host gets severed.
     */

    void detachFromHost() {
        UDCON |= _BV(DETACH);
    }

    void attachToHost() {
        UDCON &= ~_BV(DETACH);
    }


    /* Key masking
     * -----------
     *
     * There are situations when one wants to ignore key events for a while, and
     * mask them out. These functions help do that. In isolation, they do nothing,
     * plugins and the core firmware is expected to make use of these.
     *
     * See `handleKeyswitchEvent` in the Kaleidoscope sources for a use-case.
     */


    void maskKey(byte row, byte col) {
        if (row >= matrix_rows || col >= matrix_columns)
            return;

        bitWrite(masks_[row], col, 1);
    }

    void unMaskKey(byte row, byte col) {
        if (row >= matrix_rows || col >= matrix_columns)
            return;

        bitWrite(masks_[row], col, 0);
    }

    bool isKeyMasked(byte row, byte col) {
        if (row >= matrix_rows || col >= matrix_columns)
            return false;

        return bitRead(masks_[row], col);
    }




    /** Key switch states
     *
     * These methods offer a way to peek at the key switch states, for those cases
     * where we need to deal with the state closest to the hardware. Some methods
     * offer a way to check if a key is pressed, others return the number of
     * pressed keys.
     */
    /**
     * Check if a key is pressed at a given position.
     *
     * @param row is the row the key is located at in the matrix.
     * @param col is the column the key is located at in the matrix.
     *
     * @returns true if the key is pressed, false otherwise.
     */



    bool isKeyswitchPressed(byte row, byte col) {
        return (bitRead(keyState_[row], col) != 0);
    }



    /**
     * Check if a key is pressed at a given position.
     *
     * @param keyIndex is the key index, as calculated by `keyIndex`.
     *
     * @note Key indexes start at 1, not 0!
     *
     * @returns true if the key is pressed, false otherwise.
     */
    bool isKeyswitchPressed(uint8_t keyIndex) {
        keyIndex--;
        return isKeyswitchPressed(keyIndex / matrix_columns, keyIndex % matrix_columns);
    }

    /**
     * Check the number of key switches currently pressed.
     *
     * @returns the number of keys pressed.
     */
    uint8_t pressedKeyswitchCount() {
        uint8_t count = 0;

        for (uint8_t r = 0; r < matrix_rows; r++) {
            count += __builtin_popcount(keyState_[r]);
        }
        return count;
    }


    static constexpr uint8_t debounce = 3;

  private:
    static uint16_t previousKeyState_[];
    static uint16_t keyState_[];
    static uint16_t masks_[];
    static uint8_t debounce_matrix_[][];


    static uint16_t readCols() {
        uint16_t results = 0x00 ;
        for (uint8_t i = 0; i < matrix_columns; i++) {
            results |= (!READ_PIN(matrix_col_pins[i]) << i);
        }
        return results;
    }



    uint16_t debounceMaskForRow(uint8_t row) {
        uint16_t result = 0;

        for (uint16_t c = 0; c < matrix_columns; ++c) {
            if (debounce_matrix_[row][c]) {
                --debounce_matrix_[row][c];
            } else {
                result |= _BV(c);
            }
        }
        return result;
    }

    void debounceRow(uint16_t change, uint8_t row) {
        for (uint16_t i = 0; i < matrix_columns; ++i) {
            if (change & _BV(i)) {
                debounce_matrix_[row][i] = debounce;
            }
        }
    }

};

}
}

/* To be used by the hardware implementations, `keyIndex` tells us the index of
 * a key, from which we can figure out the row and column as needed. The index
 * starts at one, so that plugins that work with a list of key indexes can use
 * zero as a sentinel. This is important, because when we initialize arrays with
 * fewer elements than the declared array size, the remaining elements will be
 * zero. We can use this to avoid having to explicitly add a sentinel in
 * user-facing code.
 */
constexpr uint8_t keyIndex(byte row, byte col) {
  return ( (row * matrix_columns) + col + 1);
}
/*
  This oneliner will generate these constexprs:

  perl -e'for($i=0;$i<6;$i++) { for ($j=0; $j<16;$j++) { print "constexpr uint8_t R${i}C${j} = keyIndex($i, $j);\n"}};'

*/

 uint8_t R0C0 = keyIndex(0, 0);
 uint8_t R0C1 = keyIndex(0, 1);
 uint8_t R0C2 = keyIndex(0, 2);
 uint8_t R0C3 = keyIndex(0, 3);
 uint8_t R0C4 = keyIndex(0, 4);
 uint8_t R0C5 = keyIndex(0, 5);
 uint8_t R0C6 = keyIndex(0, 6);
 uint8_t R0C7 = keyIndex(0, 7);
 uint8_t R0C8 = keyIndex(0, 8);
 uint8_t R0C9 = keyIndex(0, 9);
 uint8_t R0C10 = keyIndex(0, 10);
 uint8_t R0C11 = keyIndex(0, 11);
 uint8_t R0C12 = keyIndex(0, 12);
 uint8_t R0C13 = keyIndex(0, 13);
 uint8_t R0C14 = keyIndex(0, 14);
 uint8_t R0C15 = keyIndex(0, 15);
 uint8_t R0C16 = keyIndex(0, 16);
 uint8_t R0C17 = keyIndex(0, 17);
 uint8_t R0C18 = keyIndex(0, 18);
 uint8_t R0C19 = keyIndex(0, 19);
 uint8_t R1C0 = keyIndex(1, 0);
 uint8_t R1C1 = keyIndex(1, 1);
 uint8_t R1C2 = keyIndex(1, 2);
 uint8_t R1C3 = keyIndex(1, 3);
 uint8_t R1C4 = keyIndex(1, 4);
 uint8_t R1C5 = keyIndex(1, 5);
 uint8_t R1C6 = keyIndex(1, 6);
 uint8_t R1C7 = keyIndex(1, 7);
 uint8_t R1C8 = keyIndex(1, 8);
 uint8_t R1C9 = keyIndex(1, 9);
 uint8_t R1C10 = keyIndex(1, 10);
 uint8_t R1C11 = keyIndex(1, 11);
 uint8_t R1C12 = keyIndex(1, 12);
 uint8_t R1C13 = keyIndex(1, 13);
 uint8_t R1C14 = keyIndex(1, 14);
 uint8_t R1C15 = keyIndex(1, 15);
 uint8_t R1C16 = keyIndex(1, 16);
 uint8_t R1C17 = keyIndex(1, 17);
 uint8_t R1C18 = keyIndex(1, 18);
 uint8_t R1C19 = keyIndex(1, 19);
 uint8_t R2C0 = keyIndex(2, 0);
 uint8_t R2C1 = keyIndex(2, 1);
 uint8_t R2C2 = keyIndex(2, 2);
 uint8_t R2C3 = keyIndex(2, 3);
 uint8_t R2C4 = keyIndex(2, 4);
 uint8_t R2C5 = keyIndex(2, 5);
 uint8_t R2C6 = keyIndex(2, 6);
 uint8_t R2C7 = keyIndex(2, 7);
 uint8_t R2C8 = keyIndex(2, 8);
 uint8_t R2C9 = keyIndex(2, 9);
 uint8_t R2C10 = keyIndex(2, 10);
 uint8_t R2C11 = keyIndex(2, 11);
 uint8_t R2C12 = keyIndex(2, 12);
 uint8_t R2C13 = keyIndex(2, 13);
 uint8_t R2C14 = keyIndex(2, 14);
 uint8_t R2C15 = keyIndex(2, 15);
 uint8_t R2C16 = keyIndex(2, 16);
 uint8_t R2C17 = keyIndex(2, 17);
 uint8_t R2C18 = keyIndex(2, 18);
 uint8_t R2C19 = keyIndex(2, 19);
 uint8_t R3C0 = keyIndex(3, 0);
 uint8_t R3C1 = keyIndex(3, 1);
 uint8_t R3C2 = keyIndex(3, 2);
 uint8_t R3C3 = keyIndex(3, 3);
 uint8_t R3C4 = keyIndex(3, 4);
 uint8_t R3C5 = keyIndex(3, 5);
 uint8_t R3C6 = keyIndex(3, 6);
 uint8_t R3C7 = keyIndex(3, 7);
 uint8_t R3C8 = keyIndex(3, 8);
 uint8_t R3C9 = keyIndex(3, 9);
 uint8_t R3C10 = keyIndex(3, 10);
 uint8_t R3C11 = keyIndex(3, 11);
 uint8_t R3C12 = keyIndex(3, 12);
 uint8_t R3C13 = keyIndex(3, 13);
 uint8_t R3C14 = keyIndex(3, 14);
 uint8_t R3C15 = keyIndex(3, 15);
 uint8_t R3C16 = keyIndex(3, 16);
 uint8_t R3C17 = keyIndex(3, 17);
 uint8_t R3C18 = keyIndex(3, 18);
 uint8_t R3C19 = keyIndex(3, 19);
 uint8_t R4C0 = keyIndex(4, 0);
 uint8_t R4C1 = keyIndex(4, 1);
 uint8_t R4C2 = keyIndex(4, 2);
 uint8_t R4C3 = keyIndex(4, 3);
 uint8_t R4C4 = keyIndex(4, 4);
 uint8_t R4C5 = keyIndex(4, 5);
 uint8_t R4C6 = keyIndex(4, 6);
 uint8_t R4C7 = keyIndex(4, 7);
 uint8_t R4C8 = keyIndex(4, 8);
 uint8_t R4C9 = keyIndex(4, 9);
 uint8_t R4C10 = keyIndex(4, 10);
 uint8_t R4C11 = keyIndex(4, 11);
 uint8_t R4C12 = keyIndex(4, 12);
 uint8_t R4C13 = keyIndex(4, 13);
 uint8_t R4C14 = keyIndex(4, 14);
 uint8_t R4C15 = keyIndex(4, 15);
 uint8_t R4C16 = keyIndex(4, 16);
 uint8_t R4C17 = keyIndex(4, 17);
 uint8_t R4C18 = keyIndex(4, 18);
 uint8_t R4C19 = keyIndex(4, 19);
 uint8_t R5C0 = keyIndex(5, 0);
 uint8_t R5C1 = keyIndex(5, 1);
 uint8_t R5C2 = keyIndex(5, 2);
 uint8_t R5C3 = keyIndex(5, 3);
 uint8_t R5C4 = keyIndex(5, 4);
 uint8_t R5C5 = keyIndex(5, 5);
 uint8_t R5C6 = keyIndex(5, 6);
 uint8_t R5C7 = keyIndex(5, 7);
 uint8_t R5C8 = keyIndex(5, 8);
 uint8_t R5C9 = keyIndex(5, 9);
 uint8_t R5C10 = keyIndex(5, 10);
 uint8_t R5C11 = keyIndex(5, 11);
 uint8_t R5C12 = keyIndex(5, 12);
 uint8_t R5C13 = keyIndex(5, 13);
 uint8_t R5C14 = keyIndex(5, 14);
 uint8_t R5C15 = keyIndex(5, 15);
 uint8_t R5C16 = keyIndex(5, 16);
 uint8_t R5C17 = keyIndex(5, 17);
 uint8_t R5C18 = keyIndex(5, 18);
 uint8_t R5C19 = keyIndex(5, 19);
 uint8_t R6C0 = keyIndex(6, 0);
 uint8_t R6C1 = keyIndex(6, 1);
 uint8_t R6C2 = keyIndex(6, 2);
 uint8_t R6C3 = keyIndex(6, 3);
 uint8_t R6C4 = keyIndex(6, 4);
 uint8_t R6C5 = keyIndex(6, 5);
 uint8_t R6C6 = keyIndex(6, 6);
 uint8_t R6C7 = keyIndex(6, 7);
 uint8_t R6C8 = keyIndex(6, 8);
 uint8_t R6C9 = keyIndex(6, 9);
 uint8_t R6C10 = keyIndex(6, 10);
 uint8_t R6C11 = keyIndex(6, 11);
 uint8_t R6C12 = keyIndex(6, 12);
 uint8_t R6C13 = keyIndex(6, 13);
 uint8_t R6C14 = keyIndex(6, 14);
 uint8_t R6C15 = keyIndex(6, 15);
 uint8_t R6C16 = keyIndex(6, 16);
 uint8_t R6C17 = keyIndex(6, 17);
 uint8_t R6C18 = keyIndex(6, 18);
 uint8_t R6C19 = keyIndex(6, 19);
 uint8_t R7C0 = keyIndex(7, 0);
 uint8_t R7C1 = keyIndex(7, 1);
 uint8_t R7C2 = keyIndex(7, 2);
 uint8_t R7C3 = keyIndex(7, 3);
 uint8_t R7C4 = keyIndex(7, 4);
 uint8_t R7C5 = keyIndex(7, 5);
 uint8_t R7C6 = keyIndex(7, 6);
 uint8_t R7C7 = keyIndex(7, 7);
 uint8_t R7C8 = keyIndex(7, 8);
 uint8_t R7C9 = keyIndex(7, 9);
 uint8_t R7C10 = keyIndex(7, 10);
 uint8_t R7C11 = keyIndex(7, 11);
 uint8_t R7C12 = keyIndex(7, 12);
 uint8_t R7C13 = keyIndex(7, 13);
 uint8_t R7C14 = keyIndex(7, 14);
 uint8_t R7C15 = keyIndex(7, 15);
 uint8_t R7C16 = keyIndex(7, 16);
 uint8_t R7C17 = keyIndex(7, 17);
 uint8_t R7C18 = keyIndex(7, 18);
 uint8_t R7C19 = keyIndex(7, 19);
 uint8_t R8C0 = keyIndex(8, 0);
 uint8_t R8C1 = keyIndex(8, 1);
 uint8_t R8C2 = keyIndex(8, 2);
 uint8_t R8C3 = keyIndex(8, 3);
 uint8_t R8C4 = keyIndex(8, 4);
 uint8_t R8C5 = keyIndex(8, 5);
 uint8_t R8C6 = keyIndex(8, 6);
 uint8_t R8C7 = keyIndex(8, 7);
 uint8_t R8C8 = keyIndex(8, 8);
 uint8_t R8C9 = keyIndex(8, 9);
 uint8_t R8C10 = keyIndex(8, 10);
 uint8_t R8C11 = keyIndex(8, 11);
 uint8_t R8C12 = keyIndex(8, 12);
 uint8_t R8C13 = keyIndex(8, 13);
 uint8_t R8C14 = keyIndex(8, 14);
 uint8_t R8C15 = keyIndex(8, 15);
 uint8_t R8C16 = keyIndex(8, 16);
 uint8_t R8C17 = keyIndex(8, 17);
 uint8_t R8C18 = keyIndex(8, 18);
 uint8_t R8C19 = keyIndex(8, 19);
 uint8_t R9C0 = keyIndex(9, 0);
 uint8_t R9C1 = keyIndex(9, 1);
 uint8_t R9C2 = keyIndex(9, 2);
 uint8_t R9C3 = keyIndex(9, 3);
 uint8_t R9C4 = keyIndex(9, 4);
 uint8_t R9C5 = keyIndex(9, 5);
 uint8_t R9C6 = keyIndex(9, 6);
 uint8_t R9C7 = keyIndex(9, 7);
 uint8_t R9C8 = keyIndex(9, 8);
 uint8_t R9C9 = keyIndex(9, 9);
 uint8_t R9C10 = keyIndex(9, 10);
 uint8_t R9C11 = keyIndex(9, 11);
 uint8_t R9C12 = keyIndex(9, 12);
 uint8_t R9C13 = keyIndex(9, 13);
 uint8_t R9C14 = keyIndex(9, 14);
 uint8_t R9C15 = keyIndex(9, 15);
 uint8_t R9C16 = keyIndex(9, 16);
 uint8_t R9C17 = keyIndex(9, 17);
 uint8_t R9C18 = keyIndex(9, 18);
 uint8_t R9C19 = keyIndex(9, 19);
 uint8_t R10C0 = keyIndex(10, 0);
 uint8_t R10C1 = keyIndex(10, 1);
 uint8_t R10C2 = keyIndex(10, 2);
 uint8_t R10C3 = keyIndex(10, 3);
 uint8_t R10C4 = keyIndex(10, 4);
 uint8_t R10C5 = keyIndex(10, 5);
 uint8_t R10C6 = keyIndex(10, 6);
 uint8_t R10C7 = keyIndex(10, 7);
 uint8_t R10C8 = keyIndex(10, 8);
 uint8_t R10C9 = keyIndex(10, 9);
 uint8_t R10C10 = keyIndex(10, 10);
 uint8_t R10C11 = keyIndex(10, 11);
 uint8_t R10C12 = keyIndex(10, 12);
 uint8_t R10C13 = keyIndex(10, 13);
 uint8_t R10C14 = keyIndex(10, 14);
 uint8_t R10C15 = keyIndex(10, 15);
 uint8_t R10C16 = keyIndex(10, 16);
 uint8_t R10C17 = keyIndex(10, 17);
 uint8_t R10C18 = keyIndex(10, 18);
 uint8_t R10C19 = keyIndex(10, 19);
 uint8_t R11C0 = keyIndex(11, 0);
 uint8_t R11C1 = keyIndex(11, 1);
 uint8_t R11C2 = keyIndex(11, 2);
 uint8_t R11C3 = keyIndex(11, 3);
 uint8_t R11C4 = keyIndex(11, 4);
 uint8_t R11C5 = keyIndex(11, 5);
 uint8_t R11C6 = keyIndex(11, 6);
 uint8_t R11C7 = keyIndex(11, 7);
 uint8_t R11C8 = keyIndex(11, 8);
 uint8_t R11C9 = keyIndex(11, 9);
 uint8_t R11C10 = keyIndex(11, 10);
 uint8_t R11C11 = keyIndex(11, 11);
 uint8_t R11C12 = keyIndex(11, 12);
 uint8_t R11C13 = keyIndex(11, 13);
 uint8_t R11C14 = keyIndex(11, 14);
 uint8_t R11C15 = keyIndex(11, 15);
 uint8_t R11C16 = keyIndex(11, 16);
 uint8_t R11C17 = keyIndex(11, 17);
 uint8_t R11C18 = keyIndex(11, 18);
 uint8_t R11C19 = keyIndex(11, 19);
 uint8_t R12C0 = keyIndex(12, 0);
 uint8_t R12C1 = keyIndex(12, 1);
 uint8_t R12C2 = keyIndex(12, 2);
 uint8_t R12C3 = keyIndex(12, 3);
 uint8_t R12C4 = keyIndex(12, 4);
 uint8_t R12C5 = keyIndex(12, 5);
 uint8_t R12C6 = keyIndex(12, 6);
 uint8_t R12C7 = keyIndex(12, 7);
 uint8_t R12C8 = keyIndex(12, 8);
 uint8_t R12C9 = keyIndex(12, 9);
 uint8_t R12C10 = keyIndex(12, 10);
 uint8_t R12C11 = keyIndex(12, 11);
 uint8_t R12C12 = keyIndex(12, 12);
 uint8_t R12C13 = keyIndex(12, 13);
 uint8_t R12C14 = keyIndex(12, 14);
 uint8_t R12C15 = keyIndex(12, 15);
 uint8_t R12C16 = keyIndex(12, 16);
 uint8_t R12C17 = keyIndex(12, 17);
 uint8_t R12C18 = keyIndex(12, 18);
 uint8_t R12C19 = keyIndex(12, 19);
 uint8_t R13C0 = keyIndex(13, 0);
 uint8_t R13C1 = keyIndex(13, 1);
 uint8_t R13C2 = keyIndex(13, 2);
 uint8_t R13C3 = keyIndex(13, 3);
 uint8_t R13C4 = keyIndex(13, 4);
 uint8_t R13C5 = keyIndex(13, 5);
 uint8_t R13C6 = keyIndex(13, 6);
 uint8_t R13C7 = keyIndex(13, 7);
 uint8_t R13C8 = keyIndex(13, 8);
 uint8_t R13C9 = keyIndex(13, 9);
 uint8_t R13C10 = keyIndex(13, 10);
 uint8_t R13C11 = keyIndex(13, 11);
 uint8_t R13C12 = keyIndex(13, 12);
 uint8_t R13C13 = keyIndex(13, 13);
 uint8_t R13C14 = keyIndex(13, 14);
 uint8_t R13C15 = keyIndex(13, 15);
 uint8_t R13C16 = keyIndex(13, 16);
 uint8_t R13C17 = keyIndex(13, 17);
 uint8_t R13C18 = keyIndex(13, 18);
 uint8_t R13C19 = keyIndex(13, 19);
 uint8_t R14C0 = keyIndex(14, 0);
 uint8_t R14C1 = keyIndex(14, 1);
 uint8_t R14C2 = keyIndex(14, 2);
 uint8_t R14C3 = keyIndex(14, 3);
 uint8_t R14C4 = keyIndex(14, 4);
 uint8_t R14C5 = keyIndex(14, 5);
 uint8_t R14C6 = keyIndex(14, 6);
 uint8_t R14C7 = keyIndex(14, 7);
 uint8_t R14C8 = keyIndex(14, 8);
 uint8_t R14C9 = keyIndex(14, 9);
 uint8_t R14C10 = keyIndex(14, 10);
 uint8_t R14C11 = keyIndex(14, 11);
 uint8_t R14C12 = keyIndex(14, 12);
 uint8_t R14C13 = keyIndex(14, 13);
 uint8_t R14C14 = keyIndex(14, 14);
 uint8_t R14C15 = keyIndex(14, 15);
 uint8_t R14C16 = keyIndex(14, 16);
 uint8_t R14C17 = keyIndex(14, 17);
 uint8_t R14C18 = keyIndex(14, 18);
 uint8_t R14C19 = keyIndex(14, 19);
 uint8_t R15C0 = keyIndex(15, 0);
 uint8_t R15C1 = keyIndex(15, 1);
 uint8_t R15C2 = keyIndex(15, 2);
 uint8_t R15C3 = keyIndex(15, 3);
 uint8_t R15C4 = keyIndex(15, 4);
 uint8_t R15C5 = keyIndex(15, 5);
 uint8_t R15C6 = keyIndex(15, 6);
 uint8_t R15C7 = keyIndex(15, 7);
 uint8_t R15C8 = keyIndex(15, 8);
 uint8_t R15C9 = keyIndex(15, 9);
 uint8_t R15C10 = keyIndex(15, 10);
 uint8_t R15C11 = keyIndex(15, 11);
 uint8_t R15C12 = keyIndex(15, 12);
 uint8_t R15C13 = keyIndex(15, 13);
 uint8_t R15C14 = keyIndex(15, 14);
 uint8_t R15C15 = keyIndex(15, 15);
 uint8_t R15C16 = keyIndex(15, 16);
 uint8_t R15C17 = keyIndex(15, 17);
 uint8_t R15C18 = keyIndex(15, 18);
 uint8_t R15C19 = keyIndex(15, 19);
 uint8_t R16C0 = keyIndex(16, 0);
 uint8_t R16C1 = keyIndex(16, 1);
 uint8_t R16C2 = keyIndex(16, 2);
 uint8_t R16C3 = keyIndex(16, 3);
 uint8_t R16C4 = keyIndex(16, 4);
 uint8_t R16C5 = keyIndex(16, 5);
 uint8_t R16C6 = keyIndex(16, 6);
 uint8_t R16C7 = keyIndex(16, 7);
 uint8_t R16C8 = keyIndex(16, 8);
 uint8_t R16C9 = keyIndex(16, 9);
 uint8_t R16C10 = keyIndex(16, 10);
 uint8_t R16C11 = keyIndex(16, 11);
 uint8_t R16C12 = keyIndex(16, 12);
 uint8_t R16C13 = keyIndex(16, 13);
 uint8_t R16C14 = keyIndex(16, 14);
 uint8_t R16C15 = keyIndex(16, 15);
 uint8_t R16C16 = keyIndex(16, 16);
 uint8_t R16C17 = keyIndex(16, 17);
 uint8_t R16C18 = keyIndex(16, 18);
 uint8_t R16C19 = keyIndex(16, 19);
 uint8_t R17C0 = keyIndex(17, 0);
 uint8_t R17C1 = keyIndex(17, 1);
 uint8_t R17C2 = keyIndex(17, 2);
 uint8_t R17C3 = keyIndex(17, 3);
 uint8_t R17C4 = keyIndex(17, 4);
 uint8_t R17C5 = keyIndex(17, 5);
 uint8_t R17C6 = keyIndex(17, 6);
 uint8_t R17C7 = keyIndex(17, 7);
 uint8_t R17C8 = keyIndex(17, 8);
 uint8_t R17C9 = keyIndex(17, 9);
 uint8_t R17C10 = keyIndex(17, 10);
 uint8_t R17C11 = keyIndex(17, 11);
 uint8_t R17C12 = keyIndex(17, 12);
 uint8_t R17C13 = keyIndex(17, 13);
 uint8_t R17C14 = keyIndex(17, 14);
 uint8_t R17C15 = keyIndex(17, 15);
 uint8_t R17C16 = keyIndex(17, 16);
 uint8_t R17C17 = keyIndex(17, 17);
 uint8_t R17C18 = keyIndex(17, 18);
 uint8_t R17C19 = keyIndex(17, 19);
 uint8_t R18C0 = keyIndex(18, 0);
 uint8_t R18C1 = keyIndex(18, 1);
 uint8_t R18C2 = keyIndex(18, 2);
 uint8_t R18C3 = keyIndex(18, 3);
 uint8_t R18C4 = keyIndex(18, 4);
 uint8_t R18C5 = keyIndex(18, 5);
 uint8_t R18C6 = keyIndex(18, 6);
 uint8_t R18C7 = keyIndex(18, 7);
 uint8_t R18C8 = keyIndex(18, 8);
 uint8_t R18C9 = keyIndex(18, 9);
 uint8_t R18C10 = keyIndex(18, 10);
 uint8_t R18C11 = keyIndex(18, 11);
 uint8_t R18C12 = keyIndex(18, 12);
 uint8_t R18C13 = keyIndex(18, 13);
 uint8_t R18C14 = keyIndex(18, 14);
 uint8_t R18C15 = keyIndex(18, 15);
 uint8_t R18C16 = keyIndex(18, 16);
 uint8_t R18C17 = keyIndex(18, 17);
 uint8_t R18C18 = keyIndex(18, 18);
 uint8_t R18C19 = keyIndex(18, 19);
 uint8_t R19C0 = keyIndex(19, 0);
 uint8_t R19C1 = keyIndex(19, 1);
 uint8_t R19C2 = keyIndex(19, 2);
 uint8_t R19C3 = keyIndex(19, 3);
 uint8_t R19C4 = keyIndex(19, 4);
 uint8_t R19C5 = keyIndex(19, 5);
 uint8_t R19C6 = keyIndex(19, 6);
 uint8_t R19C7 = keyIndex(19, 7);
 uint8_t R19C8 = keyIndex(19, 8);
 uint8_t R19C9 = keyIndex(19, 9);
 uint8_t R19C10 = keyIndex(19, 10);
 uint8_t R19C11 = keyIndex(19, 11);
 uint8_t R19C12 = keyIndex(19, 12);
 uint8_t R19C13 = keyIndex(19, 13);
 uint8_t R19C14 = keyIndex(19, 14);
 uint8_t R19C15 = keyIndex(19, 15);
 uint8_t R19C16 = keyIndex(19, 16);
 uint8_t R19C17 = keyIndex(19, 17);
 uint8_t R19C18 = keyIndex(19, 18);
 uint8_t R19C19 = keyIndex(19, 19);
