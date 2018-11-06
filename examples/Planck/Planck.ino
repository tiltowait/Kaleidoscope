/* -*- mode: c++ -*-
 * Imago -- A very basic Kaleidoscope example for the Keyboardio Imago
 * Copyright (C) 2018  Keyboard.io, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "Kaleidoscope.h"
#include "Kaleidoscope-Macros.h"

/* *INDENT-OFF* */
KEYMAPS( [0] = KEYMAP( 
Key_Escape,		Key_Q,		Key_W,		Key_E,		Key_R,	Key_T,	Key_Y,	Key_U,	Key_I,		Key_O,		Key_P,		Key_Delete,
Key_A,			Key_S,		Key_D,		Key_F,		Key_G,	Key_H,	Key_J,	Key_K,	Key_L,		Key_Semicolon,	Key_Quote,	Key_Enter,
Key_LeftShift,		Key_Z,		Key_X,		Key_C,		Key_V,	Key_B,	Key_N,	Key_M,	Key_Comma,	Key_Period,	Key_Slash,	Key_RightShift,	
Key_LeftControl,	Key_LeftGui,	Key_LeftAlt,  Key_Space,	Key_Delete,	Key_Space,	Key_RightAlt,	Key_Menu,	Key_RightControl, Key_Space, Key_Space, Key_Space ) )
/* *INDENT-ON* */

KALEIDOSCOPE_INIT_PLUGINS(Macros);

void setup() {
  Kaleidoscope.setup();
Serial.begin(9600);
}

void loop() {
  Kaleidoscope.loop();
}
