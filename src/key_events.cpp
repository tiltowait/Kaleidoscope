#include "Kaleidoscope.h"


static void press_consumer(Key mappedKey) {
  EventDispatcher::eventDispatchers().call(
    &EventDispatcher::consumerPress, Kaleidoscope.connectionMask,
    mappedKey.keyCode);
}

static void release_consumer(Key mappedKey) {
  EventDispatcher::eventDispatchers().call(
    &EventDispatcher::consumerRelease, Kaleidoscope.connectionMask,
    mappedKey.keyCode);
}

static void press_system(Key mappedKey) {
  EventDispatcher::eventDispatchers().call(&EventDispatcher::systemPress,
      Kaleidoscope.connectionMask,
      mappedKey.keyCode);
}

static void release_system(Key mappedKey) {
  EventDispatcher::eventDispatchers().call(
    &EventDispatcher::systemRelease, Kaleidoscope.connectionMask,
    mappedKey.keyCode);
}

static bool handleSyntheticKeyswitchEvent(Key mappedKey, uint8_t keyState) {
  if (mappedKey.flags & RESERVED)
    return false;

  if (!(mappedKey.flags & SYNTHETIC))
    return false;

  if (mappedKey.flags & IS_INTERNAL) {
    return false;
  } else if (mappedKey.flags & IS_CONSUMER) {
    if (keyIsPressed(keyState)) {
      press_consumer(mappedKey);
    } else if (keyWasPressed(keyState)) {
      release_consumer(mappedKey);
    }
  } else if (mappedKey.flags & IS_SYSCTL) {
    if (keyIsPressed(keyState)) {
      press_system(mappedKey);
    } else if (keyWasPressed(keyState)) {
	release_system(mappedKey);
    }
  } else if (mappedKey.flags & SWITCH_TO_KEYMAP) {
    // Should not happen, handled elsewhere.
  }

  return true;
}

static bool handleKeyswitchEventDefault(Key mappedKey, byte row, byte col, uint8_t keyState) {
  //for every newly pressed button, figure out what logical key it is and send a key down event
  // for every newly released button, figure out what logical key it is and send a key up event

  if (mappedKey.flags & SYNTHETIC) {
    handleSyntheticKeyswitchEvent(mappedKey, keyState);
  } else if (keyIsPressed(keyState)) {
    pressKey(mappedKey);
  } else if (keyToggledOff(keyState) && (keyState & INJECTED)) {
    releaseKey(mappedKey);
  }
  return true;
}

void press_key_raw(Key mappedKey) {
  EventDispatcher::eventDispatchers().call(&EventDispatcher::keyPress,
      Kaleidoscope.connectionMask,
      mappedKey.keyCode);

}
 
void pressKey(Key mappedKey) {
  if (mappedKey.flags & SHIFT_HELD) {
    press_key_raw(keyCode);
  }
  if (mappedKey.flags & CTRL_HELD) {
    press_key_raw(Key_LeftControl);
  }
  if (mappedKey.flags & LALT_HELD) {
    press_key_raw(Key_LeftAlt);
  }
  if (mappedKey.flags & RALT_HELD) {
    press_key_raw(Key_RightAlt);
  }
  if (mappedKey.flags & GUI_HELD) {
    press_key_raw(Key_LeftGui);
  }

  press_key_raw(mappedKey);
}

void release_key_raw(Key mappedKey) {
  EventDispatcher::eventDispatchers().call(&EventDispatcher::keyRelease,
      Kaleidoscope.connectionMask,
      mappedKey.keyCode);

}

void release_all_keys() {
  EventDispatcher::eventDispatchers().call(&EventDispatcher::keyReleaseAll,
      Kaleidoscope.connectionMask);
}

void releaseKey(Key mappedKey) {
  if (mappedKey.flags & SHIFT_HELD) {
    release_key_raw(Key_LeftShift);
  }
  if (mappedKey.flags & CTRL_HELD) {
    release_key_raw(Key_LeftControl);
  }
  if (mappedKey.flags & LALT_HELD) {
    release_key_raw(Key_LeftAlt);
  }
  if (mappedKey.flags & RALT_HELD) {
    release_key_raw(Key_RightAlt);
  }
  if (mappedKey.flags & GUI_HELD) {
    release_key_raw(Key_LeftGui);
  }
  release_key_raw(mappedKey);
}



void handleKeyswitchEvent(Key mappedKey, byte row, byte col, uint8_t keyState) {
  if (!(keyState & INJECTED)) {
    mappedKey = Layer.lookup(row, col);
  }
  for (byte i = 0; Kaleidoscope.eventHandlers[i] != NULL && i < HOOK_MAX; i++) {
    Kaleidoscope_::eventHandlerHook handler = Kaleidoscope.eventHandlers[i];
    mappedKey = (*handler)(mappedKey, row, col, keyState);
    if (mappedKey.raw == Key_NoKey.raw)
      return;
  }
  mappedKey = Layer.eventHandler(mappedKey, row, col, keyState);
  if (mappedKey.raw == Key_NoKey.raw)
    return;
  handleKeyswitchEventDefault(mappedKey, row, col, keyState);
}

void send_keyboard_report() {
  EventDispatcher::eventDispatchers().call(&EventDispatcher::keySendReport,
      Kaleidoscope.connectionMask);
}
