/* Copyright 2015-2021 Jack Humbert
 * Copyright 2022 ff   @einjohn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"


enum planck_layers {
  _QWERTY,
  _COLEMAK,
  _DVORAK,
  _LOWER,
  _RAISE,
  _SYMBOLS,
  _PLOVER,
  _ADJUST
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  COLEMAK,
  DVORAK,
  PLOVER,
  BACKLIT,
  EXT_PLV
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)
#define SYMBOLS MO(_SYMBOLS)

// Left-hand home row mods
#define HOME_A LSFT_T(KC_A)
#define HOME_R LCTL_T(KC_R)
#define HOME_S LGUI_T(KC_S)
#define HOME_T LALT_T(KC_T)

// Right-hand home row mods
#define HOME_N LALT_T(KC_N)
#define HOME_E RGUI_T(KC_E)
#define HOME_I RCTL_T(KC_I)
#define HOME_O RSFT_T(KC_O)

// combined non-home row mods
// cmd+shift
#define HOME_C   MT(MOD_LSFT | MOD_LGUI, KC_C)
#define HOME_COM MT(MOD_RSFT | MOD_RGUI, KC_COMM)
// alt+shift
#define HOME_D   MT(MOD_LSFT | MOD_LALT, KC_D)
#define HOME_H   MT(MOD_RSFT | MOD_LALT, KC_H)

// keymap definition
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  /* Qwerty
   * ,-----------------------------------------------------------------------------------.
   * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Esc  |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |  "   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Shift|   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |Enter |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Brite| Ctrl | Alt  | GUI  |Lower |    Space    |Raise | Left | Down |  Up  |Right |
   * `-----------------------------------------------------------------------------------'
   */
  [_QWERTY] = LAYOUT_planck_grid(
      KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSPC,
      KC_ESC,  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
      KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_ENT ,
      BACKLIT, KC_LCTL, KC_LALT, KC_LGUI, LOWER,   KC_SPC,  KC_SPC,  RAISE,   KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT
  ),

  /* Colemak Mod-DH
   * ,-----------------------------------------------------------------------------------.
   * | Tab  |   Q  |   W  |A+S/F |   P  |   B  |   J  |   L  | A+S/U|   Y  |   ;  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Esc  |SFT/A |CTL/R | OS/S |ALT/T | KC/G | KC/M |ALT/N |  OS/E| CTL/I| SFT/O| Entr |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Shft |   Z  |   X  |O+S/C |A+S/D |   V  |   K  | A+S/H| O+S/,|   .  |   /  |  "   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Lit  | Ctrl | Alt  | GUI  | Lwr  | Rse  | Smbl | Spc  | Left | Down |  Up  | Rght | <-- WIP: lwr rse shft altgr bksp del; what on layer, what a key by itself?
   * `-----------------------------------------------------------------------------------'
   */
  [_COLEMAK] = LAYOUT_planck_grid(
      KC_TAB,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_B,    KC_J,    KC_L,    KC_U,     KC_Y,    KC_SCLN, KC_BSPC,
      KC_ESC,  HOME_A,  HOME_R,  HOME_S,  HOME_T,  KC_G,    KC_M,    HOME_N,  HOME_E,   HOME_I,  HOME_O,  KC_ENT,
      KC_LSFT, KC_Z,    KC_X,    HOME_C,  HOME_D,  KC_V,    KC_K,    HOME_H,  HOME_COM, KC_DOT,  KC_SLSH, KC_QUOT,
      BACKLIT, KC_LCTL, KC_LALT, KC_LGUI, LOWER,   RAISE,   SYMBOLS, KC_SPC,  KC_LEFT,  KC_DOWN, KC_UP,   KC_RGHT
  ),

  /* Dvorak
   * ,-----------------------------------------------------------------------------------.
   * | Tab  |   "  |   ,  |   .  |   P  |   Y  |   F  |   G  |   C  |   R  |   L  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Esc  |   A  |   O  |   E  |   U  |   I  |   D  |   H  |   T  |   N  |   S  |  /   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Shift|   ;  |   Q  |   J  |   K  |   X  |   B  |   M  |   W  |   V  |   Z  |Enter |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Brite| Ctrl | Alt  | GUI  |Lower |    Space    |Raise | Left | Down |  Up  |Right |
   * `-----------------------------------------------------------------------------------'
   */
  [_DVORAK] = LAYOUT_planck_grid(
      KC_TAB,  KC_QUOT, KC_COMM, KC_DOT,  KC_P,    KC_Y,    KC_F,    KC_G,    KC_C,    KC_R,    KC_L,    KC_BSPC,
      KC_ESC,  KC_A,    KC_O,    KC_E,    KC_U,    KC_I,    KC_D,    KC_H,    KC_T,    KC_N,    KC_S,    KC_SLSH,
      KC_LSFT, KC_SCLN, KC_Q,    KC_J,    KC_K,    KC_X,    KC_B,    KC_M,    KC_W,    KC_V,    KC_Z,    KC_ENT ,
      BACKLIT, KC_LCTL, KC_LALT, KC_LGUI, LOWER,   KC_SPC,  KC_SPC,  RAISE,   KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT
  ),

  /* Lower / Navigation
   * ,-----------------------------------------------------------------------------------.
   * |   ~  |   !  |   @  |   #  |   $  |   %  |   ^  | Bksp |  up  | Del  |   )  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Del  |  SFT |  CTL |  OS  | ALT  |      |      | left | down | rght |   }  |  |   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      | Entr |ISO | | Home | End  |      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      |      | Next | Vol- | Vol+ | Play |
   * `-----------------------------------------------------------------------------------'
   */
  [_LOWER] = LAYOUT_planck_grid(
      KC_TILD, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, KC_CIRC, KC_BSPC, KC_UP,      KC_DEL,  KC_RPRN, KC_BSPC,
      KC_DEL,  KC_LSFT, KC_LCTL, KC_LGUI, KC_LALT, _______, _______, KC_LEFT, KC_DOWN,    KC_RGHT, KC_RCBR, KC_PIPE,
      _______, _______, _______, _______, _______, _______, _______, KC_ENT,  S(KC_NUBS), KC_HOME, KC_END,  _______,
      _______, _______, _______, _______, _______, _______, _______, _______, KC_MNXT,    KC_VOLD, KC_VOLU, KC_MPLY
  ),

  /* Raise / Numbers
   * ,-----------------------------------------------------------------------------------.
   * |   `  |   1  |   2  |   3  |   4  |   5  | Bksp |   7  |   8  |   9  |   *  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Del  |  SFT |  CTL |  OS  | ALT  |      |   T  |   4  |   5  |   6  |   +  |  \   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |   :  |   1  |   2  |   3  |   -  |      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |  Spc |   0  |   ,  |   .  |   /  | Play |
   * `-----------------------------------------------------------------------------------'
   */
  [_RAISE] = LAYOUT_planck_grid(
      KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_BSPC,   KC_7, KC_8,    KC_9,    KC_PAST, KC_BSPC,
      KC_DEL,  KC_LSFT, KC_LCTL, KC_LGUI, KC_LALT, _______, S(KC_T),   KC_4, KC_5,    KC_6,    KC_PPLS, KC_BSLS,
      _______, _______, _______, _______, _______, _______, S(KC_DOT), KC_1, KC_2,    KC_3,    KC_PMNS, _______,
      _______, _______, _______, _______, _______, _______, KC_SPC,    KC_0, KC_COMM, KC_DOT,  KC_PSLS, KC_MPLY
  ),

  /* Symbols #WIP
   * ,-----------------------------------------------------------------------------------.
   * |      | ö    | ä    | ß    | ü    | \    | ^    |      |      |      |      |      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      | [    | ]    | (    | )    | |    | ´    |  ALT |   OS |  CTL |  SFT |      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      | <    | >    | {    | }    | /    |      |      |      |      |      |      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      |      |      |      |      |      |
   * `-----------------------------------------------------------------------------------'
   */
  [_SYMBOLS] = LAYOUT_planck_grid(
      _______, KC_SCLN, KC_QUOT, KC_SLSH, KC_LBRC, _______, _______, _______, _______, _______, _______, _______,
      _______, _______, _______, _______, _______, _______, _______, KC_RALT, KC_RGUI, KC_RCTL, KC_RSFT, _______,
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
  ),

  /* Plover layer (http://opensteno.org)
   * ,-----------------------------------------------------------------------------------.
   * |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |   #  |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |   S  |   T  |   P  |   H  |   *  |   *  |   F  |   P  |   L  |   T  |   D  |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |   S  |   K  |   W  |   R  |   *  |   *  |   R  |   B  |   G  |   S  |   Z  |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Exit |      |      |   A  |   O  |             |   E  |   U  |      |      |      |
   * `-----------------------------------------------------------------------------------'
   */
  [_PLOVER] = LAYOUT_planck_grid(
      KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1,    KC_1   ,
      XXXXXXX, KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC,
      XXXXXXX, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
      EXT_PLV, XXXXXXX, XXXXXXX, KC_C,    KC_V,    XXXXXXX, XXXXXXX, KC_N,    KC_M,    XXXXXXX, XXXXXXX, XXXXXXX
  ),

  /* Adjust (Lower + Raise)
   *                      v------------------------RGB CONTROL--------------------v
   * ,-----------------------------------------------------------------------------------.
   * |      | Reset|Debug | RGB  |RGBMOD| HUE+ | HUE- | SAT+ | SAT- |BRGTH+|BRGTH-|  Del |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |MUSmod|Aud on|Audoff|AGnorm|AGswap|Qwerty|Colemk|Dvorak|Plover|      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |Voice-|Voice+|Mus on|Musoff|MIDIon|MIDIof|TermOn|TermOf|      |      |      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |             |      |      |      |      |      |
   * `-----------------------------------------------------------------------------------'
   */
  [_ADJUST] = LAYOUT_planck_grid(
      _______, RESET,   DEBUG,   RGB_TOG, RGB_MOD, RGB_HUI, RGB_HUD, RGB_SAI, RGB_SAD,  RGB_VAI, RGB_VAD, KC_DEL ,
      _______, _______, MU_MOD,  AU_ON,   AU_OFF,  AG_NORM, AG_SWAP, QWERTY,  COLEMAK,  DVORAK,  PLOVER,  _______,
      _______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  TERM_ON, TERM_OFF, _______, _______, _______,
      _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
  )

};

#ifdef AUDIO_ENABLE
  float plover_song[][2]     = SONG(PLOVER_SOUND);
  float plover_gb_song[][2]  = SONG(PLOVER_GOODBYE_SOUND);
#endif

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case QWERTY:
      if (record->event.pressed) {
        print("mode just switched to qwerty and this is a huge string\n");
        set_single_persistent_default_layer(_QWERTY);
      }
      return false;
      break;
    case COLEMAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_COLEMAK);
      }
      return false;
      break;
    case DVORAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_DVORAK);
      }
      return false;
      break;
    case BACKLIT:
      if (record->event.pressed) {
        register_code(KC_RSFT);
        #ifdef BACKLIGHT_ENABLE
          backlight_step();
        #endif
        #ifdef KEYBOARD_planck_rev5
          writePinLow(E6);
        #endif
      } else {
        unregister_code(KC_RSFT);
        #ifdef KEYBOARD_planck_rev5
          writePinHigh(E6);
        #endif
      }
      return false;
      break;
    case PLOVER:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          stop_all_notes();
          PLAY_SONG(plover_song);
        #endif
        layer_off(_RAISE);
        layer_off(_LOWER);
        layer_off(_ADJUST);
        layer_on(_PLOVER);
        if (!eeconfig_is_enabled()) {
            eeconfig_init();
        }
        keymap_config.raw = eeconfig_read_keymap();
        keymap_config.nkro = 1;
        eeconfig_update_keymap(keymap_config.raw);
      }
      return false;
      break;
    case EXT_PLV:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(plover_gb_song);
        #endif
        layer_off(_PLOVER);
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

bool encoder_update_user(uint8_t index, bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
    return true;
}

bool dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0: {
#ifdef AUDIO_ENABLE
            static bool play_sound = false;
#endif
            if (active) {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_song); }
#endif
                layer_on(_ADJUST);
            } else {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_gb_song); }
#endif
                layer_off(_ADJUST);
            }
#ifdef AUDIO_ENABLE
            play_sound = true;
#endif
            break;
        }
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
    return true;
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
